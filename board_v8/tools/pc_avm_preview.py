"""PC-only AVM visual experiment; extrapolated pixels are NOT measured coverage.

Uses the same 256x256 calibrated maps and feathering as AvmRenderer. The
optional presentation fill leaves observed pixels and calibration masks intact.
"""
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
NAMES = ('front', 'rear', 'left', 'right', 'rear_left', 'rear_right', 'front_tele')
DEFAULT_SCENE = Path('D:/brige/multi_camera_material/adas_demo_groups/11_side_vehicle_bsd/previous_simulated_input/66b5fa4b_30fps')


class Renderer:
    def __init__(self, calibration):
        fs = cv2.FileStorage(str(calibration), cv2.FILE_STORAGE_READ)
        if not fs.isOpened() or int(fs.getNode('version').real()) != 2:
            raise ValueError('A version-2 calibrated map is required')
        self.ppm = fs.getNode('pixels_per_metre').real()
        self.length = fs.getNode('vehicle_length').real()
        self.clip_id = fs.getNode('clip_id').string()
        self.maps = {}
        for name in NAMES:
            node = fs.getNode(name)
            if node.empty():
                continue
            mx, my, mask = (node.getNode(k).mat() for k in ('map_x', 'map_y', 'mask'))
            mask = mask.copy()
            mask[(mx < 0) | (mx > 1) | (my < 0) | (my > 1)] = 0
            padded = cv2.copyMakeBorder(mask, 1, 1, 1, 1, cv2.BORDER_CONSTANT, value=0)
            weight = np.minimum(cv2.distanceTransform(padded, cv2.DIST_L2, 3)[1:-1, 1:-1], 8)
            self.maps[name] = (mx, my, weight)
        fs.release()
        self.asset = cv2.imread(str(ROOT / 'assets/avm_car_top.png'), cv2.IMREAD_UNCHANGED)
        if self.asset is None or self.asset.shape[2] != 4:
            raise ValueError('Missing RGBA vehicle asset')
        self.fill_key = None
        self.surround_guides = False
        self.guide_cache = {}
        self.display_crop = 640
        self.seam_refinement = False
        self.seam_cache = {}
        self.color_offsets = None
        self.offset_cameras = None
        self.antialias_ground = False
        self.sampling_cache = {}

    def sample_ground(self, name, frame):
        mx, my, feather = self.maps[name]
        height, width = frame.shape[:2]
        if not self.antialias_ground:
            return cv2.remap(frame, mx*(width-1), my*(height-1), cv2.INTER_LINEAR)
        key = name, width, height
        if key not in self.sampling_cache:
            # Integrate a ground pixel's footprint rather than sampling one
            # source point. Maps and measured coverage remain unchanged.
            size = (mx.shape[1]*4, mx.shape[0]*4)
            sx = cv2.resize(mx, size, interpolation=cv2.INTER_LINEAR)*(width-1)/2
            sy = cv2.resize(my, size, interpolation=cv2.INTER_LINEAR)*(height-1)/2
            valid = cv2.resize((feather>0).astype(np.float32), size,
                               interpolation=cv2.INTER_NEAREST)
            valid *= (sx>=0)&(sx<=(width-1)/2)&(sy>=0)&(sy<=(height-1)/2)
            count = cv2.resize(valid, (mx.shape[1],mx.shape[0]), interpolation=cv2.INTER_AREA)
            self.sampling_cache[key] = sx, sy, valid, count
        sx, sy, valid, count = self.sampling_cache[key]
        # pyrDown retains source pixel origin; use half coordinates, not a
        # resized-image width ratio. Camera tiles still use full-resolution RGB.
        source = cv2.pyrDown(frame)
        sampled = cv2.remap(source, sx, sy, cv2.INTER_LINEAR).astype(np.float32)
        total = cv2.resize(sampled*valid[...,None], (mx.shape[1],mx.shape[0]),
                           interpolation=cv2.INTER_AREA)
        image = np.clip(total/np.maximum(count[...,None],1e-5),0,255).astype(np.uint8)
        missing = (feather>0)&(count<=1e-5)
        if missing.any():
            point = cv2.remap(frame,mx*(width-1),my*(height-1),cv2.INTER_LINEAR)
            image[missing] = point[missing]
        return image

    def ground(self, frames):
        if self.seam_refinement:
            return self.refined_ground(frames)
        total = np.zeros((256, 256, 3), np.float32)
        weights = np.zeros((256, 256), np.float32)
        for name, frame in frames.items():
            if name not in self.maps:
                continue
            mx, my, weight = self.maps[name]
            warped = self.sample_ground(name, frame)
            total += warped * weight[..., None]
            weights += weight
        observed = weights > 1e-5
        image = np.clip(total / np.maximum(weights[..., None], 1e-5), 0, 255).astype(np.uint8)
        return image, observed

    def seam_weights(self, active):
        """Static calibrated camera selection, shared by rendering and inspection."""
        if active not in self.seam_cache:
            # Minimum Jacobian singular value measures source pixels per ground
            # step. Prefer less grazing, better sampled views; keep a narrow
            # seam instead of averaging all cameras across whole overlaps.
            scores=[]
            for name,width,height in active:
                mx,my,feather=self.maps[name]
                ux,uy=np.gradient(mx*(width-1))
                vx,vy=np.gradient(my*(height-1))
                trace=ux*ux+uy*uy+vx*vx+vy*vy
                det=(ux*vy-uy*vx)**2
                sigma=np.sqrt(np.maximum(0,.5*(trace-np.sqrt(np.maximum(0,trace*trace-4*det)))))
                scores.append(np.maximum(sigma,.001)*(feather>0))
            owner=np.argmax(np.stack(scores),axis=0)
            # Three pixels on each side at 256 square: only the seam blends.
            weights=[cv2.GaussianBlur((owner==i).astype(np.float32),(7,7),1.5)
                     *(self.maps[name][2]>0) for i,(name,_,_) in enumerate(active)]
            self.seam_cache[active]=weights
        return self.seam_cache[active]

    def refined_ground(self, frames):
        active = tuple((name, frame.shape[1], frame.shape[0])
                       for name, frame in frames.items() if name in self.maps)
        if not active:
            return np.zeros((256,256,3),np.uint8),np.zeros((256,256),bool)
        weights=self.seam_weights(active)
        warped=[]
        for name,width,height in active:
            mx,my,_=self.maps[name]
            warped.append(self.sample_ground(name, frames[name]))
        # Match exposure from low-gradient overlap pixels. Solve pairwise
        # differences with camera zero as the reference; bounded offsets and
        # temporal smoothing avoid brightness pumping and color replacement.
        equations,targets=[],[]
        gray=[cv2.cvtColor(image,cv2.COLOR_BGR2GRAY) for image in warped]
        flat=[cv2.morphologyEx(g,cv2.MORPH_GRADIENT,np.ones((3,3),np.uint8))<12 for g in gray]
        for i in range(len(active)):
            for j in range(i+1,len(active)):
                overlap=(self.maps[active[i][0]][2]>0)&(self.maps[active[j][0]][2]>0)
                overlap&=flat[i]&flat[j]&(gray[i]>20)&(gray[j]>20)&(gray[i]<230)&(gray[j]<230)
                if np.count_nonzero(overlap)<200:
                    continue
                row=np.zeros(len(active),np.float32);row[i]=-1;row[j]=1
                equations.append(row)
                targets.append(np.median(warped[i][overlap].astype(np.float32)-warped[j][overlap],axis=0))
        offsets=np.zeros((len(active),3),np.float32)
        if equations:
            anchor=np.zeros(len(active),np.float32);anchor[0]=10
            offsets=np.linalg.lstsq(np.vstack([*equations,anchor]),np.vstack([*targets,[0,0,0]]),rcond=None)[0]
            offsets=np.clip(offsets,-12,12)
        if self.color_offsets is not None and self.offset_cameras == active:
            offsets=self.color_offsets*.85+offsets*.15
        self.color_offsets=offsets
        self.offset_cameras=active
        total=np.zeros((256,256,3),np.float32)
        summed=np.zeros((256,256),np.float32)
        for image,weight,offset in zip(warped,weights,offsets):
            total+=np.clip(image.astype(np.float32)+offset,0,255)*weight[...,None]
            summed+=weight
        observed=summed>1e-5
        image=np.clip(total/np.maximum(summed[...,None],1e-5),0,255).astype(np.uint8)
        return image,observed

    def extend(self, image, observed, limit_metres=3.5):
        if not observed.any():
            return image.copy(), np.zeros_like(observed)
        key = observed.tobytes(), limit_metres
        if key != self.fill_key:
            # Only actual mapped ground can seed extension. No sky/body pixels.
            distance, labels = cv2.distanceTransformWithLabels((~observed).astype(np.uint8), cv2.DIST_L2, 5, labelType=cv2.DIST_LABEL_PIXEL)
            yy, xx = np.indices(observed.shape)
            seed_x = np.zeros(int(labels.max()) + 1, np.float32)
            seed_y = seed_x.copy()
            seed_x[labels[observed]] = xx[observed]
            seed_y[labels[observed]] = yy[observed]
            nx, ny = seed_x[labels], seed_y[labels]
            # Reflect into the neighbouring valid texture instead of stretching
            # one border pixel into a long streak. Fall back to the nearest seed.
            rx = np.clip(2*nx - xx, 0, 255).astype(np.float32)
            ry = np.clip(2*ny - yy, 0, 255).astype(np.float32)
            usable = observed[ry.astype(int), rx.astype(int)]
            rx[~usable], ry[~usable] = nx[~usable], ny[~usable]
            limit = limit_metres * self.ppm * .4
            fade = np.clip((limit-distance) / max(1., self.ppm*.4*.5), 0, 1)
            fade[observed] = 0
            self.fill_maps = rx, ry, fade
            self.fill_key = key
        rx, ry, fade = self.fill_maps
        texture = cv2.remap(image, rx, ry, cv2.INTER_LINEAR)
        smooth = cv2.GaussianBlur(texture, (5, 5), .9)
        # Preserve the seam; soften only the interior of the approximate fill.
        texture = cv2.addWeighted(texture, .6, smooth, .4, 0)
        result = image.copy()
        fill = (~observed) & (fade > 0)
        result[fill] = texture[fill]
        return result, fill

    def view(self, ground, observed, enhanced=True):
        image, filled = self.extend(ground, observed) if enhanced else (ground.copy(), np.zeros_like(observed))
        yy, xx = np.indices(observed.shape)
        dark = np.where((((xx+yy)//12) % 2)[..., None] == 0, np.array([30,38,45]), np.array([22,29,35])).astype(np.uint8)
        if enhanced:
            fade = self.fill_maps[2] if observed.any() else np.zeros(observed.shape)
            image[filled] = (image[filled] * fade[filled,None] + dark[filled] * (1-fade[filled,None])).astype(np.uint8)
        image[~(observed | filled)] = dark[~(observed | filled)]
        image = cv2.resize(image, (640, 640), interpolation=cv2.INTER_LINEAR)
        scale = 1.55 if enhanced else 1.
        height = round(self.length * self.ppm * scale)
        width = round(self.asset.shape[1] / self.asset.shape[0] * height)
        sprite = cv2.resize(self.asset, (width, height), interpolation=cv2.INTER_AREA)
        x, y = 320-width//2, 320-height//2
        if self.surround_guides:
            # Static display guides around the sprite, not measured clearances,
            # detected obstacles or a predicted steering trajectory.
            key = width, height
            if key not in self.guide_cache:
                lines = np.zeros_like(image)
                left, right, top, bottom = x-20, x+width+20, y-18, y+height+18
                for margin in [0, 13]:
                    color = (214, 173, 45) if margin == 0 else (135, 100, 27)
                    for points in [
                        [(left-margin,top+32),(left-margin,top+6),(left+14,top-margin)],
                        [(right+margin,top+32),(right+margin,top+6),(right-14,top-margin)],
                        [(left-margin,bottom-24),(left-margin,bottom+2),(left+14,bottom+margin),
                         (right-14,bottom+margin),(right+margin,bottom+2),(right+margin,bottom-24)]]:
                        cv2.polylines(lines,[np.asarray(points,np.int32)],False,color,1,cv2.LINE_AA)
                self.guide_cache[key] = cv2.addWeighted(lines,.75,cv2.GaussianBlur(lines,(9,9),2),.65,0)
            image = cv2.add(image,self.guide_cache[key])
        alpha = sprite[...,3:4].astype(np.float32)/255
        # Soft vehicle silhouette shadow, independent of the ground masks.
        shadow = np.zeros((640,640), np.uint8)
        shadow[y:y+height,x:x+width] = sprite[...,3]
        shadow = cv2.GaussianBlur(shadow, (21,21), 5).astype(np.float32)/255*.45
        image = (image*(1-shadow[...,None])).astype(np.uint8)
        image[y:y+height,x:x+width] = (sprite[...,:3]*alpha + image[y:y+height,x:x+width]*(1-alpha)).astype(np.uint8)
        if self.display_crop != 640:
            if not 320 <= self.display_crop <= 640:
                raise ValueError('Display crop must be between 320 and 640 pixels')
            margin = (640-self.display_crop)//2
            image = cv2.resize(image[margin:margin+self.display_crop,margin:margin+self.display_crop],
                               (640,640),interpolation=cv2.INTER_LINEAR)
        for text, pos in [('FRONT',(289,24)),('REAR',(295,622)),('LEFT',(12,320)),('RIGHT',(565,320))]:
            cv2.putText(image,text,pos,cv2.FONT_HERSHEY_SIMPLEX,.6,(230,225,215),1,cv2.LINE_AA)
        return image, filled


class Replay:
    def __init__(self, scene, extras):
        self.caps, self.next = {}, {}
        for name in NAMES:
            path = (scene if name in NAMES[:4] else extras) / (name+'.mp4')
            cap = cv2.VideoCapture(str(path))
            if not cap.isOpened():
                raise ValueError(f'Missing synchronized camera video: {path}')
            self.caps[name], self.next[name] = cap, 0
        self.fps = self.caps['front'].get(cv2.CAP_PROP_FPS)
        counts = [int(c.get(cv2.CAP_PROP_FRAME_COUNT)) for c in self.caps.values()]
        rates = [c.get(cv2.CAP_PROP_FPS) for c in self.caps.values()]
        if any(abs(f-self.fps) > .01 for f in rates):
            raise ValueError('Extra cameras must first be timestamp-aligned to the main replay')
        self.count = min(counts)

    def read(self, index):
        frames = {}
        for name, cap in self.caps.items():
            if index < self.next[name] or index-self.next[name] > 45:
                cap.set(cv2.CAP_PROP_POS_FRAMES, index)
                self.next[name] = index
            while self.next[name] < index:
                if not cap.grab():
                    raise RuntimeError(f'{name}: video ended before frame {index}')
                self.next[name] += 1
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError(f'{name}: could not decode frame {index}')
            self.next[name] += 1
            frames[name] = frame
        return frames

    def close(self):
        for cap in self.caps.values():
            cap.release()


def panel(frames, bev, enhanced, seconds, duration, paused=False, fullscreen=False, driving=None):
    canvas = np.full((840,1440,3),(17,25,34),np.uint8)
    cv2.putText(canvas,'RoadFusion | PC AVM Preview',(22,39),0,.85,(238,218,164),2,cv2.LINE_AA)
    mode = 'TEXTURE EXTENSION + CAR 1.55x (APPROXIMATE)' if enhanced else 'CALIBRATED GROUND ONLY / TRUE COVERAGE'
    cv2.putText(canvas,mode,(22,72),0,.54,(65,210,240) if enhanced else (170,210,160),1,cv2.LINE_AA)
    if fullscreen:
        canvas[100:780,380:1060] = cv2.resize(bev,(680,680))
    else:
        for name, (x,y) in zip(NAMES[:4],[(20,110),(415,110),(20,390),(415,390)]):
            canvas[y:y+216,x:x+384] = cv2.resize(frames[name],(384,216))
            cv2.rectangle(canvas,(x,y),(x+100,y+29),(32,47,62),-1)
            cv2.putText(canvas,name.upper(),(x+8,y+21),0,.6,(238,228,211),1,cv2.LINE_AA)
        if driving is None:
            canvas[110:710,820:1420] = cv2.resize(bev,(600,600))
        else:
            # Keep all four primary camera tiles equal, and preserve AVM.
            cv2.putText(canvas,'CAMERA BEV / AVM',(835,105),0,.5,(222,218,188),1,cv2.LINE_AA)
            canvas[116:426,820:1130] = cv2.resize(bev,(310,310))
            cv2.rectangle(canvas,(819,115),(1131,427),(100,67,43),1,cv2.LINE_AA)
            canvas[110:790,1140:1420] = cv2.resize(driving,(280,680))
            cv2.rectangle(canvas,(1139,109),(1421,791),(131,53,112),1,cv2.LINE_AA)
            for text,pos in [('7 synchronized cameras',(830,465)),('AVM: original vehicle pixels',(830,495)),('Visual ground extension',(830,524)),('V: enlarge camera BEV',(830,554)),('Driving inset: real UFLDv2',(830,624)),('YOLO vehicle symbols',(830,654)),('2.5 Hz cached perception',(830,684)),('Schematic; depth not measured',(830,714))]:
                cv2.putText(canvas,text,pos,0,.44,(182,177,166),1,cv2.LINE_AA)
        for name, x in zip(NAMES[4:],[20,280,540]):
            canvas[657:789,x:x+244] = cv2.resize(frames[name],(244,132))
            cv2.putText(canvas,name.upper(),(x+6,680),0,.45,(240,240,240),1,cv2.LINE_AA)
        if driving is None:
            cv2.putText(canvas,'7 synchronized cameras | No YOLO/UFLD in this AVM visual test',(825,739),0,.43,(156,173,185),1,cv2.LINE_AA)
            cv2.putText(canvas,'Dark hatch = still unknown; extension is visual only',(825,765),0,.44,(100,190,225),1,cv2.LINE_AA)
    cv2.putText(canvas,f'{seconds:04.1f} / {duration:.1f}s   '+('PAUSED' if paused else '1.0x'),(22,824),0,.55,(230,223,207),1,cv2.LINE_AA)
    keys='SPACE pause | B compare | V enlarge | '+('D driving | ' if driving is not None else '')+'R restart | Q exit'
    cv2.putText(canvas,keys,(415,824),0,.5,(205,198,169),1,cv2.LINE_AA)
    return canvas


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene',type=Path,default=DEFAULT_SCENE)
    parser.add_argument('--extras',type=Path,default=ROOT/'output/avm_extra_320')
    parser.add_argument('--calibration',type=Path,default=ROOT/'output/calibrated_bev/seven.yml')
    parser.add_argument('--output',type=Path,default=ROOT/'output/pc_avm_preview')
    parser.add_argument('--snapshot',type=float,help='Save a comparison at this replay second and exit')
    parser.add_argument('--start-seconds',type=float,default=0,help='Start playback at this source time')
    parser.add_argument('--driving-view',action='store_true',help='Add a separate UFLDv2/YOLO perception inset')
    parser.add_argument('--surround-guides',action='store_true',help='Static cyan display guides; not distance measurements')
    parser.add_argument('--near-view',action='store_true',help='Show the central 80 percent of the calibrated ground extent')
    parser.add_argument('--driving-cache',type=Path,default=ROOT/'output/pc_avm_preview/driving_cache.json')
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    renderer, replay = Renderer(args.calibration), Replay(args.scene,args.extras)
    renderer.surround_guides = args.surround_guides
    renderer.display_crop = 512 if args.near_view else 640
    driving_data=None
    if args.driving_view:
        from pc_driving_scene import render_scene
        driving_data=json.loads(args.driving_cache.read_text())
        if driving_data['scene']!=str(args.scene.resolve()) or driving_data['frames']!=replay.count or abs(driving_data['fps']-replay.fps)>.01:
            replay.close()
            raise ValueError('Driving perception cache belongs to a different replay')
    def driving_at(seconds):
        if driving_data is None:
            return None
        index=min(replay.count-1,int(seconds*replay.fps))
        index=(index//driving_data['step'])*driving_data['step']
        return render_scene(driving_data['records'][str(index)],seconds,(280,680))
    try:
        if args.snapshot is not None:
            frames = replay.read(min(replay.count-1,round(args.snapshot*replay.fps)))
            ground, observed = renderer.ground(frames)
            before, _ = renderer.view(ground,observed,False)
            after, fill = renderer.view(ground,observed,True)
            # Check presentation extension cannot alter observed ground pixels.
            extended, filled = renderer.extend(ground,observed)
            assert np.array_equal(extended[observed],ground[observed])
            assert not np.any(filled & observed)
            empty = np.zeros_like(observed)
            assert not renderer.extend(np.zeros_like(ground),empty)[1].any()
            stamp = f'{args.snapshot:04.1f}s'
            cv2.imwrite(str(args.output/f'comparison_{stamp}.jpg'),np.concatenate([before,after],axis=1))
            cv2.imwrite(str(args.output/f'ui_{stamp}.jpg'),panel(frames,after,True,args.snapshot,replay.count/replay.fps,driving=driving_at(args.snapshot)))
            stats = {'seconds':args.snapshot,'observed_pixels':int(observed.sum()),'approximate_fill_pixels':int(fill.sum()),'still_unknown_pixels':int((~(observed|fill)).sum()),'car_display_scale':1.55,'observed_pixels_unchanged':True}
            (args.output/f'coverage_{stamp}.json').write_text(json.dumps(stats,indent=2))
            print(json.dumps(stats),flush=True)
            return
        title = 'RoadFusion PC | 7-camera AVM | B compare, V enlarge'
        cv2.namedWindow(title,cv2.WINDOW_NORMAL)
        cv2.resizeWindow(title,1440,840)
        cv2.moveWindow(title,40,55)
        enhanced, paused, large = True,False,False
        driving_visible = args.driving_view
        start = time.perf_counter()-args.start_seconds
        held = args.start_seconds
        last = -1
        print('PC AVM preview started. Seven synchronized cameras, wall-clock 1.0x replay.',flush=True)
        while True:
            elapsed = held if paused else (time.perf_counter()-start) % (replay.count/replay.fps)
            index = min(replay.count-1,int(elapsed*replay.fps))
            if index != last:
                frames = replay.read(index)
                ground, observed = renderer.ground(frames)
                last = index
            bev,_ = renderer.view(ground,observed,enhanced)
            ui = panel(frames,bev,enhanced,elapsed,replay.count/replay.fps,paused,large,driving=driving_at(elapsed) if driving_visible else None)
            cv2.imshow(title,ui)
            key = cv2.waitKey(1) & 255
            if key in (27,ord('q')) or cv2.getWindowProperty(title,cv2.WND_PROP_VISIBLE)<1:
                break
            if key == ord(' '):
                if paused:
                    start = time.perf_counter()-held
                else:
                    held = elapsed
                paused = not paused
            if key == ord('b'):
                enhanced = not enhanced
            if key == ord('v'):
                large = not large
            if key == ord('d') and driving_data:
                driving_visible = not driving_visible
            if key == ord('r'):
                start = time.perf_counter()
                held = 0.
            if key == ord('s'):
                cv2.imwrite(str(args.output/'live_window.jpg'),ui)
            time.sleep(.005)
    finally:
        replay.close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
