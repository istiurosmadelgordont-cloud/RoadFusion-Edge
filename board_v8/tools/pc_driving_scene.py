"""Driving perception inset for the PC preview, separate from camera AVM.

Lane curves come from UFLDv2; target symbols come from YOLO detections.
This perspective display is schematic, not measured depth or automatic driving.
"""
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def decode_lanes(outputs):
    loc_row,loc_col,exist_row,exist_col = outputs
    if [a.shape for a in outputs] != [(1,200,72,4),(1,100,81,4),(1,2,72,4),(1,2,81,4)]:
        raise ValueError('Unexpected UFLDv2 output schema')
    curves=[]
    for slot in range(4):
        column=slot in (0,3)
        loc,exist=(loc_col,exist_col) if column else (loc_row,exist_row)
        grids,anchors=loc.shape[1:3]
        points=[]
        for a in range(anchors):
            if exist[0,1,a,slot]<=exist[0,0,a,slot]:
                continue
            best=int(np.argmax(loc[0,:,a,slot]))
            low,high=max(0,best-1),min(grids-1,best+1)
            values=loc[0,low:high+1,a,slot]
            prob=np.exp(values-values.max())
            pos=float(np.dot(prob,np.arange(low,high+1)+.5)/prob.sum())
            x=a/(anchors-1)*640 if column else pos/(grids-1)*640
            y=pos/(grids-1)*360 if column else (.42+.58*a/(anchors-1))*360
            if 0<=x<640 and 145<=y<360:
                points.append([x,y])
        if len(points)<5 or len(points)/anchors<.2:
            curves.append([])
            continue
        p=np.asarray(points)
        if np.ptp(p[:,1])<35:
            curves.append([])
            continue
        fit=np.polyfit(p[:,1],p[:,0],2)
        yy=np.linspace(p[:,1].min(),p[:,1].max(),40)
        curves.append(np.stack([np.polyval(fit,yy),yy],axis=1).tolist())
    return curves


def infer_lanes(net,frame):
    image=cv2.resize(frame,(1600,533))[-320:]
    rgb=cv2.cvtColor(image,cv2.COLOR_BGR2RGB).astype(np.float32)
    normalized=((rgb-[123.675,116.28,103.53])/[58.395,57.12,57.375]).astype(np.float32)
    net.setInput(np.ascontiguousarray(normalized.transpose(2,0,1)[None]))
    return decode_lanes(net.forward(['loc_row','loc_col','exist_row','exist_col']))


def blend_polygon(image,poly,color,alpha):
    mask=np.zeros(image.shape[:2],np.uint8)
    cv2.fillPoly(mask,[np.asarray(poly,np.int32)],255,cv2.LINE_AA)
    amount=mask[...,None].astype(np.float32)/255*alpha
    image[:]=(image*(1-amount)+np.array(color)*amount).astype(np.uint8)


def vehicle(image,x,y,width,height,ego=False):
    """Lightweight perspective vehicle symbol; camera AVM remains unchanged."""
    w,h=max(5,int(width)),max(9,int(height))
    x,y=int(x),int(y)
    halo=np.zeros_like(image)
    cv2.ellipse(halo,(x,y+3),(w,h//4),0,0,360,(160,80,35) if ego else (62,39,60),-1,cv2.LINE_AA)
    halo=cv2.GaussianBlur(halo,(19,19),5)
    cv2.add(image,halo,dst=image)
    outline=np.array([[x-w//2,y],[x-w//2-1,y-h//3],[x-w//3,y-h],[x+w//3,y-h],[x+w//2+1,y-h//3],[x+w//2,y]],np.int32)
    cv2.fillPoly(image,[outline],(200,195,201) if ego else (190,171,211),cv2.LINE_AA)
    roof=np.array([[x-w//3,y-h//3],[x-w//4,y-h*3//4],[x+w//4,y-h*3//4],[x+w//3,y-h//3]],np.int32)
    cv2.fillPoly(image,[roof],(49,34,52),cv2.LINE_AA)
    cv2.line(image,(x-w//4,y-h+2),(x+w//4,y-h+2),(246,225,237),1,cv2.LINE_AA)
    cv2.line(image,(x-w//2+2,y-4),(x+w//2-2,y-4),(80,80,251) if ego else (167,98,250),2,cv2.LINE_AA)
    cv2.polylines(image,[outline],True,(246,223,223) if ego else (240,174,238),1,cv2.LINE_AA)


def render_scene(record,seconds,size=(360,380)):
    width,height=size
    scale=2
    w,h=width*scale,height*scale
    yy=np.linspace(0,1,h)[:,None,None]
    top=np.array([25,12,16],np.float32)
    bottom=np.array([48,15,21],np.float32)
    image=np.tile((top*(1-yy)+bottom*yy).astype(np.uint8),(1,w,1))
    # Very soft violet atmosphere, kept behind perception geometry.
    glow=np.zeros_like(image)
    cv2.ellipse(glow,(w//2,h//2),(w//3,h//2),0,0,360,(65,8,39),-1)
    glow=cv2.GaussianBlur(glow,(121,121),35)
    image=cv2.add(image,glow)
    lane_points=[p for p in record.get('lanes',[]) if p]
    source_far=min(p[0][1] for p in lane_points) if lane_points else 145.
    source_near=max(p[-1][1] for p in lane_points) if lane_points else 360.
    source_span=max(35.,source_near-source_far)
    def project(points):
        p=np.array(points,np.float32).copy()
        p[:,0]=(p[:,0]/640-.5)*w*.95+w*.5
        p[:,1]=h*.13+(p[:,1]-source_far)/source_span*h*.81
        return p.astype(np.int32)
    curves=[project(p) if p else None for p in record.get('lanes',[[],[],[],[]])]
    observed=curves[1] is not None and curves[2] is not None
    if observed:
        left,right=curves[1],curves[2]
        far=max(left[0,1],right[0,1]);near=min(left[-1,1],right[-1,1])
        ys=np.linspace(far,near,40)
        lx=np.interp(ys,left[:,1],left[:,0]);rx=np.interp(ys,right[:,1],right[:,0])
        if np.all(rx-lx>4):
            blend_polygon(image,np.vstack([np.stack([lx,ys],1),np.stack([rx[::-1],ys[::-1]],1)]),(142,42,79),.35)
            centres=(lx+rx)*.5
            for j in range(6,34,5):
                x=int(centres[j]);y=int(ys[j]);span=max(4,int((rx[j]-lx[j])*.11))
                arrow=np.array([[x-span,y+8],[x,y],[x+span,y+8],[x+span,y+13],[x,y+5],[x-span,y+13]])
                blend_polygon(image,arrow,(244,157,105),.6)
    glow=np.zeros_like(image)
    for slot,curve in enumerate(curves):
        if curve is None:
            continue
        color=(246,123,174) if slot in (0,3) else (237,173,213)
        cv2.polylines(glow,[curve],False,(122,28,79),12,cv2.LINE_AA)
        # Dashes express a schematic boundary, not solid/dashed classification.
        for k in range(len(curve)-1):
            if slot in (1,2) and (k//3)%2:
                continue
            cv2.line(image,tuple(curve[k]),tuple(curve[k+1]),color,3,cv2.LINE_AA)
    cv2.addWeighted(image,1,cv2.GaussianBlur(glow,(17,17),4),.65,0,image)
    for detection in sorted(record.get('vehicles',[]),key=lambda d:d[3]):
        x1,y1,x2,y2,kind,score=detection
        if kind not in (2,3,4) or score<.3 or y2<150:
            continue
        center=project([[(x1+x2)*.5,y2]])[0]
        depth=np.clip((y2-source_far)/source_span,0,1)
        carwidth=w*(.02+.09*depth)
        carheight=carwidth*(2 if kind==2 else 2.7)
        vehicle(image,*center,carwidth,carheight)
    vehicle(image,w*.5,h*.89,w*.15,w*.30,True)
    cv2.putText(image,'DRIVING PERCEPTION',(24,34),0,.65,(242,223,232),1,cv2.LINE_AA)
    label='UFLDv2 + YOLO | schematic' if observed else 'Lane unobserved | YOLO targets'
    cv2.putText(image,label,(24,h-18),0,.47,(214,161,189),1,cv2.LINE_AA)
    return cv2.resize(image,(width,height),interpolation=cv2.INTER_AREA)


def prepare(scene,output,yolo_cache,lane_model):
    from pc_avm_preview import Replay,ROOT as AVM_ROOT
    replay=Replay(scene,AVM_ROOT/'output/avm_extra_320')
    data=json.loads(yolo_cache.read_text())
    if data['scene']!=str(scene.resolve()) or data['frames']!=replay.count or abs(data['fps']-replay.fps)>.01:
        raise ValueError('YOLO cache belongs to a different replay')
    net=cv2.dnn.readNetFromONNX(str(lane_model))
    records={}
    start=time.perf_counter()
    try:
        for index in range(0,replay.count,12):
            frame=replay.read(index)['front']
            lanes=infer_lanes(net,frame)
            records[str(index)]={'lanes':lanes,'vehicles':data['detections'][str((index//6)*6)]['front']}
            if index%120==0:
                print(f'Driving inset: {index}/{replay.count}; {time.perf_counter()-start:.1f}s',flush=True)
        output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(json.dumps({'scene':str(scene.resolve()),'fps':replay.fps,'frames':replay.count,'step':12,'lane_model':str(lane_model),'records':records}))
        print(f'Driving inset cache ready: {len(records)} real UFLDv2 results',flush=True)
    finally:
        replay.close()


if __name__=='__main__':
    from pc_avm_preview import DEFAULT_SCENE
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,default=DEFAULT_SCENE)
    p.add_argument('--output',type=Path,default=ROOT/'output/pc_avm_preview/driving_cache.json')
    p.add_argument('--yolo-cache',type=Path,default=ROOT/'output/pc_avm_original_vehicles/vehicle_cache.json')
    p.add_argument('--lane-model',type=Path,default=ROOT.parent/'models/ufldv2_culane_res18_1600x320.onnx')
    args=p.parse_args()
    prepare(args.scene,args.output,args.yolo_cache,args.lane_model)
