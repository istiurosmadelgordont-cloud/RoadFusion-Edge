import cv2, numpy as np, math, json, argparse
from pathlib import Path
parser=argparse.ArgumentParser(description='Approximate BEV experiment, NOT measured calibration')
parser.add_argument('--scene',required=True,type=Path)
parser.add_argument('--output',required=True,type=Path)
parser.add_argument('--seconds',type=float,default=4)
args=parser.parse_args()
out=args.output;out.mkdir(parents=True,exist_ok=True)
root=args.scene
frames=[]; data={}
# Explicit engineering assumptions, not measured calibration.
for name,yaw,fov,ox,oy in [('front',0,120,0,2),('rear',180,30,0,-2),('left',-90,120,-1,0),('right',90,120,1,0)]:
    c=cv2.VideoCapture(str(root/(name+'.mp4')));c.set(cv2.CAP_PROP_POS_MSEC,args.seconds*1000);ok,f=c.read();c.release();assert ok
    cv2.imwrite(str(out/(name+'.jpg')),f);frames.append(f)
    a=math.radians(yaw);pitch=math.radians(0);h=1.6
    forward=np.array([math.sin(a)*math.cos(pitch),math.cos(a)*math.cos(pitch),-math.sin(pitch)])
    right=np.array([math.cos(a),-math.sin(a),0]);down=np.cross(forward,right)
    fx=.5/math.tan(math.radians(fov/2));fy=fx*f.shape[1]/f.shape[0]
    image=np.array([[.05,.76],[.95,.76],[.95,.91 if name=="front" else .98],[.05,.91 if name=="front" else .98]],np.float32)
    ground=[]
    for u,v in image:
        ray=forward+right*(u-.5)/fx+down*(v-(.45 if name=="rear" else .67))/fy
        t=-h/ray[2];pt=np.array([ox,oy,h])+t*ray;ground.append(pt[:2])
    data[name]={'image_points':image.tolist(),'ground_points':np.array(ground).tolist()}
(out/'assumed_points.json').write_text(json.dumps(data,indent=2))
total=np.zeros((640,640,3),np.float32);weights=np.zeros((640,640),np.float32)
for f,(name,d) in zip(frames,data.items()):
    src=np.array(d['image_points'],np.float32)*[f.shape[1]-1,f.shape[0]-1]
    dst=np.array(d['ground_points'],np.float32)*[25,-25]+[320,320]
    H=cv2.getPerspectiveTransform(src.astype(np.float32),dst.astype(np.float32))
    mask=np.zeros(f.shape[:2],np.uint8);cv2.fillConvexPoly(mask,src.astype(np.int32),255)
    m=cv2.warpPerspective(mask,H,(640,640));w=np.minimum(cv2.distanceTransform(m,cv2.DIST_L2,3),20)
    total+=cv2.warpPerspective(f,H,(640,640))*w[:,:,None];weights+=w
im=np.uint8(total/np.maximum(weights[:,:,None],1e-6));im[weights==0]=[30,38,45]
cv2.rectangle(im,(295,261),(345,379),(190,190,190),-1)
cv2.putText(im,'APPROXIMATE - ASSUMED CAMERA POSE',(12,26),0,.55,(0,210,255),1)
cv2.imwrite(str(out/'prototype.jpg'),im)
