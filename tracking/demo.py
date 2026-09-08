"""Deterministic textured room with metric RGB-D and known camera poses.

Only --demo imports this source. Live mode cannot substitute synthetic data.
"""
import time
import cv2
import numpy as np

K = np.array([[280., 0., 160.], [0., 280., 120.], [0., 0., 1.]])
_rng = np.random.default_rng(731)
_texture = _rng.integers(30, 220, (768, 768, 3), dtype=np.uint8)
_texture = cv2.GaussianBlur(_texture, (5, 5), 0)
for _ in range(1400):
    x,y = _rng.integers(0,768,2)
    color = tuple(int(c) for c in _rng.integers(20,245,3))
    cv2.circle(_texture,(int(x),int(y)),int(_rng.integers(2,9)),color,-1)
_v,_u = np.indices((240,320))
_rays = np.stack(((_u-160)/280,(_v-120)/280,np.ones_like(_u)),axis=-1)


def render(pose):
    """Raycast the inside of a 4 m wide room; depth is camera Z, metres."""
    rays = _rays @ pose[:3,:3].T
    origin = pose[:3,3]
    best = np.full((240,320),np.inf)
    surface = np.zeros((240,320),int)
    planes = [(0,-2.),(0,2.),(1,-1.5),(1,1.5),(2,4.)]
    for i,(axis,location) in enumerate(planes):
        with np.errstate(divide='ignore',invalid='ignore'):
            distance = (location-origin[axis])/rays[:,:,axis]
        valid = (distance>.1)&(distance<best)
        best[valid]=distance[valid];surface[valid]=i
    points = origin + rays*best[:,:,None]
    rgb = np.zeros((240,320,3),np.uint8)
    for i,(axis,_) in enumerate(planes):
        other=[a for a in range(3) if a!=axis]
        tx=np.floor(points[:,:,other[0]]*180).astype(int)%768
        ty=np.floor(points[:,:,other[1]]*180).astype(int)%768
        mask=surface==i
        rgb[mask]=_texture[ty[mask],tx[mask]]
    return rgb,best.astype('float32')


def pose_at(t):
    pose=np.eye(4)
    pose[:3,:3]=cv2.Rodrigues(np.array([.025*np.sin(t*.6),.09*np.sin(t*.45),.02*np.sin(t*.7)]))[0]
    pose[:3,3]=[.22*np.sin(t*.45),.08*np.sin(t*.7),.09*np.sin(t*.4)]
    return pose


def run_demo(tracker,stop):
    start=time.monotonic()
    while not stop.is_set():
        t=time.monotonic()-start
        rgb,depth=render(pose_at(t))
        tracker.process(rgb,depth,K,time.time())
        stop.wait(.2)
