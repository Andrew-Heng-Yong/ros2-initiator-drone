"""Compare both pose solvers on known synthetic motion or recorded RGB-D.

Run python3 scripts/evaluate.py [recording-directory]. Recorded results measure
continuity and throughput only; they have no external pose ground truth.
"""
import json
from pathlib import Path
import sys
import time
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tracking.odometry import RGBDOdometry
from tracking.demo import K,render,pose_at


def evaluate(folder=None):
    cv2.setNumThreads(2)
    if folder:
        frames=[]
        for path in sorted(Path(folder).glob('*.npz')):
            with np.load(path) as f:
                frames.append((f['rgb'],f['depth'],f['K'],float(f['timestamp']),None))
        if not frames:raise ValueError('No recorded frames')
    else:
        frames=[(*render(pose_at(i*.15)),K,i*.15,pose_at(i*.15)) for i in range(80)]
    report={}
    for method in ['svd','pnp']:
        odom=RGBDOdometry(frames[0][2],method=method)
        latency=[];errors=[];angles=[];positions=[];tracked=0
        for rgb,depth,k,stamp,truth in frames:
            start=time.perf_counter()
            out=odom.update(rgb,depth,stamp)
            latency.append((time.perf_counter()-start)*1000)
            if out['status']=='tracking':
                tracked+=1
                positions.append(out['pose'][:3,3])
            # Include held poses during tracking loss so failures cannot improve accuracy.
            if truth is not None:
                errors.append(float(np.linalg.norm(out['pose'][:3,3]-truth[:3,3])))
                angles.append(float(np.linalg.norm(cv2.Rodrigues(out['pose'][:3,:3].T@truth[:3,:3])[0])*180/np.pi))
        report[method]={'frames':len(frames),'tracked':tracked,'p50_ms':round(float(np.median(latency)),2),
                        'p95_ms':round(float(np.percentile(latency,95)),2),
                        'position_rmse_m':round(float(np.sqrt(np.mean(np.square(errors)))),4) if errors else None,
                        'rotation_rmse_deg':round(float(np.sqrt(np.mean(np.square(angles)))),3) if angles else None,
                        'max_position_from_start_m':round(float(max(np.linalg.norm(p) for p in positions)),4) if positions else None}
    return {'source':str(folder) if folder else 'synthetic textured room with known poses','results':report}


if __name__=='__main__':
    print(json.dumps(evaluate(sys.argv[1] if len(sys.argv)>1 else None),indent=2))
