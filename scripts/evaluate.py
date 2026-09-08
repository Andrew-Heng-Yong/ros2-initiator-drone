"""Compare both pose solvers on known synthetic motion or recorded RGB-D.

Run python3 scripts/evaluate.py [recording-directory]. Recorded results measure
continuity, depth consistency and throughput; they have no external pose ground truth.
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


def depth_consistency(previous, current, previous_pose, current_pose):
    """Held-out depth agreement between views, not external pose accuracy."""
    _, depth, k, _, _ = previous
    rows, cols = np.indices(depth.shape)
    keep = (rows%8==0) & (cols%8==0) & np.isfinite(depth) & (depth>.2) & (depth<6)
    z=depth[keep]
    points=np.column_stack(((cols[keep]-k[0,2])*z/k[0,0], (rows[keep]-k[1,2])*z/k[1,1], z))
    transform=np.linalg.solve(current_pose,previous_pose)
    points=points@transform[:3,:3].T+transform[:3,3]
    points=points[np.isfinite(points).all(axis=1) & (points[:,2]>.2)]
    pixels=points@current[2].T
    pixels=np.rint(pixels[:,:2]/pixels[:,2:]).astype(int)
    height,width=current[1].shape
    x,y=pixels.T
    inside=(x>=0)&(y>=0)&(x<width)&(y<height)
    measured=current[1][y[inside],x[inside]]
    valid=np.isfinite(measured)&(measured>.2)&(measured<6)
    residual=np.abs(measured[valid]-points[inside,2][valid])
    if len(residual)<100:return None
    return float(np.median(residual)),float(np.mean(residual<.05))


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
        latency=[];errors=[];angles=[];positions=[];tracked=0;depth_errors=[];previous=None
        statuses={}
        for index,(rgb,depth,k,stamp,truth) in enumerate(frames):
            if not np.allclose(k,odom.K):raise ValueError('Camera intrinsics changed within recording')
            start=time.perf_counter()
            out=odom.update(rgb,depth,stamp)
            latency.append((time.perf_counter()-start)*1000)
            statuses[out['status']]=statuses.get(out['status'],0)+1
            if out['status']=='tracking' and previous is not None and previous['status'] in ('tracking','initializing'):
                agreement=depth_consistency(frames[index-1],frames[index],previous['pose'],out['pose'])
                if agreement is not None:depth_errors.append(agreement)
            previous=out
            if out['status']=='tracking':
                tracked+=1
                positions.append(out['pose'][:3,3])
            # Include held poses during tracking loss so failures cannot improve accuracy.
            if truth is not None:
                errors.append(float(np.linalg.norm(out['pose'][:3,3]-truth[:3,3])))
                angles.append(float(np.linalg.norm(cv2.Rodrigues(out['pose'][:3,:3].T@truth[:3,:3])[0])*180/np.pi))
        report[method]={'frames':len(frames),'tracked':tracked,'status_counts':statuses,'p50_ms':round(float(np.median(latency)),2),
                        'p95_ms':round(float(np.percentile(latency,95)),2),
                        'position_rmse_m':round(float(np.sqrt(np.mean(np.square(errors)))),4) if errors else None,
                        'rotation_rmse_deg':round(float(np.sqrt(np.mean(np.square(angles)))),3) if angles else None,
                        'max_position_from_start_m':round(float(max(np.linalg.norm(p) for p in positions)),4) if positions else None,
                        'depth_consistency_pairs':len(depth_errors),
                        'median_pair_depth_error_m':round(float(np.median(np.array(depth_errors)[:,0])),4) if depth_errors else None,
                        'worst_pair_depth_error_m':round(float(max(e[0] for e in depth_errors)),4) if depth_errors else None,
                        'mean_depth_fraction_within_5cm':round(float(np.mean(np.array(depth_errors)[:,1])),3) if depth_errors else None}
    return {'source':str(folder) if folder else 'synthetic textured room with known poses','results':report}


if __name__=='__main__':
    print(json.dumps(evaluate(sys.argv[1] if len(sys.argv)>1 else None),indent=2))
