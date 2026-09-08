"""Replay a recording to check gyro calibration and assisted RGB-D tracking.

Run on the Mac: python scripts/validate_gyro.py RECORDING config/gyro.json
Odd visual intervals are held out by calibrate_gyro.py's even/odd training split.
Results measure agreement with vision, not external pose ground truth.
"""
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.calibrate_gyro import _load_recording, _visual_pairs, _integrated_vector, _log_rotation
from scripts.evaluate import depth_consistency
from tracking.odometry import RGBDOdometry


def validate(folder, config):
    cv2.setNumThreads(2)
    frames, samples, metadata = _load_recording(folder, 1, 0)
    pairs, _, _ = _visual_pairs(frames, 12)
    mount = np.array(config['rotation_camera_from_gyro'])
    scale, offset = config['scale'], config['time_offset']

    def rotation(t0, t1, m=mount, s=scale, o=offset):
        vector = _integrated_vector(samples, t0, t1, m, s, o, .25)
        return None if vector is None else cv2.Rodrigues(vector)[0]

    def angle(r):
        return float(np.degrees(np.linalg.norm(_log_rotation(r))))

    def rms(values):
        return float(np.sqrt(np.mean(np.square(values)))) if len(values) else None

    heldout = []
    for t0, t1, visual in pairs[1::2]:
        gyro = rotation(t0, t1)
        if gyro is not None:
            heldout.append((_log_rotation(visual), _log_rotation(gyro), angle(gyro.T @ visual)))
    report = {'recording': str(folder), 'heldout_intervals': len(heldout),
              'heldout_rmse_deg': rms([row[2] for row in heldout]), 'axes': {}, 'timing_rmse_deg': {}}
    for axis in range(3):
        for sign in (-1, 1):
            selected = [r for r in heldout if np.argmax(np.abs(r[0])) == axis
                        and r[0][axis] * sign > np.radians(.2)]
            report['axes'][f'{"xyz"[axis]}{sign:+d}'] = {
                'intervals': len(selected), 'correct_direction': int(sum(r[1][axis] * sign > 0 for r in selected)),
                'rmse_deg': rms([r[2] for r in selected])}
    for trial_offset in (0., .02, .025, .03, .035, .04, .05):
        errors = []
        for t0, t1, visual in pairs[1::2]:
            gyro = rotation(t0, t1, o=trial_offset)
            if gyro is not None:
                errors.append(angle(gyro.T @ visual))
        report['timing_rmse_deg'][str(trial_offset)] = rms(errors)

    outputs = {}
    replay_frames = [(rgb, depth, k, t, None) for t, rgb, depth, k in frames]
    for assisted in (False, True):
        cv2.setRNGSeed(0)
        odom = RGBDOdometry(frames[0][3], method='pnp')
        results, timings, consistency = [], [], []
        priors = 0
        last_accepted = None
        for i, (t, rgb, depth, k) in enumerate(frames):
            prior = None
            if assisted and last_accepted is not None and 0 < t-last_accepted <= 1.:
                increment = rotation(last_accepted, t)
                if increment is not None:
                    prior = odom.rotation_prior_from_increment(increment)
            priors += prior is not None
            start = time.perf_counter()
            result = odom.update(rgb, depth, t, rotation_prior=prior)
            if result['status'] in ('tracking', 'initializing'):
                last_accepted = t
            timings.append((time.perf_counter() - start) * 1000)
            if results and result['status'] == 'tracking' and results[-1]['status'] in ('tracking', 'initializing'):
                agreement = depth_consistency(replay_frames[i-1], replay_frames[i], results[-1]['pose'], result['pose'])
                if agreement is not None:
                    consistency.append(agreement[0])
            results.append(result)
        label = 'assisted' if assisted else 'visual'
        outputs[label] = results
        report[label] = {'tracked': sum(r['status'] == 'tracking' for r in results),
                         'frames': len(results), 'priors': priors, 'solver_p50_ms': float(np.median(timings)),
                         'depth_pairs': len(consistency), 'median_depth_error_m': float(np.median(consistency)),
                         'worst_depth_error_m': float(max(consistency))}
    report['assisted_vs_visual'] = {
        'max_rotation_difference_deg': max(angle(a['pose'][:3, :3].T @ b['pose'][:3, :3])
                                           for a, b in zip(outputs['visual'], outputs['assisted'])),
        'max_position_difference_m': float(max(np.linalg.norm(a['pose'][:3, 3] - b['pose'][:3, 3])
                                                for a, b in zip(outputs['visual'], outputs['assisted'])))}
    # Integrate across the entire sequence to expose accumulated bias/timing errors.
    cumulative = []
    initial = outputs['visual'][0]['pose'][:3, :3]
    for i in range(1, len(frames)):
        if i % 10 and i != len(frames)-1:
            continue
        gyro = rotation(frames[0][0], frames[i][0])
        if gyro is not None:
            cumulative.append(angle(gyro.T @ initial.T @ outputs['visual'][i]['pose'][:3, :3]))
    report['cumulative_gyro_vs_visual'] = {'max_deg': max(cumulative), 'final_deg': cumulative[-1]}
    return report


if __name__ == '__main__':
    print(json.dumps(validate(Path(sys.argv[1]), json.loads(Path(sys.argv[2]).read_text())), indent=2))
