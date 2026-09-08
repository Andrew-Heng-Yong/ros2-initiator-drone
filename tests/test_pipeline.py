"""Checks the estimator-to-portal contract without ROS or hardware."""
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tracking.server import Tracker
from tracking.demo import K, render
from tracking.gyro import Gyro
from scripts.evaluate import depth_consistency


def main():
    plane=np.full((200,200),2.,dtype=np.float32)
    intrinsics=np.array([[100.,0,100],[0,100,100],[0,0,1]])
    frame=(None,plane,intrinsics,0.,None)
    assert depth_consistency(frame,frame,np.eye(4),np.eye(4))==(0.,1.)
    displaced=np.eye(4);displaced[2,3]=.3
    assert depth_consistency(frame,frame,np.eye(4),displaced)[0]>.29
    rgb,depth=render(np.eye(4))
    tracker=Tracker()
    tracker.process(rgb,depth,np.zeros((3,3)),1.)
    assert tracker.snapshot()['status']=='uncalibrated'
    assert tracker.snapshot()['points']==0
    tracker.process(rgb,depth,K,2.)
    tracker.process(rgb,depth,K,2.2)
    state=tracker.snapshot()
    assert state['status']=='tracking',state
    assert state['points']>1000
    pose=tracker.pose.copy();cloud=tracker.mapper.points().copy()
    tracker.process(np.zeros_like(rgb),depth,K,2.4)
    assert tracker.snapshot()['status']=='lost'
    assert tracker.last_stamp==2.2
    assert np.array_equal(tracker.pose,pose)
    assert np.array_equal(tracker.mapper.points(),cloud)
    tracker.process(rgb,depth,K,2.6)
    assert tracker.snapshot()['status']=='tracking'
    tracker.last_frame-=3
    assert tracker.snapshot()['status']=='stale'
    tracker.reset_requested=True
    tracker.process(rgb,depth,K,2.8)
    assert tracker.snapshot()['points']==0
    assert np.allclose(tracker.pose,np.eye(4))
    assert len(tracker.path)==0
    gyro=Gyro()
    gyro.inject_sample(3.,[.1,.2,.3])
    tracker.gyro=gyro
    with TemporaryDirectory() as folder:
        tracker.record_dir=Path(folder)
        tracker.process(rgb,depth,K,3.,.02,3.02)
        assert not list(Path(folder).glob('*.part'))
        with np.load(Path(folder)/'00000.npz',allow_pickle=False) as recording:
            assert recording['gyro_samples'].shape==(1,4)
            assert float(recording['depth_timestamp'])==3.02
            assert float(recording['sync_error'])==.02
            assert not bool(recording['gyro_bias_subtracted'])
            assert np.isnan(recording['gyro_bias_rad_s']).all()
    tracker.record_dir=None
    with patch.object(gyro, 'relative_rotation', return_value=np.eye(3)) as rotation:
        tracker.process(rgb,depth,K,3.2)
        rotation.assert_called_once_with(3.,3.2)
        assert tracker.snapshot()['metrics']['gyro_prior']
        rotation.reset_mock()
        tracker.process(rgb,depth,K,4.3)
        rotation.assert_not_called()
        assert not tracker.snapshot()['metrics']['gyro_prior']
    print('PASS: calibration, live pipeline, map, lost freeze, recovery, stale and reset')


if __name__=='__main__':main()
