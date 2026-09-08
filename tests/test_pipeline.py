"""Checks the estimator-to-portal contract without ROS or hardware."""
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tracking.server import Tracker
from tracking.demo import K, render


def main():
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
    print('PASS: calibration, live pipeline, map, lost freeze, recovery, stale and reset')


if __name__=='__main__':main()
