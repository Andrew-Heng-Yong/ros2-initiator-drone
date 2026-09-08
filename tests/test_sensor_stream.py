import json
import struct
import tempfile
import time
import unittest
from pathlib import Path
import numpy as np
from tracking.server import Tracker
from tracking.sensor_stream import SensorStream


class SensorContractCheck(unittest.TestCase):
    def test_capture_reset_record_and_pose_contract(self):
        tracker = Tracker(processing='phone')
        rgb = np.full((16, 24, 3), 100, dtype=np.uint8)
        depth = np.full((16, 24), 1.25, dtype=np.float32)
        depth[0, 0] = np.nan
        k = np.array([[20.,0,12],[0,20,8],[0,0,1]])
        now = time.time()
        tracker.thermal(np.full((62,80),32.5,dtype=np.float32),now)
        with tempfile.TemporaryDirectory() as folder:
            tracker.record_dir=Path(folder)
            tracker.process(rgb,depth,k,now,depth_stamp=now)
            self.assertIsNone(tracker.odom)
            data=tracker.stream.packet
            self.assertEqual(data[:4],b'DVS1')
            n=struct.unpack('<I',data[4:8])[0]
            meta=json.loads(data[8:8+n])
            self.assertEqual(meta['thermal_encoding'],'float32_celsius')
            offset=8+n+meta['jpeg_bytes']
            decoded=np.frombuffer(data[offset:offset+meta['depth_bytes']],dtype='<f4').reshape(depth.shape)
            np.testing.assert_array_equal(decoded,depth)
            with np.load(Path(folder)/'00000.npz') as frame:
                self.assertEqual(frame['thermal'].shape,(62,80))
                self.assertEqual(float(frame['thermal_timestamp']),now)
        tracker.record_dir=None
        packet_session=meta['session']
        pose=dict(session=packet_session,timestamp=now,valid=True,
                  phone_pose=np.eye(4).flatten().tolist(),rig_pose=np.eye(4).flatten().tolist())
        tracker.stream.accept_poses(pose)
        self.assertTrue(tracker.stream.pose_snapshot()['valid'])
        tracker.stream.pose_received-=3
        self.assertFalse(tracker.stream.pose_snapshot()['valid'])
        with self.assertRaises(ValueError): tracker.stream.accept_poses(pose)
        tracker.reset_requested=True
        tracker.process(rgb,depth,k,now+.1)
        self.assertNotEqual(tracker.stream.session,packet_session)
        with self.assertRaises(ValueError): tracker.stream.accept_poses(pose)
        pose.update(session=tracker.stream.session,timestamp=time.time())
        pose['rig_pose'][0]=float('nan')
        with self.assertRaises(ValueError): tracker.stream.accept_poses(pose)


if __name__=='__main__': unittest.main()
