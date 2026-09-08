import json
import struct
import tempfile
import time
import unittest
import zlib
from pathlib import Path
import numpy as np
from tracking.server import Tracker
from tracking.sensor_stream import SensorStream


class SensorContractCheck(unittest.TestCase):
    def test_thermal_colour_does_not_change_when_hot_object_leaves(self):
        tracker=Tracker(processing='phone')
        images=[]
        tracker.preview=lambda name,image: images.append(image.copy())
        tracker.thermal(np.array([[20.,27.],[20.,20.]],dtype=np.float32))
        tracker.thermal(np.full((2,2),20.,dtype=np.float32))
        np.testing.assert_array_equal(images[0][0,0],images[1][0,0])
        self.assertEqual(tracker.metrics['thermal_range'],[19.,28.])
        with self.assertRaises(ValueError): Tracker(thermal_range=(28,19))

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
            decoded=np.frombuffer(zlib.decompress(data[offset:offset+meta['depth_bytes']],wbits=-15),dtype='<f4').reshape(depth.shape)
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
