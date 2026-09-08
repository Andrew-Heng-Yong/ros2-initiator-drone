"""DVS1: little-endian header length, JSON metadata, JPEG, float32 depth/thermal.

A single replaceable packet bounds memory independently of client speed.
Timestamps are Unix seconds; gyro rates already include scale/time correction.
"""
import json
import struct
import time
import uuid
import numpy as np


class SensorStream:
    def __init__(self, alignment=None, demo=False):
        self.alignment = alignment or {}
        self.demo = demo
        self.reset()

    def reset(self):
        self.session = str(uuid.uuid4())
        self.sequence = 0
        self.packet = None
        self.poses = None
        self.pose_received = 0.

    def publish(self, jpeg, depth, K, stamp, depth_stamp, thermal, thermal_stamp, gyro):
        self.sequence += 1
        depth = np.asarray(depth, dtype='<f4')
        thermal = np.asarray(thermal if thermal is not None else np.empty((0, 0)), dtype='<f4')
        metadata = dict(mode='demo' if self.demo else 'live', version=1, session=self.session, sequence=self.sequence,
                        timestamp=stamp, depth_timestamp=depth_stamp,
                        thermal_timestamp=thermal_stamp, width=depth.shape[1], height=depth.shape[0],
                        thermal_width=thermal.shape[1], thermal_height=thermal.shape[0],
                        K=np.asarray(K).reshape(-1).tolist(), jpeg_bytes=len(jpeg),
                        depth_bytes=depth.nbytes, thermal_bytes=thermal.nbytes,
                        depth_encoding='float32_metres', thermal_encoding='float32_celsius',
                        thermal_flipped_y=True, alignment=self.alignment, gyro=gyro)
        header = json.dumps(metadata, allow_nan=False, separators=(',', ':')).encode()
        self.packet = b'DVS1' + struct.pack('<I', len(header)) + header + jpeg + depth.tobytes() + thermal.tobytes()

    def accept_poses(self, value):
        if value.get('session') != self.session:
            raise ValueError('Sensor session changed')
        stamp = value.get('timestamp')
        if not isinstance(stamp, (int, float)) or not np.isfinite(stamp) or abs(time.time()-stamp) > 2:
            raise ValueError('Stale or invalid capture timestamp')
        if self.poses and stamp <= self.poses['timestamp']:
            raise ValueError('Out-of-order pose')
        if not isinstance(value.get('valid'), bool):
            raise ValueError('valid must be boolean')
        for name in ('phone_pose', 'rig_pose'):
            pose = np.asarray(value.get(name), dtype=float)
            if pose.shape != (16,) or not np.isfinite(pose).all():
                raise ValueError('Expected finite column-major 4x4 pose')
            pose = pose.reshape(4, 4).T
            if (not np.allclose(pose[3], [0,0,0,1], atol=1e-5)
                    or not np.allclose(pose[:3,:3].T @ pose[:3,:3], np.eye(3), atol=.01)
                    or not np.isclose(np.linalg.det(pose[:3,:3]), 1, atol=.01)):
                raise ValueError('Expected rigid transform')
        self.poses = value
        self.pose_received = time.monotonic()

    def pose_snapshot(self):
        if self.poses is None:
            return None
        return dict(self.poses, valid=self.poses['valid'] and time.monotonic()-self.pose_received < 2)
