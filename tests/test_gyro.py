"""Deterministic checks for the gyro math and calibration gate."""

from __future__ import annotations

import json
import struct
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import unittest
from unittest.mock import patch

import numpy as np

import tracking.gyro as gyro_module
from tracking.gyro import Gyro, integrate_angular_velocity, so3_exp, stationary_bias


class GyroMathTests(unittest.TestCase):
    def test_known_z_axis_rotation(self) -> None:
        angle = np.pi / 2.0
        result = integrate_angular_velocity(
            [(0.0, [0.0, 0.0, angle]), (1.0, [0.0, 0.0, angle])],
            0.0,
            1.0,
            max_gap=2.0,
        )
        np.testing.assert_allclose(result, so3_exp([0.0, 0.0, angle]), atol=1e-12)

    def test_compound_rotation_preserves_so3_order(self) -> None:
        x = integrate_angular_velocity(
            [(0.0, [np.pi / 2.0, 0.0, 0.0]), (1.0, [np.pi / 2.0, 0.0, 0.0])],
            0.0,
            1.0,
            max_gap=2.0,
        )
        y = integrate_angular_velocity(
            [(1.0, [0.0, np.pi / 2.0, 0.0]), (2.0, [0.0, np.pi / 2.0, 0.0])],
            1.0,
            2.0,
            max_gap=2.0,
        )
        np.testing.assert_allclose(x @ y, so3_exp([np.pi / 2.0, 0.0, 0.0]) @ so3_exp([0.0, np.pi / 2.0, 0.0]), atol=1e-12)

    def test_bounds_and_large_timestamp_gap_return_none(self) -> None:
        samples = [(10.0, [0.0, 0.0, 1.0]), (10.1, [0.0, 0.0, 1.0]), (11.0, [0.0, 0.0, 1.0])]
        self.assertIsNone(integrate_angular_velocity(samples, 9.9, 10.0))
        self.assertIsNone(integrate_angular_velocity(samples, 10.1, 11.0, max_gap=0.2))
        self.assertIsNone(integrate_angular_velocity([(10.0, [0.0, 0.0, 1.0]), (11.0, [0.0, 0.0, 1.0])], 10.4, 10.5, max_gap=0.2))

    def test_stationary_bias_and_movement_rejection(self) -> None:
        bias = np.array([0.02, -0.01, 0.03])
        stationary = [
            (index * 0.01, bias + (0.01 if index % 2 else -0.01) * np.array([1.0, -1.0, 0.5]))
            for index in range(301)
        ]
        estimate, reason = stationary_bias(stationary)
        self.assertEqual(reason, "calibrated")
        np.testing.assert_allclose(estimate, bias, atol=4e-5)

        moving = [(index * 0.01, [0.0, 0.25 if index % 2 else -0.25, 0.0]) for index in range(301)]
        estimate, reason = stationary_bias(moving)
        self.assertIsNone(estimate)
        self.assertEqual(reason, "motion")

        irregular_times = np.cumsum([0.0] + [0.009 + 0.0002 * (index % 3) for index in range(340)])
        estimate, reason = stationary_bias([(timestamp, bias) for timestamp in irregular_times])
        self.assertEqual(reason, "calibrated")
        np.testing.assert_allclose(estimate, bias, atol=1e-12)

    def test_fusion_requires_calibration_enable_and_mount_validation(self) -> None:
        gyro = Gyro(
            device="/definitely/missing",
            enabled=False,
            calibration_duration=0.1,
            mounting_validated=False,
        )
        samples = [(0.0, [0.01, 0.0, 0.0])]
        samples.extend(
            (samples[-1][0] + 0.009 + 0.0002 * (index % 3), [0.01, 0.0, 0.0])
            for index in range(13)
        )
        for timestamp, value in samples:
            gyro.inject_sample(timestamp, value)
        self.assertTrue(gyro.status()["calibrated"])
        self.assertIsNone(gyro.relative_rotation(0.1, 0.1))
        gyro.enabled = True
        self.assertIsNone(gyro.relative_rotation(0.1, 0.1))
        gyro.set_mounting_validated(True)
        self.assertIsNone(gyro.relative_rotation(0.1, 0.1))
        self.assertFalse(gyro.status()["fusion_ready"])
        json.dumps(gyro.status())
        gyro.close()

    def test_recording_snapshot_calibrated_and_unavailable_shapes(self) -> None:
        calibrated = Gyro(
            device="/definitely/missing",
            range_dps=500,
            scale=1.25,
            time_offset=0.02,
            calibration_duration=0.1,
        )
        values = np.array([0.02, -0.01, 0.03])
        for index in range(13):
            calibrated.inject_sample(index * 0.01, values)
        snapshot = calibrated.recording_snapshot()
        self.assertEqual(snapshot["samples"].shape, (13, 4))
        self.assertEqual(snapshot["bias_rad_s"].shape, (3,))
        self.assertTrue(snapshot["calibrated"])
        np.testing.assert_allclose(snapshot["samples"][:, 1:], np.tile(values, (13, 1)))
        np.testing.assert_allclose(snapshot["bias_rad_s"], values)
        self.assertEqual(snapshot["sample_columns"].tolist(), [
            "timestamp_s", "gx_rad_s", "gy_rad_s", "gz_rad_s"
        ])
        self.assertTrue(snapshot["scale_applied"])
        self.assertTrue(snapshot["time_offset_applied"])
        self.assertFalse(snapshot["bias_subtracted"])
        self.assertEqual(snapshot["range_dps"], 500)
        calibrated.close()

        unavailable = Gyro(device="/definitely/missing")
        unavailable_snapshot = unavailable.recording_snapshot()
        self.assertEqual(unavailable_snapshot["samples"].shape, (0, 4))
        self.assertEqual(unavailable_snapshot["bias_rad_s"].shape, (3,))
        self.assertTrue(np.isnan(unavailable_snapshot["bias_rad_s"]).all())
        self.assertFalse(unavailable_snapshot["calibrated"])
        unavailable.close()

    def test_hardware_path_reads_gyro_block_only(self) -> None:
        selected = {"register": None}
        writes: list[bytes] = []
        responses = {
            gyro_module.REG_WHO_AM_I: b"\x68",
            gyro_module.REG_CONFIG: b"\x03",
            gyro_module.REG_GYRO_CONFIG: b"\x08",
            gyro_module.REG_GYRO_OUT: struct.pack(">hhh", 0, 0, 655),
        }

        def fake_write(_fd: int, payload: bytes) -> int:
            writes.append(payload)
            if len(payload) == 1:
                selected["register"] = payload[0]
            return len(payload)

        def fake_read(_fd: int, length: int) -> bytes:
            payload = responses[selected["register"]]
            self.assertEqual(len(payload), length)
            return payload

        gyro = Gyro(device="/dev/fake", range_dps=500)
        with patch.object(gyro_module.os, "write", side_effect=fake_write), patch.object(
            gyro_module.os, "read", side_effect=fake_read
        ):
            who_am_i, actual_range = gyro._configure_fd(3)
            gyro._fd = 3
            sample = gyro._read_sample()
        self.assertEqual((who_am_i, actual_range), (0x68, 500))
        self.assertAlmostEqual(sample.angular_velocity[2], np.deg2rad(10.0), places=12)
        self.assertTrue({payload[0] for payload in writes}.issubset({0x19, 0x1A, 0x1B, 0x43, 0x6B, 0x75}))
        gyro.close()


if __name__ == "__main__":
    unittest.main()
