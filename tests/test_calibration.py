"""Synthetic checks for offline gyro calibration."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import scripts.calibrate_gyro as calibration_module
from scripts.calibrate_gyro import CalibrationError, fit_calibration
from tracking.gyro import integrate_angular_velocity, so3_exp


class CalibrationTests(unittest.TestCase):
    def test_known_mount_scale_and_time_offset(self) -> None:
        mount = so3_exp([0.25, -0.18, 0.12])
        scale = 1.17
        offset = 0.012
        sample_rows = []
        for index in range(241):
            timestamp = index * 0.01
            rate = np.array([
                0.32 * np.sin(0.07 * index),
                0.28 * np.cos(0.11 * index),
                0.22 * np.sin(0.13 * index + 0.4),
            ])
            sample_rows.append([timestamp, *rate])
        samples = np.asarray(sample_rows)
        sample_pairs = [(row[0], row[1:]) for row in samples]
        pairs = []
        for index in range(20, 220, 4):
            t0, t1 = index * 0.01, (index + 4) * 0.01
            rotation = integrate_angular_velocity(
                sample_pairs,
                t0 - offset,
                t1 - offset,
                rotation_camera_from_gyro=mount,
                scale=scale,
                max_gap=0.2,
            )
            self.assertIsNotNone(rotation)
            pairs.append((t0, t1, rotation))
        result = fit_calibration(
            samples,
            pairs,
            offsets=np.arange(-0.02, 0.0201, 0.002),
            max_gap=0.2,
            min_intervals=6,
        )
        self.assertAlmostEqual(result["scale"], scale, places=3)
        self.assertAlmostEqual(result["time_offset_s"], offset, places=3)
        np.testing.assert_allclose(result["rotation_camera_from_gyro"], mount, atol=3e-3)
        self.assertLess(result["quality"]["all_rmse_deg"], 0.05)

    def test_stationary_rotation_is_rejected(self) -> None:
        samples = np.array([[index * 0.01, 0.0, 0.0, 0.0] for index in range(300)])
        pairs = [(index * 0.1, (index + 1) * 0.1, np.eye(3)) for index in range(1, 12)]
        with self.assertRaisesRegex(CalibrationError, "stationary_or_insufficient_excitation"):
            fit_calibration(samples, pairs, offsets=[0.0], max_gap=0.2)

    def test_recording_loader_deduplicates_histories_and_visual_path(self) -> None:
        import tempfile

        class FakeOdometry:
            def __init__(self, camera_matrix, method="pnp"):
                self.K = np.asarray(camera_matrix)
                assert self.K.shape == (3, 3)
                self.calls = 0

            def update(self, _rgb, _depth, _timestamp):
                self.calls += 1
                pose = np.eye(4)
                pose[:3, :3] = so3_exp([0.0, 0.0, 0.1 * self.calls])
                return {
                    "status": "initializing" if self.calls == 1 else "tracking",
                    "inliers": 20,
                    "pose": pose,
                }

        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            rgb = np.zeros((4, 4, 3), dtype=np.uint8)
            depth = np.ones((4, 4), dtype=np.float32)
            camera_matrix = np.eye(3)
            common = {
                "gyro_bias_rad_s": np.array([0.2, 0.4, 0.6]),
                "gyro_scale": 2.0,
                "gyro_time_offset_s": 0.01,
                "scale_applied": True,
                "time_offset_applied": True,
                "bias_subtracted": False,
            }
            np.savez(directory / "00000.npz", rgb=rgb, depth=depth, K=camera_matrix,
                     timestamp=1.0, gyro_samples=np.array([[0.0, 1.2, 0.4, 0.6], [0.1, 1.2, 0.4, 0.6]]), **common)
            np.savez(directory / "00001.npz", rgb=rgb, depth=depth, K=camera_matrix,
                     timestamp=1.1, gyro_samples=np.array([[0.1, 1.2, 0.4, 0.6], [0.2, 1.2, 0.4, 0.6]]), **common)
            np.savez(directory / "00002.npz", rgb=rgb, depth=depth, K=camera_matrix,
                     timestamp=1.2, gyro_samples=np.array([[0.2, 1.2, 0.4, 0.6], [0.3, 1.2, 0.4, 0.6]]), **common)
            frames, samples, metadata = calibration_module._load_recording(directory, 1, 0)
            self.assertEqual(len(frames), 3)
            self.assertEqual(samples.shape, (4, 4))
            self.assertEqual(metadata["gyro_samples_deduplicated"], 2)
            np.testing.assert_allclose(samples[:, 0], [-0.01, 0.09, 0.19, 0.29])
            np.testing.assert_allclose(samples[:, 1:], np.tile([0.5, 0.0, 0.0], (4, 1)))
            with patch.object(calibration_module, "RGBDOdometry", FakeOdometry):
                pairs, statuses, good = calibration_module._visual_pairs(frames, 12)
            self.assertEqual(good, 2)
            self.assertEqual(len(pairs), 1)
            self.assertEqual(dict(statuses), {"initializing": 1, "tracking": 2})


if __name__ == "__main__":
    unittest.main()
