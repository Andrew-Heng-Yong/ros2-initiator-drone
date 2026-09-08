"""Deterministic checks for the ROS-free RGB-D odometry core."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tracking.odometry import RGBDOdometry, rigid_transform_ransac


def _synthetic_rgbd():
    height, width = 240, 320
    fx = fy = 220.0
    cx, cy = width / 2.0, height / 2.0
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])

    rng = np.random.default_rng(42)
    gray = np.zeros((height, width), dtype=np.uint8)
    # Textured, repeatable marks give ORB stable descriptors while leaving a
    # large enough interior margin for the translated view.
    for _ in range(180):
        x = int(rng.integers(25, width - 25))
        y = int(rng.integers(25, height - 25))
        radius = int(rng.integers(2, 7))
        cv2.circle(gray, (x, y), radius, int(rng.integers(40, 255)), -1)
        if rng.random() < 0.45:
            cv2.line(gray, (x - radius, y), (x + radius, y), 255, 1)
    gray = cv2.GaussianBlur(gray, (3, 3), 0.4)
    first = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)

    translation_m = np.array([0.24, -0.06, 0.0])
    shift = np.array([-fx * translation_m[0] / 4.0, -fy * translation_m[1] / 4.0])
    affine = np.array([[1.0, 0.0, shift[0]], [0.0, 1.0, shift[1]]], dtype=np.float32)
    second_gray = cv2.warpAffine(
        gray,
        affine,
        (width, height),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )
    second = cv2.cvtColor(second_gray, cv2.COLOR_GRAY2RGB)
    first_depth = np.full((height, width), 4.0, dtype=np.float32)
    second_depth = np.full((height, width), 4.0, dtype=np.float32)
    # The synthetic plane has no measured depth in the warped border.
    border = 18
    second_depth[:border] = 0.0
    second_depth[-border:] = 0.0
    second_depth[:, :border] = 0.0
    second_depth[:, -border:] = 0.0
    return K, first, first_depth, second, second_depth, translation_m


class RGBDOdometryTests(unittest.TestCase):
    def test_depth_patch_median_matches_numpy_with_holes_and_edges(self):
        odom = RGBDOdometry(np.eye(3))
        depth = np.random.default_rng(5).uniform(0, 8, (8, 9)).astype(np.float32)
        depth[::2, ::2] = np.nan
        depth[0, :3] = [0, np.inf, -1]
        for y in range(8):
            for x in range(9):
                patch = depth[max(0,y-1):y+2, max(0,x-1):x+2]
                valid = patch[np.isfinite(patch) & (patch >= .2) & (patch <= 6)]
                actual = odom._depth_at(depth, (x,y))
                if valid.size:
                    self.assertAlmostEqual(actual, float(np.median(valid)), places=6)
                else:
                    self.assertIsNone(actual)
        self.assertIsNone(odom._depth_at(depth, (-1,0)))

    def test_rigid_ransac_rejects_outliers_and_preserves_metric_direction(self):
        rng = np.random.default_rng(9)
        source = rng.uniform([-1.0, -0.8, 2.5], [1.0, 0.8, 5.0], size=(60, 3))
        angle = 0.18
        rotation_true = np.array(
            [
                [np.cos(angle), -np.sin(angle), 0.0],
                [np.sin(angle), np.cos(angle), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        translation_true = np.array([0.31, -0.12, 0.18])
        target = source @ rotation_true.T + translation_true
        target[-18:] = rng.uniform(-4.0, 4.0, size=(18, 3))

        rotation, translation, mask, rmse = rigid_transform_ransac(
            source, target, threshold_m=0.01, iterations=800, seed=12
        )

        self.assertGreaterEqual(int(mask.sum()), 42)
        np.testing.assert_allclose(rotation, rotation_true, atol=2.0e-3)
        np.testing.assert_allclose(translation, translation_true, atol=2.0e-3)
        self.assertLess(rmse, 1.0e-6)
        # source -> target is the same direction as the documented pose fit.
        self.assertGreater(float(translation[0]), 0.0)

    def test_synthetic_rgbd_motion_loss_and_recovery(self):
        K, first, first_depth, second, second_depth, expected_translation = _synthetic_rgbd()
        odometry = RGBDOdometry(
            K,
            ransac_threshold_m=0.04,
            max_rmse_m=0.06,
            min_inliers=6,
            seed=3,
        )

        initial = odometry.update(first, first_depth, 0.0)
        self.assertEqual(initial["status"], "initializing")
        self.assertTrue(np.allclose(initial["pose"], np.eye(4)))

        tracked = odometry.update(second, second_depth, 1.0)
        self.assertEqual(tracked["status"], "tracking")
        self.assertEqual(tracked["solver"], "svd")
        self.assertGreaterEqual(tracked["inliers"], 6)
        self.assertGreaterEqual(tracked["matches"], tracked["inliers"])
        np.testing.assert_allclose(
            tracked["pose"][:3, 3], expected_translation, atol=0.035
        )
        self.assertLess(tracked["rmse"], 0.04)
        self.assertEqual(odometry.reference_timestamp, 1.0)

        blank = np.zeros_like(first)
        blank_depth = np.zeros_like(first_depth)
        lost = odometry.update(blank, blank_depth, 2.0)
        self.assertEqual(lost["status"], "lost")
        np.testing.assert_allclose(lost["pose"], tracked["pose"])

        recovered = odometry.update(second, second_depth, 3.0)
        self.assertEqual(recovered["status"], "tracking")
        self.assertGreaterEqual(recovered["inliers"], 6)
        np.testing.assert_allclose(recovered["pose"], tracked["pose"], atol=0.035)

    def test_frame_and_timestamp_validation(self):
        K = np.diag([100.0, 100.0, 1.0])
        rgb = np.zeros((20, 24, 3), dtype=np.uint8)
        depth = np.ones((20, 24), dtype=np.float32)
        odometry = RGBDOdometry(K)
        odometry.update(rgb, depth, 1.0)
        with self.assertRaises(ValueError):
            odometry.update(rgb, depth, 1.0)
        with self.assertRaises(ValueError):
            odometry.update(rgb.astype(np.float32), depth, 2.0)

    def test_pnp_alternate_can_use_reference_depth_when_current_depth_has_holes(self):
        K, first, first_depth, second, second_depth, _ = _synthetic_rgbd()
        odometry = RGBDOdometry(K, method="pnp", min_inliers=6, seed=3)
        odometry.update(first, first_depth, 0.0)
        result = odometry.update(
            second,
            np.zeros_like(second_depth),
            1.0,
            rotation_prior=np.eye(3),
        )
        self.assertEqual(result["status"], "tracking")
        self.assertEqual(result["solver"], "pnp")
        self.assertGreaterEqual(result["inliers"], 6)

    def test_svd_is_strict_and_auto_owns_pnp_fallback(self):
        K, first, first_depth, second, second_depth, _ = _synthetic_rgbd()
        strict = RGBDOdometry(K, method="svd", min_inliers=6)
        strict.update(first, first_depth, 0.0)
        result = strict.update(second, np.zeros_like(second_depth), 1.0)
        self.assertEqual(result["status"], "lost")
        self.assertIsNone(result["solver"])

        automatic = RGBDOdometry(K, method="auto", min_inliers=6)
        automatic.update(first, first_depth, 0.0)
        result = automatic.update(second, np.zeros_like(second_depth), 1.0)
        self.assertEqual(result["status"], "tracking")
        self.assertEqual(result["solver"], "pnp")


if __name__ == "__main__":
    unittest.main()
