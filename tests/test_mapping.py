import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import unittest

import numpy as np

from tracking.mapping import SceneMap


def frame(width=1, height=1, depth_value=1.0, color=(10, 20, 30)):
    rgb = np.empty((height, width, 3), dtype=np.uint8)
    rgb[:] = color
    depth = np.full((height, width), depth_value, dtype=np.float32)
    return rgb, depth


class SceneMapTests(unittest.TestCase):
    def test_projection_and_camera_to_world_transform(self):
        rgb, depth = frame(depth_value=2.0, color=(1, 2, 3))
        K = np.array([[100.0, 0.0, 0.0], [0.0, 100.0, 0.0], [0.0, 0.0, 1.0]])
        pose = np.eye(4)
        pose[:3, 3] = (1.0, -2.0, 0.5)

        scene = SceneMap(sample_stride=4)
        scene.add(rgb, depth, K, pose)

        np.testing.assert_allclose(scene.points(), [[1.0, -2.0, 2.5, 1.0, 2.0, 3.0]])

    def test_invalid_and_out_of_range_depth_is_rejected(self):
        rgb, depth = frame(width=4, height=1)
        depth[0] = (0.19, 0.2, 6.0, 6.01)
        depth[0, 0] = np.nan
        depth[0, 3] = np.inf
        scene = SceneMap(sample_stride=1)
        scene.add(rgb, depth, np.eye(3), np.eye(4))

        points = scene.points()
        self.assertEqual(len(points), 2)
        np.testing.assert_allclose(points[:, 2], (0.2, 6.0))

    def test_oldest_voxel_is_evicted_at_cap(self):
        rgb, depth = frame(width=3, height=1, depth_value=1.0)
        scene = SceneMap(voxel_size=0.01, max_points=2, sample_stride=1)
        scene.add(rgb, depth, np.eye(3), np.eye(4))

        # Keep the same sampled pixels but move the frame to a new x location.
        pose = np.eye(4)
        pose[0, 3] = 1.0
        scene.add(rgb, depth, np.eye(3), pose)

        np.testing.assert_allclose(scene.points()[:, 0], (2.0, 3.0))

    def test_voxel_deduplication_preserves_first_sample(self):
        rgb, depth = frame(width=2, height=2, depth_value=1.0, color=(40, 50, 60))
        scene = SceneMap(voxel_size=2.0, sample_stride=1)
        scene.add(rgb, depth, np.eye(3), np.eye(4))

        self.assertEqual(len(scene.points()), 1)
        np.testing.assert_array_equal(scene.points()[0, 3:], (40, 50, 60))

    def test_ply_is_ascii_and_deterministic(self):
        rgb, depth = frame(color=(7, 8, 9))
        scene = SceneMap(sample_stride=1)
        scene.add(rgb, depth, np.eye(3), np.eye(4))

        expected = (
            b"ply\nformat ascii 1.0\n"
            b"element vertex 1\n"
            b"property float x\nproperty float y\nproperty float z\n"
            b"property uchar red\nproperty uchar green\nproperty uchar blue\n"
            b"end_header\n0.000000 0.000000 1.000000 7 8 9\n"
        )
        self.assertEqual(scene.export_ply(), expected)
        self.assertEqual(scene.export_ply(), expected)


if __name__ == "__main__":
    unittest.main()
