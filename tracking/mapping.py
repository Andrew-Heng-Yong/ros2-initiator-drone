"""Small bounded RGB-D voxel map."""

from collections import OrderedDict

import numpy as np


class SceneMap:
    """Accumulate RGB-D samples in a bounded, insertion-ordered voxel map."""

    def __init__(
        self,
        voxel_size=0.04,
        max_points=60000,
        min_depth=0.2,
        max_depth=6.0,
        sample_stride=4,
    ):
        self.voxel_size = float(voxel_size)
        self.max_points = int(max_points)
        self.min_depth = float(min_depth)
        self.max_depth = float(max_depth)
        self.sample_stride = int(sample_stride)
        if not np.isfinite(self.voxel_size) or self.voxel_size <= 0:
            raise ValueError("voxel_size must be finite and positive")
        if self.max_points <= 0:
            raise ValueError("max_points must be positive")
        if (
            not np.isfinite(self.min_depth)
            or not np.isfinite(self.max_depth)
            or self.min_depth < 0
            or self.min_depth > self.max_depth
        ):
            raise ValueError("depth limits must be finite and ordered")
        if self.sample_stride <= 0:
            raise ValueError("sample_stride must be positive")
        self._voxels = OrderedDict()

    def add(self, rgb, depth, K, pose):
        """Integrate one RGB-D frame using a camera-to-world pose."""
        rgb = np.asarray(rgb)
        depth = np.asarray(depth)
        K = np.asarray(K, dtype=np.float64)
        pose = np.asarray(pose, dtype=np.float64)

        if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
            raise ValueError("rgb must have shape (H, W, 3) and dtype uint8")
        if depth.ndim != 2 or depth.shape != rgb.shape[:2]:
            raise ValueError("depth must have shape (H, W) matching rgb")
        if K.shape != (3, 3) or not np.isfinite(K).all():
            raise ValueError("K must be a finite 3x3 matrix")
        if pose.shape != (4, 4) or not np.isfinite(pose).all():
            raise ValueError("pose must be a finite 4x4 matrix")

        rows = np.arange(0, depth.shape[0], self.sample_stride)
        cols = np.arange(0, depth.shape[1], self.sample_stride)
        sampled_depth = depth[np.ix_(rows, cols)].astype(np.float64, copy=False)
        valid = np.isfinite(sampled_depth)
        valid &= sampled_depth >= self.min_depth
        valid &= sampled_depth <= self.max_depth
        if not valid.any():
            return

        row_grid, col_grid = np.meshgrid(rows, cols, indexing="ij")
        sampled_depth = sampled_depth[valid]
        pixels = np.stack(
            (
                col_grid[valid].astype(np.float64),
                row_grid[valid].astype(np.float64),
                np.ones(sampled_depth.size),
            )
        )
        try:
            rays = np.linalg.solve(K, pixels).T
        except np.linalg.LinAlgError as exc:
            raise ValueError("K must be invertible") from exc
        camera_points = rays * sampled_depth[:, None]
        world_points = camera_points @ pose[:3, :3].T + pose[:3, 3]
        colors = rgb[np.ix_(rows, cols)][valid]
        points = np.column_stack((world_points, colors))

        for point in points:
            key = tuple(np.floor(point[:3] / self.voxel_size).astype(np.int64))
            if key in self._voxels:
                continue
            if len(self._voxels) >= self.max_points:
                self._voxels.popitem(last=False)
            self._voxels[key] = point.astype(np.float64, copy=False)

    def __len__(self):
        return len(self._voxels)

    def points(self):
        """Return points as an ``N x 6`` float array in insertion order."""
        if not self._voxels:
            return np.empty((0, 6), dtype=np.float64)
        return np.array(tuple(self._voxels.values()), dtype=np.float64, copy=True)

    def clear(self):
        self._voxels.clear()

    def export_ply(self):
        """Return the map as deterministic ASCII PLY bytes."""
        points = self.points()
        header = (
            "ply\n"
            "format ascii 1.0\n"
            f"element vertex {len(points)}\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "property uchar red\n"
            "property uchar green\n"
            "property uchar blue\n"
            "end_header\n"
        )
        body = "".join(
            f"{x:.6f} {y:.6f} {z:.6f} {int(r)} {int(g)} {int(b)}\n"
            for x, y, z, r, g, b in points
        )
        return (header + body).encode("ascii")
