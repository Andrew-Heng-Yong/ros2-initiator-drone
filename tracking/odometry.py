"""Small RGB-D camera odometry core.

The estimator keeps a recent RGB-D frame as an active reference keyframe.  For
each later frame, mutually consistent ORB matches are back-projected with the
registered depth images and a metric rigid transform is estimated with RANSAC
and the Kabsch/SVD fit.  The original keyframe remains as a recovery anchor;
the active keyframe is refreshed after enough motion to keep long walks local.

Pose convention
---------------
``pose`` is the camera-to-initial-world transform ``T_WC``.  The initial world
frame is the first keyframe camera frame, so a point measured in the current
camera frame is transformed as::

    p_initial = pose[:3, :3] @ p_current + pose[:3, 3]

Consequently, a camera that translates +X in the initial world has a +X pose
translation.  The transform is kept at its last good value while tracking is
lost; the reference keyframe is retained so a later frame can recover.

If an active keyframe is refreshed, the estimated current-camera-to-keyframe
transform is composed with that keyframe's stored ``T_WC``.  A gyro prior must
use this same current-camera-to-active-keyframe direction and interval.

This module intentionally has no ROS dependency.  ``cv2`` is used only for
ORB, descriptor matching, and the optional PnP estimator.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional, Sequence, Tuple

import cv2
import numpy as np


_EPS = 1.0e-9


def _fit_rigid_transform(source: np.ndarray, target: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Fit ``target ~= R @ source + t`` with a reflection-safe SVD fit."""

    source = np.asarray(source, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("source and target must both have shape (N, 3)")
    if source.shape[0] < 3 or not np.isfinite(source).all() or not np.isfinite(target).all():
        raise ValueError("at least three finite 3-D points are required")

    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    source_zero = source - source_center
    target_zero = target - target_center
    source_singular = np.linalg.svd(source_zero, compute_uv=False)
    target_singular = np.linalg.svd(target_zero, compute_uv=False)
    if (
        source_singular[0] <= _EPS
        or target_singular[0] <= _EPS
        or source_singular[1] <= max(source_singular[0] * 1.0e-7, _EPS)
        or target_singular[1] <= max(target_singular[0] * 1.0e-7, _EPS)
    ):
        raise ValueError("degenerate 3-D point geometry")

    covariance = source_zero.T @ target_zero
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
        raise ValueError("non-finite rigid transform")
    return rotation, translation


def rigid_transform_ransac(
    source: np.ndarray,
    target: np.ndarray,
    *,
    threshold_m: float = 0.05,
    iterations: int = 500,
    seed: int = 7,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """Estimate ``target ~= R @ source + t`` and return ``R, t, mask, rmse``.

    The function is public so a caller can validate the metric fitting stage
    independently of image feature extraction.  A minimum of three
    non-collinear points is required; coplanar points are valid.
    """

    source = np.asarray(source, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("source and target must both have shape (N, 3)")
    if source.shape[0] < 3:
        raise ValueError("at least three correspondences are required")
    if not np.isfinite(source).all() or not np.isfinite(target).all():
        raise ValueError("correspondences must be finite")
    if not np.isfinite(threshold_m) or threshold_m <= 0.0:
        raise ValueError("threshold_m must be finite and positive")
    if int(iterations) < 1:
        raise ValueError("iterations must be positive")

    count = source.shape[0]
    generator = np.random.default_rng(int(seed))
    best: Optional[Tuple[int, float, np.ndarray, np.ndarray, np.ndarray]] = None
    max_iterations = int(iterations)
    confidence = 0.995
    required_iterations = max_iterations
    iteration = 0

    # A quality-ordered prefix improves deterministic recovery when descriptor
    # matches are already mostly correct; random samples handle outliers.  The
    # confidence bound stops early once the observed consensus is strong.
    while iteration < min(max_iterations, max(1, count * 4)) and iteration < required_iterations:
        if iteration < count - 2:
            sample = np.array([iteration, iteration + 1, iteration + 2], dtype=np.int64)
        else:
            sample = generator.choice(count, size=3, replace=False)
        iteration += 1
        try:
            rotation, translation = _fit_rigid_transform(source[sample], target[sample])
        except ValueError:
            continue
        residual = np.linalg.norm((source @ rotation.T) + translation - target, axis=1)
        inlier_mask = residual <= threshold_m
        inlier_count = int(inlier_mask.sum())
        if inlier_count < 3:
            continue
        sample_rmse = float(np.sqrt(np.mean(np.square(residual[inlier_mask]))))
        candidate = (
            inlier_count,
            sample_rmse,
            rotation,
            translation,
            inlier_mask,
        )
        if best is None or candidate[0] > best[0] or (
            candidate[0] == best[0] and candidate[1] < best[1]
        ):
            best = candidate
            inlier_ratio = inlier_count / float(count)
            if inlier_ratio >= 1.0:
                required_iterations = iteration
            elif inlier_ratio > 0.0:
                denominator = math.log(max(_EPS, 1.0 - inlier_ratio**3))
                if denominator < 0.0:
                    required_iterations = min(
                        required_iterations,
                        max(iteration, int(math.ceil(math.log(1.0 - confidence) / denominator))),
                    )

    if best is None:
        raise ValueError("RANSAC found no non-degenerate transform")

    _, _, rotation, translation, inlier_mask = best
    # Refine only the winning consensus set; no SVD is spent on losing
    # hypotheses.
    rotation, translation = _fit_rigid_transform(source[inlier_mask], target[inlier_mask])
    residual = np.linalg.norm((source @ rotation.T) + translation - target, axis=1)
    inlier_mask = residual <= threshold_m
    if int(inlier_mask.sum()) < 3:
        raise ValueError("RANSAC consensus became degenerate")
    rotation, translation = _fit_rigid_transform(source[inlier_mask], target[inlier_mask])
    residual = np.linalg.norm((source @ rotation.T) + translation - target, axis=1)
    inlier_mask = residual <= threshold_m
    if int(inlier_mask.sum()) < 3:
        raise ValueError("RANSAC consensus became degenerate")
    rmse = float(np.sqrt(np.mean(np.square(residual[inlier_mask]))))
    return rotation, translation, inlier_mask, rmse


def _rotation_from_prior(rotation_prior: Any) -> np.ndarray:
    """Convert a 3x3/4x4 matrix, Rodrigues vector, or xyzw quaternion."""

    prior = np.asarray(rotation_prior, dtype=np.float64)
    if not np.isfinite(prior).all():
        raise ValueError("rotation_prior must be finite")
    # OpenCV commonly represents Rodrigues vectors and quaternions as column
    # vectors.  Accept those equivalent shapes as well as their flat forms.
    if prior.shape in {(3, 1), (1, 3), (4, 1), (1, 4)}:
        prior = prior.reshape(-1)
    if prior.shape == (4, 4):
        prior = prior[:3, :3]
    elif prior.shape == (3,):
        prior, _ = cv2.Rodrigues(prior.reshape(3, 1))
    elif prior.shape == (4,):
        x, y, z, w = prior
        norm = float(np.linalg.norm(prior))
        if norm <= _EPS:
            raise ValueError("rotation_prior quaternion has zero norm")
        x, y, z, w = prior / norm
        prior = np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )
    if prior.shape != (3, 3):
        raise ValueError("rotation_prior must be 3x3, 4x4, Rodrigues xyz, or xyzw")
    # Project tiny numerical drift back to SO(3); reject an actual reflection.
    u, _, vt = np.linalg.svd(prior)
    projected = u @ vt
    if np.linalg.det(projected) < 0.0:
        raise ValueError("rotation_prior must describe a proper rotation")
    if np.linalg.norm(prior - projected, ord="fro") > 0.10:
        raise ValueError("rotation_prior is not a rotation matrix")
    return projected


def _rotation_difference_rad(first: np.ndarray, second: np.ndarray) -> float:
    relative = first.T @ second
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    return float(math.acos(cosine))


class RGBDOdometry:
    """ORB RGB-D odometry with metric camera-to-initial-world poses.

    ``method="svd"`` is strict 3-D-to-3-D tracking, ``method="pnp"`` uses
    ``cv2.solvePnPRansac`` followed by reprojection refinement, and
    ``method="auto"`` tries SVD before falling back to PnP.  The returned
    dictionary reports the accepted solver as ``"svd"`` or ``"pnp"``;
    lost and initializing frames report ``None``.  ``rmse`` is in metres for
    a 3-D fit and pixels for a PnP-only fit where current depth is unavailable.
    """

    def __init__(
        self,
        K: Sequence[Sequence[float]],
        method: str = "svd",
        *,
        max_features: int = 800,
        ratio_test: float = 0.80,
        ransac_threshold_m: float = 0.05,
        ransac_iterations: int = 500,
        min_inliers: int = 12,
        min_inlier_ratio: float = 0.30,
        max_rmse_m: float = 0.08,
        min_depth_m: float = 0.2,
        max_depth_m: float = 6.0,
        min_geometry_spread_m: float = 0.02,
        keyframe_translation_m: float = 0.20,
        keyframe_rotation_rad: float = math.radians(15.0),
        max_translation_speed_m_s: float = 5.0,
        max_rotation_speed_rad_s: float = 3.0,
        rotation_prior_gate_rad: float = math.radians(60.0),
        seed: int = 7,
    ) -> None:
        camera_matrix = np.asarray(K, dtype=np.float64)
        if camera_matrix.shape != (3, 3) or not np.isfinite(camera_matrix).all():
            raise ValueError("K must be a finite 3x3 camera matrix")
        if camera_matrix[0, 0] <= 0.0 or camera_matrix[1, 1] <= 0.0:
            raise ValueError("K focal lengths must be positive")
        if abs(float(np.linalg.det(camera_matrix))) <= _EPS:
            raise ValueError("K must be invertible")
        selected_method = str(method).lower()
        if selected_method not in {"svd", "pnp", "auto"}:
            raise ValueError("method must be 'svd', 'pnp', or 'auto'")
        if int(max_features) < 32:
            raise ValueError("max_features must be at least 32")
        if not 0.0 < float(ratio_test) < 1.0:
            raise ValueError("ratio_test must be between 0 and 1")
        if int(min_inliers) < 3:
            raise ValueError("min_inliers must be at least 3")
        if int(ransac_iterations) < 1:
            raise ValueError("ransac_iterations must be positive")
        if not 0.0 < float(min_inlier_ratio) <= 1.0:
            raise ValueError("min_inlier_ratio must be in (0, 1]")
        for value, name in (
            (ransac_threshold_m, "ransac_threshold_m"),
            (max_rmse_m, "max_rmse_m"),
            (min_depth_m, "min_depth_m"),
            (max_depth_m, "max_depth_m"),
            (min_geometry_spread_m, "min_geometry_spread_m"),
            (keyframe_translation_m, "keyframe_translation_m"),
            (keyframe_rotation_rad, "keyframe_rotation_rad"),
            (max_translation_speed_m_s, "max_translation_speed_m_s"),
            (max_rotation_speed_rad_s, "max_rotation_speed_rad_s"),
            (rotation_prior_gate_rad, "rotation_prior_gate_rad"),
        ):
            if not np.isfinite(value) or float(value) <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if min_depth_m >= max_depth_m:
            raise ValueError("min_depth_m must be less than max_depth_m")

        self.K = camera_matrix.copy()
        self.method = selected_method
        self.max_features = int(max_features)
        self.ratio_test = float(ratio_test)
        self.ransac_threshold_m = float(ransac_threshold_m)
        self.ransac_iterations = int(ransac_iterations)
        self.min_inliers = int(min_inliers)
        self.min_inlier_ratio = float(min_inlier_ratio)
        self.max_rmse_m = float(max_rmse_m)
        self.min_depth_m = float(min_depth_m)
        self.max_depth_m = float(max_depth_m)
        self.min_geometry_spread_m = float(min_geometry_spread_m)
        self.keyframe_translation_m = float(keyframe_translation_m)
        self.keyframe_rotation_rad = float(keyframe_rotation_rad)
        self.max_translation_speed_m_s = float(max_translation_speed_m_s)
        self.max_rotation_speed_rad_s = float(max_rotation_speed_rad_s)
        self.rotation_prior_gate_rad = float(rotation_prior_gate_rad)
        self.seed = int(seed)
        self._orb = cv2.ORB_create(
            nfeatures=self.max_features,
            scaleFactor=1.2,
            nlevels=8,
            edgeThreshold=15,
            patchSize=31,
            fastThreshold=10,
        )
        self._matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self.reset()

    def reset(self) -> None:
        """Discard the keyframe and return to the identity pose."""

        self._reference_rgb: Optional[np.ndarray] = None
        self._reference_depth: Optional[np.ndarray] = None
        self._reference_keypoints = []
        self._reference_descriptors: Optional[np.ndarray] = None
        self._reference_timestamp: Optional[float] = None
        self._reference_pose = np.eye(4, dtype=np.float64)
        self._origin_rgb: Optional[np.ndarray] = None
        self._origin_depth: Optional[np.ndarray] = None
        self._origin_keypoints = []
        self._origin_descriptors: Optional[np.ndarray] = None
        self._origin_timestamp: Optional[float] = None
        self._origin_pose = np.eye(4, dtype=np.float64)
        self._active_is_origin = True
        self._last_timestamp: Optional[float] = None
        self._last_good_timestamp: Optional[float] = None
        self._pose = np.eye(4, dtype=np.float64)
        self._status = "initializing"

    def update(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        timestamp: float,
        rotation_prior: Optional[Any] = None,
    ) -> Dict[str, Any]:
        """Process one registered RGB-D frame and return diagnostic state.

        ``rgb`` is RGB ``uint8`` with shape ``(H, W, 3)``.  ``depth`` is a
        matching ``(H, W)`` numeric array in metres; non-finite and out-of-
        range samples are treated as holes.  Timestamps must be finite and
        strictly increasing.  ``rotation_prior`` is optional and may be the
        3x3/4x4 new-camera-to-active-reference rotation, a Rodrigues xyz
        vector, or an xyzw quaternion.  It should span
        ``reference_timestamp`` to this frame (the convention returned by
        ``tracking.gyro.relative_rotation``).  It gates weak grossly
        inconsistent fits; a strong vision consensus is always allowed to
        override it.
        """

        rgb, depth, current_timestamp = self._validate_frame(rgb, depth, timestamp)
        if self._last_timestamp is not None and current_timestamp <= self._last_timestamp:
            raise ValueError("timestamps must be finite and strictly increasing")
        prior = None if rotation_prior is None else _rotation_from_prior(rotation_prior)
        self._last_timestamp = current_timestamp

        keypoints, descriptors = self._detect(rgb)
        if self._reference_rgb is None:
            self._set_reference(rgb, depth, keypoints, descriptors, current_timestamp)
            self._last_good_timestamp = current_timestamp
            self._status = "initializing"
            return self._result(0, 0, float("inf"), None)

        # If startup was textureless, allow a better startup frame to become
        # the keyframe.  Once tracking succeeds, the active keyframe only moves
        # after a gated, sufficiently large motion.
        if self._reference_descriptors is None or len(self._reference_keypoints) < 4:
            if self._status == "initializing":
                self._set_reference(rgb, depth, keypoints, descriptors, current_timestamp)
                self._last_good_timestamp = current_timestamp
            return self._result(0, 0, float("inf"), None)

        candidate, match_count = self._estimate_for_keyframe(
            self._reference_depth,
            self._reference_keypoints,
            self._reference_descriptors,
            depth,
            keypoints,
            descriptors,
            prior,
        )
        reference_pose = self._reference_pose
        used_origin = False
        if candidate is None and not self._active_is_origin:
            # The supplied prior spans the active keyframe timestamp.  It is
            # therefore not valid for the older origin anchor; let the
            # origin vision fit stand on its own instead of applying a prior
            # with the wrong interval or direction.
            candidate, origin_matches = self._estimate_for_keyframe(
                self._origin_depth,
                self._origin_keypoints,
                self._origin_descriptors,
                depth,
                keypoints,
                descriptors,
                None,
            )
            if origin_matches:
                match_count = origin_matches
            reference_pose = self._origin_pose
            used_origin = candidate is not None
        if candidate is None:
            self._status = "lost"
            return self._result(match_count, 0, float("inf"), None)

        rotation, translation, inliers, rmse, solver = candidate
        world_pose = reference_pose.copy()
        world_pose[:3, :3] = reference_pose[:3, :3] @ rotation
        world_pose[:3, 3] = reference_pose[:3, :3] @ translation + reference_pose[:3, 3]
        if not self._motion_is_sane(world_pose, current_timestamp):
            self._status = "lost"
            return self._result(match_count, 0, float("inf"), None)
        self._pose = world_pose
        self._last_good_timestamp = current_timestamp
        self._status = "tracking"
        if descriptors is not None and len(keypoints) >= 4:
            relative_motion = float(np.linalg.norm(translation))
            relative_rotation = _rotation_difference_rad(np.eye(3), rotation)
            if (
                used_origin
                or relative_motion >= self.keyframe_translation_m
                or relative_rotation >= self.keyframe_rotation_rad
            ):
                self._set_reference(
                    rgb,
                    depth,
                    keypoints,
                    descriptors,
                    current_timestamp,
                    world_pose,
                    preserve_origin=True,
                )
        return self._result(match_count, inliers, rmse, solver)

    def _validate_frame(
        self, rgb: np.ndarray, depth: np.ndarray, timestamp: float
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        image = np.asarray(rgb)
        if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("rgb must be an uint8 array with shape (H, W, 3) in RGB order")
        if image.shape[0] == 0 or image.shape[1] == 0:
            raise ValueError("rgb must have non-zero height and width")
        range_image = np.asarray(depth)
        if range_image.ndim != 2 or range_image.shape != image.shape[:2]:
            raise ValueError("depth must have shape (H, W) matching rgb")
        if not np.issubdtype(range_image.dtype, np.number):
            raise ValueError("depth must be numeric metres")
        try:
            current_timestamp = float(timestamp)
        except (TypeError, ValueError) as exc:
            raise ValueError("timestamp must be a finite number") from exc
        if not np.isfinite(current_timestamp):
            raise ValueError("timestamp must be finite")
        return image, range_image.astype(np.float32, copy=False), current_timestamp

    def _detect(self, rgb: np.ndarray):
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        keypoints, descriptors = self._orb.detectAndCompute(gray, None)
        if keypoints is None:
            keypoints = []
        return keypoints, descriptors

    @property
    def reference_timestamp(self) -> Optional[float]:
        """Timestamp of the retained keyframe, for gyro-prior integration."""

        return self._reference_timestamp

    def _set_reference(
        self,
        rgb,
        depth,
        keypoints,
        descriptors,
        timestamp,
        world_pose=None,
        *,
        preserve_origin=False,
    ) -> None:
        self._reference_rgb = rgb.copy()
        self._reference_depth = depth.copy()
        self._reference_keypoints = list(keypoints)
        self._reference_timestamp = float(timestamp)
        self._reference_descriptors = (
            None if descriptors is None else descriptors.copy()
        )
        self._reference_pose = (
            np.eye(4, dtype=np.float64)
            if world_pose is None
            else np.asarray(world_pose, dtype=np.float64).copy()
        )
        if self._reference_pose.shape != (4, 4) or not np.isfinite(self._reference_pose).all():
            raise ValueError("world_pose must be a finite 4x4 transform")
        if not preserve_origin:
            self._origin_rgb = self._reference_rgb.copy()
            self._origin_depth = self._reference_depth.copy()
            self._origin_keypoints = list(self._reference_keypoints)
            self._origin_descriptors = (
                None
                if self._reference_descriptors is None
                else self._reference_descriptors.copy()
            )
            self._origin_timestamp = self._reference_timestamp
            self._origin_pose = self._reference_pose.copy()
            self._active_is_origin = True
        else:
            self._active_is_origin = False

    def _estimate_for_keyframe(
        self,
        reference_depth,
        reference_keypoints,
        reference_descriptors,
        current_depth,
        current_keypoints,
        current_descriptors,
        rotation_prior,
    ):
        matches = self._match_descriptors(current_descriptors, reference_descriptors)
        if not matches or reference_depth is None:
            return None, len(matches)
        (
            reference_points,
            current_points,
            pnp_reference_points,
            pnp_current_pixels,
        ) = self._correspondences(
            matches,
            current_depth,
            current_keypoints,
            reference_depth=reference_depth,
            reference_keypoints=reference_keypoints,
        )
        return (
            self._estimate(
                reference_points,
                current_points,
                pnp_reference_points,
                pnp_current_pixels,
                rotation_prior,
            ),
            len(matches),
        )

    def _match_descriptors(self, current_descriptors, reference_descriptors):
        if current_descriptors is None or reference_descriptors is None:
            return []
        if len(current_descriptors) < 2 or len(reference_descriptors) < 2:
            return []
        forward_knn = self._matcher.knnMatch(reference_descriptors, current_descriptors, k=2)
        reverse_knn = self._matcher.knnMatch(current_descriptors, reference_descriptors, k=2)

        def ratio_matches(knn):
            accepted = {}
            for pair in knn:
                if len(pair) < 2:
                    continue
                best, second = pair
                # Zero-distance ties are ambiguous repeated descriptors; an
                # isolated zero-distance best match remains valid.
                if second.distance <= 0.0 or best.distance >= self.ratio_test * second.distance:
                    continue
                previous = accepted.get(best.trainIdx)
                if previous is None or best.distance < previous.distance:
                    accepted[best.trainIdx] = best
            return accepted

        forward = ratio_matches(forward_knn)
        reverse = ratio_matches(reverse_knn)
        mutual = [
            match
            for match in forward.values()
            if reverse.get(match.queryIdx) is not None
            and reverse[match.queryIdx].queryIdx == match.trainIdx
        ]
        # Mutual matching removes many repeated-texture errors.  If a small
        # scene leaves too few mutual pairs, keep the ratio-filtered unique
        # forward set and let metric RANSAC make the final decision.
        selected = mutual if len(mutual) >= 4 else list(forward.values())
        return sorted(selected, key=lambda match: (float(match.distance), match.queryIdx))

    def _depth_at(self, depth: np.ndarray, point: Tuple[float, float]) -> Optional[float]:
        x, y = int(round(float(point[0]))), int(round(float(point[1])))
        height, width = depth.shape
        if x < 0 or y < 0 or x >= width or y >= height:
            return None
        # Median filtering over a tiny patch bridges registered depth holes
        # while keeping the operation bounded for Raspberry Pi frame rates.
        x0, x1 = max(0, x - 1), min(width, x + 2)
        y0, y1 = max(0, y - 1), min(height, y + 2)
        patch = depth[y0:y1, x0:x1]
        valid = patch[
            np.isfinite(patch)
            & (patch >= self.min_depth_m)
            & (patch <= self.max_depth_m)
        ]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _backproject(self, pixel: Tuple[float, float], depth_m: float) -> np.ndarray:
        u, v = float(pixel[0]), float(pixel[1])
        return np.array(
            [
                (u - self.K[0, 2]) * depth_m / self.K[0, 0],
                (v - self.K[1, 2]) * depth_m / self.K[1, 1],
                depth_m,
            ],
            dtype=np.float64,
        )

    def _correspondences(
        self,
        matches,
        depth,
        current_keypoints,
        *,
        reference_depth=None,
        reference_keypoints=None,
    ):
        reference_points = []
        current_points = []
        pnp_reference_points = []
        pnp_current_pixels = []
        if reference_depth is None:
            reference_depth = self._reference_depth
        if reference_keypoints is None:
            reference_keypoints = self._reference_keypoints
        if reference_depth is None:
            return (
                np.empty((0, 3), dtype=np.float64),
                np.empty((0, 3), dtype=np.float64),
                np.empty((0, 3), dtype=np.float64),
                np.empty((0, 2), dtype=np.float64),
            )
        for match in matches:
            reference_pixel = reference_keypoints[match.queryIdx].pt
            current_pixel = current_keypoints[match.trainIdx].pt
            reference_z = self._depth_at(reference_depth, reference_pixel)
            current_z = self._depth_at(depth, current_pixel)
            if reference_z is None:
                continue
            pnp_reference_points.append(self._backproject(reference_pixel, reference_z))
            pnp_current_pixels.append(current_pixel)
            if current_z is None:
                continue
            reference_points.append(self._backproject(reference_pixel, reference_z))
            current_points.append(self._backproject(current_pixel, current_z))
        return (
            np.asarray(reference_points, dtype=np.float64).reshape(-1, 3),
            np.asarray(current_points, dtype=np.float64).reshape(-1, 3),
            np.asarray(pnp_reference_points, dtype=np.float64).reshape(-1, 3),
            np.asarray(pnp_current_pixels, dtype=np.float64).reshape(-1, 2),
        )

    def _estimate(
        self,
        reference_points: np.ndarray,
        current_points: np.ndarray,
        pnp_reference_points: np.ndarray,
        pnp_current_pixels: np.ndarray,
        rotation_prior: Optional[np.ndarray],
    ):
        svd_candidate = None
        if self.method in {"svd", "auto"} and len(current_points) >= 3:
            try:
                rotation, translation, mask, rmse = rigid_transform_ransac(
                    current_points,
                    reference_points,
                    threshold_m=self.ransac_threshold_m,
                    iterations=self.ransac_iterations,
                    seed=self.seed,
                )
                if self._accepted(
                    rotation,
                    mask,
                    rmse,
                    len(current_points),
                    rotation_prior,
                    points=current_points,
                ):
                    svd_candidate = (rotation, translation, int(mask.sum()), rmse, "svd")
            except (ValueError, np.linalg.LinAlgError):
                pass

        if svd_candidate is not None:
            return svd_candidate
        if self.method == "svd":
            return None

        pnp_candidate = self._estimate_pnp(
            pnp_reference_points,
            pnp_current_pixels,
            rotation_prior,
        )
        return pnp_candidate

    def _estimate_pnp(
        self,
        reference_points: np.ndarray,
        current_pixels: np.ndarray,
        rotation_prior: Optional[np.ndarray],
    ):
        if len(reference_points) < 4 or len(current_pixels) != len(reference_points):
            return None
        if not np.isfinite(reference_points).all() or not np.isfinite(current_pixels).all():
            return None

        def retry_without_prior():
            if rotation_prior is None:
                return None
            return self._estimate_pnp(
                reference_points,
                current_pixels,
                None,
            )

        try:
            object_zero = reference_points - reference_points.mean(axis=0)
            if np.linalg.svd(object_zero, compute_uv=False)[1] <= _EPS:
                return None
            solve_kwargs = dict(
                iterationsCount=100,
                reprojectionError=3.0,
                confidence=0.99,
            )
            if rotation_prior is not None:
                prior_camera_rotation = rotation_prior.T
                prior_rvec, _ = cv2.Rodrigues(prior_camera_rotation)
                try:
                    success, rvec, tvec, pnp_inliers = cv2.solvePnPRansac(
                        reference_points.astype(np.float32),
                        current_pixels.astype(np.float32),
                        self.K,
                        None,
                        rvec=prior_rvec,
                        tvec=np.zeros((3, 1), dtype=np.float64),
                        useExtrinsicGuess=True,
                        flags=cv2.SOLVEPNP_ITERATIVE,
                        **solve_kwargs,
                    )
                except (TypeError, cv2.error):
                    try:
                        success, rvec, tvec, pnp_inliers = cv2.solvePnPRansac(
                            reference_points.astype(np.float32),
                            current_pixels.astype(np.float32),
                            self.K,
                            None,
                            flags=cv2.SOLVEPNP_EPNP,
                            **solve_kwargs,
                        )
                    except (TypeError, cv2.error, np.linalg.LinAlgError):
                        return retry_without_prior()
            else:
                success, rvec, tvec, pnp_inliers = cv2.solvePnPRansac(
                    reference_points.astype(np.float32),
                    current_pixels.astype(np.float32),
                    self.K,
                    None,
                    flags=cv2.SOLVEPNP_EPNP,
                    **solve_kwargs,
                )
        except (cv2.error, np.linalg.LinAlgError):
            return None
        if not success or pnp_inliers is None or len(pnp_inliers) < 4:
            return retry_without_prior()
        inlier_indices = np.asarray(pnp_inliers, dtype=np.int64).reshape(-1)
        if (
            np.any(inlier_indices < 0)
            or np.any(inlier_indices >= len(reference_points))
        ):
            return retry_without_prior()

        # Refine the RANSAC solution against its inlier pixels when available
        # (OpenCV 4.6+).  RANSAC remains the gate if the refinement routine is
        # absent or rejects the starting solution.
        if hasattr(cv2, "solvePnPRefineLM"):
            try:
                rvec, tvec = cv2.solvePnPRefineLM(
                    reference_points[inlier_indices].astype(np.float32),
                    current_pixels[inlier_indices].astype(np.float32),
                    self.K,
                    None,
                    rvec,
                    tvec,
                    criteria=(
                        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT,
                        20,
                        1.0e-6,
                    ),
                )
            except (TypeError, cv2.error, np.linalg.LinAlgError):
                pass

        try:
            camera_rotation, _ = cv2.Rodrigues(rvec)
            projected, _ = cv2.projectPoints(
                reference_points.astype(np.float32), rvec, tvec, self.K, None
            )
        except (cv2.error, np.linalg.LinAlgError):
            return None
        pixel_residual = np.linalg.norm(
            projected.reshape(-1, 2).astype(np.float64) - current_pixels, axis=1
        )
        mask = pixel_residual <= 3.0
        if int(mask.sum()) < 4:
            return retry_without_prior()
        camera_points = (
            reference_points @ camera_rotation.T
            + np.asarray(tvec, dtype=np.float64).reshape(1, 3)
        )
        if (
            not np.isfinite(camera_points).all()
            or np.any(camera_points[mask, 2] <= self.min_depth_m * 0.5)
        ):
            return retry_without_prior()
        rotation = camera_rotation.T
        translation = -rotation @ np.asarray(tvec, dtype=np.float64).reshape(3)

        # Keep this alternate solver reprojection driven.  Current depth is
        # often noisier than the registered RGB pixels; replacing a good PnP
        # pose with an unconditional 3-D fit would reintroduce that noise.
        rmse = float(np.sqrt(np.mean(np.square(pixel_residual[mask]))))
        if not np.isfinite(rmse) or not self._accepted(
            rotation,
            mask,
            rmse,
            len(reference_points),
            rotation_prior,
            pnp=True,
            points=reference_points,
        ):
            return retry_without_prior()
        return rotation, translation, int(mask.sum()), rmse, "pnp"

    def _accepted(
        self,
        rotation: np.ndarray,
        mask: np.ndarray,
        rmse: float,
        correspondence_count: int,
        rotation_prior: Optional[np.ndarray],
        *,
        pnp: bool = False,
        points: Optional[np.ndarray] = None,
    ) -> bool:
        inliers = int(mask.sum())
        if inliers < self.min_inliers or correspondence_count <= 0:
            return False
        if inliers / float(correspondence_count) < self.min_inlier_ratio:
            return False
        if points is not None:
            inlier_points = np.asarray(points)[mask]
            if len(inlier_points) < 3:
                return False
            spread = np.linalg.svd(
                inlier_points - inlier_points.mean(axis=0), compute_uv=False
            )
            if spread[1] < self.min_geometry_spread_m:
                return False
        # PnP's fallback error is in pixels when current depth is unavailable.
        if (not pnp or correspondence_count == 0) and rmse > self.max_rmse_m:
            return False
        if not np.isfinite(rotation).all() or np.linalg.det(rotation) <= 0.0:
            return False
        if rotation_prior is not None:
            difference = _rotation_difference_rad(rotation_prior, rotation)
            # A high-consensus vision fit wins over a stale gyro prior.  The
            # prior gates only weak fits that could otherwise jump wildly.
            if (
                difference > self.rotation_prior_gate_rad
                and inliers / float(correspondence_count) < 0.75
            ):
                return False
        return True

    def _motion_is_sane(self, world_pose: np.ndarray, timestamp: float) -> bool:
        """Reject a numerically consistent fit that implies impossible motion."""

        if self._last_good_timestamp is None:
            return True
        dt = float(timestamp - self._last_good_timestamp)
        if dt <= 0.0 or not np.isfinite(dt):
            return False
        translation_speed = float(
            np.linalg.norm(world_pose[:3, 3] - self._pose[:3, 3]) / dt
        )
        rotation_speed = _rotation_difference_rad(
            self._pose[:3, :3], world_pose[:3, :3]
        ) / dt
        return (
            np.isfinite(translation_speed)
            and np.isfinite(rotation_speed)
            and translation_speed <= self.max_translation_speed_m_s
            and rotation_speed <= self.max_rotation_speed_rad_s
        )

    def _result(
        self, matches: int, inliers: int, rmse: float, solver: Optional[str]
    ) -> Dict[str, Any]:
        return {
            "pose": self._pose.copy(),
            "status": self._status,
            "inliers": int(inliers),
            "matches": int(matches),
            "rmse": float(rmse),
            "solver": solver,
        }


__all__ = ["RGBDOdometry", "rigid_transform_ransac"]
