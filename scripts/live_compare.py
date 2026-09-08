#!/usr/bin/env python3
"""Capture a small RGB-D window in RAM and compare the two odometry methods.

The probe deliberately does not use the portal recording endpoint.  It pairs
ROS RGB/depth messages the same way as ``tracking.server`` and only emits
aggregate JSON after both methods have consumed the same in-memory frames.
"""

from __future__ import annotations

import argparse
from collections import Counter, deque
import json
from pathlib import Path
import sys
import time

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


PAIR_TOLERANCE_S = 0.035


def _percentile(values, percentile):
    if not values:
        return None
    return round(float(np.percentile(np.asarray(values, dtype=np.float64), percentile)), 3)


def _rotation_angle_deg(rotation):
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def _pose_metrics(poses):
    if not poses:
        return {
            "max_translation_from_start_m": None,
            "final_translation_from_start_m": None,
            "max_step_translation_m": None,
            "max_rotation_from_start_deg": None,
        }
    first = poses[0]
    translations = [np.linalg.norm(p[:3, 3] - first[:3, 3]) for p in poses]
    rotations = [_rotation_angle_deg(first[:3, :3].T @ p[:3, :3]) for p in poses]
    steps = [np.linalg.norm(poses[i][:3, 3] - poses[i - 1][:3, 3]) for i in range(1, len(poses))]
    return {
        "max_translation_from_start_m": round(float(max(translations)), 5),
        "final_translation_from_start_m": round(float(translations[-1]), 5),
        "max_step_translation_m": round(float(max(steps)), 5) if steps else 0.0,
        "max_rotation_from_start_deg": round(float(max(rotations)), 4),
    }


def _image_checks(frames):
    if not frames:
        return {
            "rgb_pixel_mean_abs_first_last": None,
            "rgb_pixel_mean_abs_adjacent_median": None,
            "depth_median_abs_first_last_m": None,
            "depth_median_abs_adjacent_median_m": None,
            "depth_valid_percent_first": None,
            "depth_valid_percent_last": None,
            "depth_median_first_m": None,
            "depth_median_last_m": None,
        }

    rgb_diffs = []
    depth_diffs = []
    for previous, current in zip(frames, frames[1:]):
        rgb_diffs.append(float(np.mean(np.abs(current[1].astype(np.float32) - previous[1]))))
        previous_depth, current_depth = previous[2], current[2]
        valid = np.isfinite(previous_depth) & np.isfinite(current_depth)
        valid &= (previous_depth > 0.0) & (current_depth > 0.0)
        if np.any(valid):
            depth_diffs.append(float(np.median(np.abs(current_depth[valid] - previous_depth[valid]))))

    first_rgb, last_rgb = frames[0][1], frames[-1][1]
    first_depth, last_depth = frames[0][2], frames[-1][2]
    rgb_first_last = float(np.mean(np.abs(last_rgb.astype(np.float32) - first_rgb)))
    valid = np.isfinite(first_depth) & np.isfinite(last_depth)
    valid &= (first_depth > 0.0) & (last_depth > 0.0)
    depth_first_last = (
        float(np.median(np.abs(last_depth[valid] - first_depth[valid])))
        if np.any(valid)
        else None
    )
    def valid_percent(depth):
        return 100.0 * float(np.mean(np.isfinite(depth) & (depth > 0.0)))

    def valid_median(depth):
        valid_depth = depth[np.isfinite(depth) & (depth > 0.0)]
        return float(np.median(valid_depth)) if valid_depth.size else None

    return {
        "rgb_pixel_mean_abs_first_last": round(rgb_first_last, 4),
        "rgb_pixel_mean_abs_adjacent_median": _percentile(rgb_diffs, 50),
        "depth_median_abs_first_last_m": round(depth_first_last, 6) if depth_first_last is not None else None,
        "depth_median_abs_adjacent_median_m": _percentile(depth_diffs, 50),
        "depth_valid_percent_first": round(valid_percent(first_depth), 3),
        "depth_valid_percent_last": round(valid_percent(last_depth), 3),
        "depth_median_first_m": round(valid_median(first_depth), 5) if valid_median(first_depth) is not None else None,
        "depth_median_last_m": round(valid_median(last_depth), 5) if valid_median(last_depth) is not None else None,
    }


def _compare_methods(frames, camera_matrix):
    from tracking.odometry import RGBDOdometry

    reports = {}
    trajectories = {}
    for method in ("svd", "pnp"):
        odometry = RGBDOdometry(camera_matrix, method=method)
        statuses = Counter()
        latencies = []
        poses = []
        inliers = []
        matches = []
        failures = []
        for index, (stamp, rgb, depth, _sync_error) in enumerate(frames):
            started = time.perf_counter()
            try:
                result = odometry.update(rgb, depth, stamp)
                statuses[str(result.get("status", "unknown"))] += 1
                pose = np.asarray(result.get("pose"), dtype=np.float64)
                if pose.shape == (4, 4) and np.isfinite(pose).all():
                    poses.append(pose.copy())
                else:
                    poses.append(None)
                if result.get("status") == "tracking":
                    inliers.append(int(result.get("inliers", 0)))
                    matches.append(int(result.get("matches", 0)))
            except Exception as error:  # Keep the other method comparable.
                statuses["error"] += 1
                failures.append({"index": index, "error": str(error)})
                poses.append(None)
            latencies.append((time.perf_counter() - started) * 1000.0)

        trajectories[method] = poses
        report = {
            "frames": len(frames),
            "tracked": int(statuses.get("tracking", 0)),
            "lost": int(statuses.get("lost", 0)),
            "initializing": int(statuses.get("initializing", 0)),
            "errors": failures,
            "status_counts": dict(statuses),
            "p50_ms": _percentile(latencies, 50),
            "p95_ms": _percentile(latencies, 95),
            "max_ms": round(float(max(latencies)), 3) if latencies else None,
            "median_inliers": _percentile(inliers, 50),
            "median_matches": _percentile(matches, 50),
        }
        report.update(_pose_metrics([pose for pose in poses if pose is not None]))
        reports[method] = report

    svd_poses, pnp_poses = trajectories["svd"], trajectories["pnp"]
    translation_differences = []
    rotation_differences = []
    for svd_pose, pnp_pose in zip(svd_poses, pnp_poses):
        if svd_pose is None or pnp_pose is None:
            continue
        translation_differences.append(float(np.linalg.norm(svd_pose[:3, 3] - pnp_pose[:3, 3])))
        rotation_differences.append(_rotation_angle_deg(svd_pose[:3, :3].T @ pnp_pose[:3, :3]))
    reports["same_frame_disagreement"] = {
        "paired_pose_count": min(len(svd_poses), len(pnp_poses)),
        "median_translation_m": _percentile(translation_differences, 50),
        "max_translation_m": round(float(max(translation_differences)), 5) if translation_differences else None,
        "median_rotation_deg": _percentile(rotation_differences, 50),
        "max_rotation_deg": round(float(max(rotation_differences)), 4) if rotation_differences else None,
    }
    return reports


def _capture(pair_count, timeout_s):
    import rclpy
    import cv2
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    rclpy.init(args=None)
    node = Node("live_odometry_compare")
    bridge = CvBridge()
    queues = {"rgb": deque(maxlen=8), "depth": deque(maxlen=8)}
    calibration = {}
    frames = []
    counters = Counter()

    def info(message):
        calibration["K"] = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        calibration["D"] = np.asarray(message.d, dtype=np.float64)

    def on_image(kind, message):
        stamp = float(message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)
        queues[kind].append((stamp, message))
        other = "depth" if kind == "rgb" else "rgb"
        if not queues[other]:
            return
        match = min(queues[other], key=lambda pair: abs(pair[0] - stamp))
        error = abs(match[0] - stamp)
        if error > PAIR_TOLERANCE_S:
            counters["pair_over_tolerance"] += 1
            return
        queues[other].remove(match)
        queues[kind].pop()
        rgb_message, depth_message = (message, match[1]) if kind == "rgb" else (match[1], message)
        try:
            if rgb_message.header.frame_id != depth_message.header.frame_id:
                counters["frame_id_mismatch"] += 1
                return
            camera_matrix = calibration.get("K")
            distortion = calibration.get("D")
            if (camera_matrix is None or not np.isfinite(camera_matrix).all()
                    or camera_matrix[0,0] <= 0 or camera_matrix[1,1] <= 0):
                counters["missing_calibration"] += 1
                return
            rgb = np.asarray(bridge.imgmsg_to_cv2(rgb_message, "rgb8"), dtype=np.uint8)
            depth = np.asarray(bridge.imgmsg_to_cv2(depth_message, "passthrough"), dtype=np.float32)
            if depth_message.encoding == "16UC1":
                depth *= 0.001
            elif depth_message.encoding != "32FC1":
                counters["unsupported_depth_encoding"] += 1
                return
            if rgb.shape[:2] != depth.shape:
                counters["shape_mismatch"] += 1
                return
            if distortion is not None and np.any(distortion):
                height, width = depth.shape
                mx, my = cv2.initUndistortRectifyMap(
                    camera_matrix, distortion, None, camera_matrix, (width, height), cv2.CV_32FC1
                )
                rgb = cv2.remap(rgb, mx, my, cv2.INTER_LINEAR)
                depth = cv2.remap(depth, mx, my, cv2.INTER_NEAREST)
            # The copies make the bounded in-memory ownership explicit after
            # CvBridge returns buffers backed by the ROS message.
            rgb_stamp = rgb_message.header.stamp.sec + rgb_message.header.stamp.nanosec * 1e-9
            frames.append((rgb_stamp, rgb.copy(), depth.copy(), float(error)))
            counters["paired"] += 1
        except Exception as error:
            counters["conversion_errors"] += 1
            counters["last_error"] = str(error)

    node.create_subscription(CameraInfo, "/camera/color/camera_info", info, qos_profile_sensor_data)
    node.create_subscription(Image, "/camera/color/image_raw", lambda message: on_image("rgb", message), qos_profile_sensor_data)
    node.create_subscription(Image, "/camera/depth/image_raw", lambda message: on_image("depth", message), qos_profile_sensor_data)

    started = time.monotonic()
    try:
        while len(frames) < pair_count and time.monotonic() - started < timeout_s:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    frames.sort(key=lambda frame: frame[0])
    return frames, calibration.get("K"), counters, time.monotonic() - started


def run(args):
    import cv2

    cv2.setNumThreads(2)
    frames, camera_matrix, counters, capture_seconds = _capture(args.pairs, args.timeout)
    report = {
        "source": "live ROS RGB-D in-memory capture",
        "requested_pairs": args.pairs,
        "captured_pairs": len(frames),
        "capture_seconds": round(float(capture_seconds), 3),
        "timed_out": len(frames) < args.pairs,
        "pairing": {
            "tolerance_ms": PAIR_TOLERANCE_S * 1000.0,
            "sync_p50_ms": _percentile([frame[3] * 1000.0 for frame in frames], 50),
            "sync_p95_ms": _percentile([frame[3] * 1000.0 for frame in frames], 95),
            "timestamp_span_s": round(float(frames[-1][0] - frames[0][0]), 6) if len(frames) > 1 else 0.0,
        },
        "capture_counters": dict(counters),
        "image_checks": _image_checks(frames),
    }
    if camera_matrix is not None:
        report["camera_matrix"] = np.asarray(camera_matrix, dtype=np.float64).round(6).tolist()
    if frames and camera_matrix is not None:
        report["shape"] = {"rgb": list(frames[0][1].shape), "depth": list(frames[0][2].shape)}
        report["results"] = _compare_methods(frames, camera_matrix)
    else:
        report["error"] = "no paired frames with valid camera calibration"
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pairs", type=int, default=12, help="number of pairs to retain in RAM (default: 12)")
    parser.add_argument("--timeout", type=float, default=45.0, help="capture timeout in seconds (default: 45)")
    args = parser.parse_args()
    if args.pairs < 2 or args.pairs > 20:
        parser.error("--pairs must be between 2 and 20")
    if not np.isfinite(args.timeout) or args.timeout <= 0.0:
        parser.error("--timeout must be positive")
    try:
        result = run(args)
        print(json.dumps(result, allow_nan=False, separators=(",", ":")))
        return 0 if "error" not in result else 2
    except Exception as error:
        print(json.dumps({"error": str(error)}, allow_nan=False, separators=(",", ":")))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
