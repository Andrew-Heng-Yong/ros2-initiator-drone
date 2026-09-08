"""Benchmark the RGB-D odometry against a TUM RGB-D sequence.

Usage::

    python3 scripts/evaluate_tum.py /path/to/rgbd_dataset_freiburg1_xyz

The report is the only stdout output, so it can be redirected directly to a
JSON file.  TUM depth pixels are converted from millimetres-like units with
the dataset's documented ``/5000`` scale.
"""

from __future__ import annotations

import argparse
import bisect
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

DEFAULT_K = np.array(
    [[525.0, 0.0, 319.5], [0.0, 525.0, 239.5], [0.0, 0.0, 1.0]],
    dtype=np.float64,
)
ASSOCIATION_TOLERANCE_S = 0.020
GROUNDTRUTH_NEAREST_TOLERANCE_S = 0.010


@dataclass(frozen=True)
class StreamEntry:
    timestamp: float
    path: Path


@dataclass(frozen=True)
class TruthEntry:
    timestamp: float
    translation: np.ndarray
    quaternion_xyzw: np.ndarray


@dataclass(frozen=True)
class Frame:
    timestamp: float
    rgb_path: Path
    depth_path: Path


def _data_lines(path: Path) -> Iterable[list[str]]:
    with path.open("r", encoding="utf-8") as source:
        for line in source:
            fields = line.split()
            if fields and not fields[0].startswith("#"):
                yield fields


def _read_stream(path: Path, root: Path) -> list[StreamEntry]:
    entries: list[StreamEntry] = []
    for fields in _data_lines(path):
        if len(fields) < 2:
            continue
        try:
            timestamp = float(fields[0])
        except ValueError:
            continue
        if not np.isfinite(timestamp):
            continue
        entries.append(StreamEntry(timestamp, root / fields[1]))
    entries.sort(key=lambda entry: entry.timestamp)
    return entries


def associate_streams(
    rgb: Sequence[StreamEntry],
    depth: Sequence[StreamEntry],
    tolerance_s: float = ASSOCIATION_TOLERANCE_S,
) -> list[Frame]:
    """Pair each RGB entry with at most one nearest depth entry."""

    if tolerance_s <= 0.0 or not np.isfinite(tolerance_s):
        raise ValueError("association tolerance must be positive and finite")
    depth_times = [entry.timestamp for entry in depth]
    used: set[int] = set()
    pairs: list[Frame] = []
    for rgb_entry in rgb:
        index = bisect.bisect_left(depth_times, rgb_entry.timestamp)
        candidates = sorted(
            (candidate for candidate in (index - 1, index) if 0 <= candidate < len(depth)),
            key=lambda candidate: abs(depth_times[candidate] - rgb_entry.timestamp),
        )
        for candidate in candidates:
            if candidate in used:
                continue
            if abs(depth_times[candidate] - rgb_entry.timestamp) <= tolerance_s:
                used.add(candidate)
                pairs.append(Frame(rgb_entry.timestamp, rgb_entry.path, depth[candidate].path))
            break
    return pairs


def read_groundtruth(path: Path) -> list[TruthEntry]:
    entries: list[TruthEntry] = []
    for fields in _data_lines(path):
        if len(fields) < 8:
            continue
        try:
            values = np.asarray([float(value) for value in fields[:8]], dtype=np.float64)
        except ValueError:
            continue
        if not np.isfinite(values).all():
            continue
        quaternion = values[4:8]
        norm = float(np.linalg.norm(quaternion))
        if norm <= 1.0e-12:
            continue
        entries.append(TruthEntry(values[0], values[1:4], quaternion / norm))
    entries.sort(key=lambda entry: entry.timestamp)
    return entries


def _quaternion_slerp(first: np.ndarray, second: np.ndarray, fraction: float) -> np.ndarray:
    second = second.copy()
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second = -second
        dot = -dot
    if dot > 0.9995:
        result = first + fraction * (second - first)
        return result / np.linalg.norm(result)
    angle = float(np.arccos(np.clip(dot, -1.0, 1.0)))
    sine = float(np.sin(angle))
    first_weight = np.sin((1.0 - fraction) * angle) / sine
    second_weight = np.sin(fraction * angle) / sine
    result = first_weight * first + second_weight * second
    return result / np.linalg.norm(result)


def _quaternion_to_rotation(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = quaternion
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def truth_at(
    entries: Sequence[TruthEntry],
    timestamp: float,
    nearest_tolerance_s: float = GROUNDTRUTH_NEAREST_TOLERANCE_S,
    timestamps: Sequence[float] | None = None,
) -> tuple[np.ndarray, str] | None:
    """Return a T_WC pose and whether it was interpolated or nearest matched."""

    if not entries:
        return None
    times = timestamps if timestamps is not None else [entry.timestamp for entry in entries]
    index = bisect.bisect_left(times, timestamp)
    if index < len(entries) and entries[index].timestamp == timestamp:
        left = right = entries[index]
        fraction = 0.0
        mode = "interpolated"
    elif index == 0:
        nearest = entries[0]
        if abs(nearest.timestamp - timestamp) > nearest_tolerance_s:
            return None
        fraction = 0.0
        mode = "nearest"
        left = right = nearest
    elif index == len(entries):
        nearest = entries[-1]
        if abs(nearest.timestamp - timestamp) > nearest_tolerance_s:
            return None
        fraction = 1.0
        mode = "nearest"
        left = right = nearest
    else:
        left, right = entries[index - 1], entries[index]
        fraction = (timestamp - left.timestamp) / (right.timestamp - left.timestamp)
        mode = "interpolated"
    translation = left.translation + fraction * (right.translation - left.translation)
    quaternion = _quaternion_slerp(left.quaternion_xyzw, right.quaternion_xyzw, fraction)
    pose = np.eye(4, dtype=np.float64)
    pose[:3, :3] = _quaternion_to_rotation(quaternion)
    pose[:3, 3] = translation
    return pose, mode


def _resolve_dataset(path: Path) -> Path:
    path = path.expanduser().resolve()
    candidates = [path]
    if path.is_dir():
        candidates.extend(child for child in path.iterdir() if child.is_dir())
    for candidate in candidates:
        if all((candidate / name).is_file() for name in ("rgb.txt", "depth.txt", "groundtruth.txt")):
            return candidate
    raise FileNotFoundError("dataset must contain rgb.txt, depth.txt, and groundtruth.txt")


def _load_frame(frame: Frame) -> tuple[np.ndarray, np.ndarray]:
    import cv2

    rgb_bgr = cv2.imread(str(frame.rgb_path), cv2.IMREAD_COLOR)
    depth_raw = cv2.imread(str(frame.depth_path), cv2.IMREAD_UNCHANGED)
    if rgb_bgr is None:
        raise OSError(f"could not read RGB image {frame.rgb_path}")
    if depth_raw is None:
        raise OSError(f"could not read depth image {frame.depth_path}")
    rgb = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB)
    depth = depth_raw.astype(np.float32, copy=False) / 5000.0
    if depth.ndim != 2 or depth.shape != rgb.shape[:2]:
        raise ValueError("RGB and depth images have different shapes")
    return rgb, depth


def _rmse(values: Sequence[float]) -> float | None:
    return float(np.sqrt(np.mean(np.square(values)))) if values else None


def _rotation_error_deg(estimated: np.ndarray, truth: np.ndarray) -> float:
    relative = estimated[:3, :3].T @ truth[:3, :3]
    cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def _latency_report(values: Sequence[float]) -> dict[str, float | None]:
    if not values:
        return {"p50_ms": None, "p95_ms": None, "max_ms": None}
    return {
        "p50_ms": float(np.percentile(values, 50)),
        "p95_ms": float(np.percentile(values, 95)),
        "max_ms": float(np.max(values)),
    }


def evaluate_method(
    frames: Sequence[Frame],
    truth_entries: Sequence[TruthEntry],
    method: str,
    K: np.ndarray = DEFAULT_K,
) -> dict[str, object]:
    """Run one odometry method, preserving its last pose across failures."""

    from tracking.odometry import RGBDOdometry

    odometry = RGBDOdometry(K, method=method)
    last_pose = np.eye(4, dtype=np.float64)
    truth_origin: np.ndarray | None = None
    truth_origin_timestamp: float | None = None
    all_position_errors: list[float] = []
    all_rotation_errors: list[float] = []
    tracked_position_errors: list[float] = []
    tracked_rotation_errors: list[float] = []
    latencies: list[float] = []
    status_counts: dict[str, int] = {}
    truth_modes = {"interpolated": 0, "nearest": 0}
    exception_count = 0
    last_error: str | None = None
    truth_times = [entry.timestamp for entry in truth_entries]
    truth_origin_inverse: np.ndarray | None = None

    for frame in frames:
        truth_result = truth_at(truth_entries, frame.timestamp, timestamps=truth_times)
        try:
            rgb, depth = _load_frame(frame)
            started = time.perf_counter()
            output = odometry.update(rgb, depth, frame.timestamp)
            latencies.append((time.perf_counter() - started) * 1000.0)
            status = str(output.get("status", "error"))
            pose = np.asarray(output.get("pose", last_pose), dtype=np.float64)
            if pose.shape != (4, 4) or not np.isfinite(pose).all():
                raise ValueError("odometry returned an invalid pose")
            last_pose = pose.copy()
        except Exception as error:  # Keep evaluating later frames after one bad frame.
            exception_count += 1
            last_error = str(error)
            status = "error"
            pose = last_pose
        status_counts[status] = status_counts.get(status, 0) + 1

        if truth_result is None:
            continue
        truth_pose, truth_mode = truth_result
        truth_modes[truth_mode] += 1
        if truth_origin is None:
            truth_origin = truth_pose
            truth_origin_inverse = np.linalg.inv(truth_origin)
            truth_origin_timestamp = frame.timestamp
        truth_relative = truth_origin_inverse @ truth_pose
        position_error = float(np.linalg.norm(pose[:3, 3] - truth_relative[:3, 3]))
        rotation_error = _rotation_error_deg(pose, truth_relative)
        all_position_errors.append(position_error)
        all_rotation_errors.append(rotation_error)
        if status == "tracking":
            tracked_position_errors.append(position_error)
            tracked_rotation_errors.append(rotation_error)

    frames_count = len(frames)
    tracked = status_counts.get("tracking", 0)
    initializing = status_counts.get("initializing", 0)
    lost = frames_count - tracked - initializing
    return {
        "frames": frames_count,
        "tracked_frames": tracked,
        "lost_frames": max(0, lost),
        "initializing_frames": initializing,
        "error_frames": exception_count,
        "tracking_ratio": tracked / frames_count if frames_count else 0.0,
        "tracked_ratio": tracked / frames_count if frames_count else 0.0,
        "truth_frames": len(all_position_errors),
        "truth_origin_timestamp": truth_origin_timestamp,
        "truth_interpolated_frames": truth_modes["interpolated"],
        "truth_nearest_frames": truth_modes["nearest"],
        "position_rmse_m": _rmse(all_position_errors),
        "orientation_rmse_deg": _rmse(all_rotation_errors),
        "rotation_rmse_deg": _rmse(all_rotation_errors),
        "tracked_position_rmse_m": _rmse(tracked_position_errors),
        "tracked_orientation_rmse_deg": _rmse(tracked_rotation_errors),
        "latency": _latency_report(latencies),
        "status_counts": status_counts,
        "failure_pose_hold": True,
        "last_error": last_error,
    }


def evaluate_dataset(
    dataset: Path,
    *,
    stride: int = 6,
    limit: int = 0,
    method: str = "both",
) -> dict[str, object]:
    if stride < 1 or limit < 0:
        raise ValueError("stride must be >= 1 and limit must be >= 0")
    root = _resolve_dataset(dataset)
    rgb_entries = _read_stream(root / "rgb.txt", root)
    depth_entries = _read_stream(root / "depth.txt", root)
    truth_entries = read_groundtruth(root / "groundtruth.txt")
    pairs = associate_streams(rgb_entries, depth_entries)
    selected = pairs[::stride]
    if limit:
        selected = selected[:limit]
    methods = [method] if method in {"svd", "pnp"} else ["svd", "pnp"]
    results = {
        selected_method: evaluate_method(selected, truth_entries, selected_method)
        for selected_method in methods
    }
    return {
        "dataset": str(root),
        "camera": "TUM fr1_xyz default K; no undistortion",
        "rgb_entries": len(rgb_entries),
        "depth_entries": len(depth_entries),
        "paired_frames": len(pairs),
        "selected_frames": len(selected),
        "paired_rgb_ratio": len(pairs) / len(rgb_entries) if rgb_entries else 0.0,
        "stride": stride,
        "limit": limit,
        "association_tolerance_ms": ASSOCIATION_TOLERANCE_S * 1000.0,
        "groundtruth_nearest_tolerance_ms": GROUNDTRUTH_NEAREST_TOLERANCE_S * 1000.0,
        "methods": results,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path, help="extracted TUM RGB-D dataset directory")
    parser.add_argument("--stride", type=int, default=6, help="evaluate every N associated frames (default: 6)")
    parser.add_argument("--limit", type=int, default=0, help="maximum selected frames; 0 evaluates all")
    parser.add_argument("--method", choices=("both", "svd", "pnp"), default="both")
    args = parser.parse_args(argv)
    try:
        report = evaluate_dataset(args.dataset, stride=args.stride, limit=args.limit, method=args.method)
    except Exception as error:
        print(f"evaluate_tum: {error}", file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
