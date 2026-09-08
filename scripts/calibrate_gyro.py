#!/usr/bin/env python3
"""Estimate a disabled camera-to-gyro calibration from an RGB-D recording.

The recorder stores corrected gyro samples.  This tool removes that recorded
scale and timestamp offset once, subtracts the exported bias, and fits a new
scale, mount rotation, and sensor-to-camera time offset against independent
RGB-D PnP rotations.  It never enables fusion in its output.
"""

from __future__ import annotations

import argparse
from collections import Counter
import glob
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tracking.gyro import integrate_angular_velocity, so3_exp
from tracking.odometry import RGBDOdometry


class CalibrationError(ValueError):
    pass


def _log_rotation(rotation):
    vector, _ = cv2.Rodrigues(np.asarray(rotation, dtype=np.float64))
    return vector[:, 0]


def _angle_deg(rotation):
    return float(np.degrees(np.linalg.norm(_log_rotation(rotation))))


def _kabsch(source, target):
    covariance = np.asarray(source, dtype=float).T @ np.asarray(target, dtype=float)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1] *= -1.0
        rotation = vt.T @ u.T
    return rotation


def _integrated_vector(samples, t0, t1, mount, scale, offset, max_gap):
    if isinstance(samples, np.ndarray) and samples.ndim == 2 and samples.shape[1] == 4:
        # Keep interpolation neighbours; avoid re-parsing the whole recording
        # for every short interval in each calibration trial.
        start, end = sorted((t0 - offset, t1 - offset))
        lo = max(0, np.searchsorted(samples[:, 0], start, side="left") - 1)
        hi = np.searchsorted(samples[:, 0], end, side="right") + 1
        samples = samples[lo:hi]
        samples = [(row[0], row[1:]) for row in samples]
    rotation = integrate_angular_velocity(
        samples,
        t0 - offset,
        t1 - offset,
        rotation_camera_from_gyro=mount,
        scale=scale,
        max_gap=max_gap,
    )
    return None if rotation is None else _log_rotation(rotation)


def _residual(samples, pairs, mount, scale, offset, max_gap):
    values = []
    for t0, t1, visual_rotation in pairs:
        estimate = _integrated_vector(samples, t0, t1, mount, scale, offset, max_gap)
        if estimate is None:
            return None
        values.append(_log_rotation(visual_rotation) - estimate)
    return np.asarray(values, dtype=float)


def _refine(samples, pairs, mount, scale, offset, max_gap):
    for _ in range(6):
        residual = _residual(samples, pairs, mount, scale, offset, max_gap)
        if residual is None:
            return mount, scale
        flat = residual.reshape(-1)
        jacobian = np.empty((flat.size, 4), dtype=float)
        epsilon = 1.0e-5
        for axis in range(3):
            delta = np.zeros(3)
            delta[axis] = epsilon
            perturbed = _residual(
                samples, pairs, so3_exp(delta) @ mount, scale, offset, max_gap
            )
            jacobian[:, axis] = ((perturbed - residual) / epsilon).reshape(-1)
        perturbed = _residual(
            samples, pairs, mount, scale * np.exp(epsilon), offset, max_gap
        )
        jacobian[:, 3] = ((perturbed - residual) / epsilon).reshape(-1)
        step, *_ = np.linalg.lstsq(jacobian, -flat, rcond=None)
        step = np.clip(step, -0.25, 0.25)
        if np.linalg.norm(step) < 1.0e-7:
            break
        mount = so3_exp(step[:3]) @ mount
        scale *= float(np.exp(step[3]))
        if not np.isfinite(scale) or scale <= 0.0:
            break
    return mount, scale


def _fit_at_offset(samples, pairs, offset, max_gap):
    visual = np.asarray([_log_rotation(pair[2]) for pair in pairs])
    sensor = []
    for t0, t1, _ in pairs:
        vector = _integrated_vector(samples, t0, t1, np.eye(3), 1.0, offset, max_gap)
        if vector is None:
            return None
        sensor.append(vector)
    sensor = np.asarray(sensor)
    if np.linalg.matrix_rank(sensor, tol=1.0e-5) < 2:
        return None
    mount = _kabsch(sensor, visual)
    rotated = sensor @ mount.T
    scale = float(np.sum(rotated * visual) / max(np.sum(sensor * sensor), 1.0e-12))
    if not np.isfinite(scale) or scale <= 0.0:
        return None
    mount, scale = _refine(samples, pairs, mount, scale, offset, max_gap)
    return mount, scale


def _rmse_deg(residual):
    if residual is None or not residual.size:
        return None
    return float(np.degrees(np.sqrt(np.mean(np.sum(residual * residual, axis=1)))))


def fit_calibration(
    samples,
    visual_pairs,
    *,
    offsets=None,
    max_gap=0.25,
    min_intervals=6,
):
    """Fit mount, scale, and time offset from ``(t0, t1, R_camera)`` pairs.

    ``samples`` must be an ``(N, 4)`` array with raw sensor timestamps and
    bias-subtracted rad/s columns.  The returned dictionary is JSON-friendly.
    """

    samples = np.asarray(samples, dtype=float)
    pairs = list(visual_pairs)
    if samples.ndim != 2 or samples.shape[1] != 4 or not np.isfinite(samples).all():
        raise CalibrationError("gyro samples must be finite with shape (N, 4)")
    samples = samples[np.argsort(samples[:, 0], kind="stable")]
    if len(pairs) < min_intervals:
        raise CalibrationError("insufficient_good_intervals")
    visual_vectors = np.asarray([_log_rotation(pair[2]) for pair in pairs])
    excitation = np.linalg.svd(visual_vectors, compute_uv=False)
    if excitation[0] < 0.05 or excitation.size < 2 or excitation[1] < 0.01:
        raise CalibrationError("stationary_or_insufficient_excitation")
    if offsets is None:
        offsets = np.arange(-0.050, 0.0501, 0.005)
    offsets = [float(value) for value in offsets]
    if len(pairs) >= 2 * min_intervals:
        train, heldout = pairs[::2], pairs[1::2]
    else:
        split = max(3, len(pairs) // 2)
        train, heldout = pairs[:split], pairs[split:]
    candidates = []
    for offset in offsets:
        fit = _fit_at_offset(samples, train, offset, max_gap)
        if fit is None:
            continue
        mount, scale = fit
        all_residual = _residual(samples, pairs, mount, scale, offset, max_gap)
        if all_residual is None:
            continue
        train_error = _rmse_deg(_residual(samples, train, mount, scale, offset, max_gap))
        heldout_error = _rmse_deg(_residual(samples, heldout, mount, scale, offset, max_gap)) if heldout else None
        if train_error is not None:
            candidates.append((train_error, offset, mount, scale, heldout_error, _rmse_deg(all_residual)))
    if not candidates:
        raise CalibrationError("gyro_bounds_or_timestamp_gap")
    candidates.sort(key=lambda item: item[0])
    train_error, offset, mount, scale, heldout_error, all_error = candidates[0]
    second_error = candidates[1][0] if len(candidates) > 1 else None
    return {
        "rotation_camera_from_gyro": mount.tolist(),
        "scale": float(scale),
        "time_offset_s": float(offset),
        "quality": {
            "intervals": len(pairs),
            "train_intervals": len(train),
            "heldout_intervals": len(heldout),
            "excitation_singular_values": excitation.tolist(),
            "train_rmse_deg": train_error,
            "heldout_rmse_deg": heldout_error,
            "all_rmse_deg": all_error,
            "second_best_train_rmse_deg": second_error,
            "offset_ambiguity_gap_deg": None if second_error is None else second_error - train_error,
            "offset_grid_s": offsets,
        },
    }


def _value(data, *names):
    for name in names:
        if name in data:
            return data[name]
    return None


def _flag(data, *names):
    value = _value(data, *names)
    if value is None:
        return None
    return bool(np.asarray(value).reshape(-1)[0])


def _load_recording(folder, stride, limit):
    paths = sorted(glob.glob(str(Path(folder) / "*.npz")))
    if stride > 1:
        paths = paths[::stride]
    if limit:
        paths = paths[:limit]
    frames = []
    gyro_rows = []
    biases = []
    scales = []
    offsets = []
    ranges = []
    scale_flags = []
    time_offset_flags = []
    bias_flags = []
    for path in paths:
        with np.load(path, allow_pickle=False) as data:
            rgb, depth, camera_matrix, timestamp = (
                _value(data, "rgb"), _value(data, "depth"), _value(data, "K"), _value(data, "timestamp")
            )
            if rgb is None or depth is None or camera_matrix is None or timestamp is None:
                continue
            frames.append((float(timestamp), rgb, depth, np.asarray(camera_matrix, dtype=float)))
            sample_array = _value(data, "gyro_samples", "samples")
            if sample_array is not None:
                array = np.asarray(sample_array, dtype=float)
                if array.ndim == 2 and array.shape[1] == 4:
                    gyro_rows.extend(array.tolist())
            bias = _value(data, "gyro_bias_rad_s", "bias_rad_s")
            if bias is not None:
                value = np.asarray(bias, dtype=float).reshape(-1)
                if value.size == 3 and np.isfinite(value).all():
                    biases.append(value)
            scale = _value(data, "gyro_scale", "scale")
            if scale is not None:
                scales.append(float(np.asarray(scale).reshape(-1)[0]))
            time_offset = _value(data, "gyro_time_offset_s", "time_offset_s")
            if time_offset is not None:
                offsets.append(float(np.asarray(time_offset).reshape(-1)[0]))
            range_dps = _value(data, "gyro_range_dps", "range_dps")
            if range_dps is not None:
                ranges.append(int(np.asarray(range_dps).reshape(-1)[0]))
            for values, names in (
                (scale_flags, ("gyro_scale_applied", "scale_applied")),
                (time_offset_flags, ("gyro_time_offset_applied", "time_offset_applied")),
                (bias_flags, ("gyro_bias_subtracted", "bias_subtracted")),
            ):
                flag = _flag(data, *names)
                if flag is not None:
                    values.append(flag)
    if not frames:
        raise CalibrationError("no_RGBD_frames")
    frames.sort(key=lambda item: item[0])
    unique_rows = []
    for row in sorted(gyro_rows, key=lambda item: item[0]):
        if not unique_rows or row[0] - unique_rows[-1][0] > 1.0e-7:
            unique_rows.append(row)
    if not unique_rows:
        raise CalibrationError("no_gyro_samples")
    if not biases or not scales or not offsets:
        raise CalibrationError("missing_gyro_calibration_metadata")
    if scale_flags and not all(scale_flags):
        raise CalibrationError("unsupported_unscaled_gyro_samples")
    if time_offset_flags and not all(time_offset_flags):
        raise CalibrationError("unsupported_unshifted_gyro_timestamps")
    if bias_flags and any(bias_flags):
        raise CalibrationError("unsupported_bias_subtracted_gyro_samples")
    bias = biases[0]
    if any(np.linalg.norm(value - bias) > 1.0e-5 for value in biases[1:]):
        raise CalibrationError("inconsistent_gyro_bias_metadata")
    scale = scales[0]
    time_offset = offsets[0]
    if not np.isfinite(scale) or scale <= 0.0 or any(abs(value - scale) > 1.0e-6 for value in scales[1:]):
        raise CalibrationError("inconsistent_gyro_scale_metadata")
    if not np.isfinite(time_offset) or any(abs(value - time_offset) > 1.0e-6 for value in offsets[1:]):
        raise CalibrationError("inconsistent_gyro_time_offset_metadata")
    samples = np.asarray(unique_rows, dtype=float)
    samples[:, 0] -= time_offset
    samples[:, 1:] = (samples[:, 1:] - bias) / scale
    metadata = {
        "frames": len(frames),
        "gyro_samples": len(samples),
        "gyro_samples_deduplicated": len(gyro_rows) - len(samples),
        "recorded_scale_removed": scale,
        "recorded_time_offset_removed_s": time_offset,
        "bias_removed_rad_s": bias.tolist(),
        "scale_applied": True,
        "time_offset_applied": True,
        "bias_subtracted": False,
        "range_dps": Counter(ranges).most_common(1)[0][0] if ranges else None,
    }
    return frames, samples, metadata


def _visual_pairs(frames, min_inliers):
    odometry = RGBDOdometry(frames[0][3], method="pnp")
    good = []
    statuses = Counter()
    for index, (timestamp, rgb, depth, camera_matrix) in enumerate(frames):
        if not np.allclose(camera_matrix, odometry.K):
            odometry = RGBDOdometry(camera_matrix, method="pnp")
        try:
            result = odometry.update(rgb, depth, timestamp)
        except Exception:
            statuses["error"] += 1
            continue
        statuses[result["status"]] += 1
        if result["status"] == "tracking" and int(result.get("inliers", 0)) >= min_inliers:
            good.append((index, timestamp, np.asarray(result["pose"], dtype=float)))
    pairs = []
    for first, second in zip(good, good[1:]):
        if second[0] != first[0] + 1:
            continue
        relative = first[2][:3, :3].T @ second[2][:3, :3]
        pairs.append((first[1], second[1], relative))
    return pairs, statuses, len(good)


def run(args):
    cv2.setNumThreads(2)
    frames, samples, metadata = _load_recording(args.recording, args.stride, args.limit)
    pairs, statuses, good_frames = _visual_pairs(frames, args.min_inliers)
    metadata.update({
        "visual_good_frames": good_frames,
        "visual_status_counts": dict(statuses),
        "visual_pairs": len(pairs),
    })
    offsets = np.arange(-args.offset_range_ms, args.offset_range_ms + args.offset_step_ms * 0.5, args.offset_step_ms) / 1000.0
    fit = fit_calibration(samples, pairs, offsets=offsets, max_gap=args.max_gap_ms / 1000.0, min_intervals=args.min_intervals)
    candidate = {
        "enabled": False,
        "mounting_validated": False,
        "rotation_camera_from_gyro": fit["rotation_camera_from_gyro"],
        "scale": fit["scale"],
        "time_offset": fit["time_offset_s"],
    }
    if metadata["range_dps"] is not None:
        candidate["range_dps"] = metadata["range_dps"]
    quality = dict(metadata)
    quality.update(fit["quality"])
    quality["accepted"] = True
    return {"accepted": True, "candidate_config": candidate, "quality": quality}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--min-inliers", type=int, default=12)
    parser.add_argument("--min-intervals", type=int, default=6)
    parser.add_argument("--offset-range-ms", type=float, default=50.0)
    parser.add_argument("--offset-step-ms", type=float, default=5.0)
    parser.add_argument("--max-gap-ms", type=float, default=250.0)
    args = parser.parse_args()
    disabled = {"enabled": False, "mounting_validated": False}
    try:
        numeric = (args.offset_range_ms, args.offset_step_ms, args.max_gap_ms)
        if args.stride < 1 or args.limit < 0 or args.min_inliers < 3 or args.min_intervals < 3:
            raise CalibrationError("invalid_cli_limits")
        if not all(np.isfinite(value) and value > 0.0 for value in numeric):
            raise CalibrationError("invalid_cli_limits")
        result = run(args)
    except Exception as error:
        result = {
            "accepted": False,
            "candidate_config": disabled,
            "quality": {"accepted": False, "reason": str(error)},
        }
    print(json.dumps(result, allow_nan=False, separators=(",", ":")))
    return 0 if result["accepted"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
