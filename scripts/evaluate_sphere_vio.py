#!/usr/bin/env python3
"""Evaluate Sphere-VIO trajectories against D2SLAM groundtruth.

The groundtruth and estimated trajectory files are both TUM-style text files:
  timestamp x y z qx qy qz qw

Groundtruth poses are interpolated to every estimated timestamp. Position uses
linear interpolation and orientation uses shortest-path quaternion slerp.
"""

import argparse
import math

import numpy as np


def _is_number(text):
    try:
        float(text)
        return True
    except ValueError:
        return False


def load_tum(path):
    """Load a TUM trajectory as timestamp, position, and xyzw quaternion arrays."""
    timestamps = []
    positions = []
    quaternions = []
    with open(path, "r", encoding="utf-8") as file:
        for line in file:
            values = line.replace(",", " ").strip().split()
            if not values or not _is_number(values[0]):
                continue
            values = values[:8]
            if len(values) < 8:
                continue
            numbers = [float(value) for value in values]
            timestamps.append(numbers[0])
            positions.append(numbers[1:4])
            quaternions.append(numbers[4:8])

    if not timestamps:
        raise ValueError("No poses found in {}".format(path))

    timestamps = np.asarray(timestamps, dtype=np.float64)
    positions = np.asarray(positions, dtype=np.float64)
    quaternions = np.asarray(quaternions, dtype=np.float64)
    norms = np.linalg.norm(quaternions, axis=1, keepdims=True)
    quaternions = quaternions / norms
    order = np.argsort(timestamps)
    return timestamps[order], positions[order], quaternions[order]


def slerp(q0, q1, amount):
    """Spherical interpolation between two xyzw quaternions."""
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        result = q0 + amount * (q1 - q0)
        return result / np.linalg.norm(result)
    theta_0 = math.acos(dot)
    sin_theta = math.sin(theta_0)
    weight_0 = math.sin((1.0 - amount) * theta_0) / sin_theta
    weight_1 = math.sin(amount * theta_0) / sin_theta
    return weight_0 * q0 + weight_1 * q1


def interpolate_groundtruth(gt_t, gt_p, gt_q, query_t):
    """Interpolate groundtruth poses at the supplied query timestamps."""
    query_t = np.asarray(query_t, dtype=np.float64)
    if len(gt_t) < 2:
        raise ValueError("Groundtruth has fewer than two poses")

    positions = np.empty((len(query_t), 3), dtype=np.float64)
    for axis in range(3):
        positions[:, axis] = np.interp(
            query_t, gt_t, gt_p[:, axis], left=np.nan, right=np.nan
        )

    quaternions = np.empty((len(query_t), 4), dtype=np.float64)
    indices = np.searchsorted(gt_t, query_t, side="left")
    for row, stamp in enumerate(query_t):
        index = indices[row]
        if index == 0 or index >= len(gt_t):
            if index == 0:
                quaternions[row] = gt_q[0]
            else:
                quaternions[row] = gt_q[-1]
            continue
        low = index - 1
        high = index
        span = gt_t[high] - gt_t[low]
        amount = 0.0 if span <= 0.0 else (stamp - gt_t[low]) / span
        quaternions[row] = slerp(gt_q[low], gt_q[high], amount)
    return positions, quaternions


def align_umeyama(source, target, with_scale):
    """Align source poses to target using Umeyama with optional scale."""
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    source_centered = source - source_center
    target_centered = target - target_center
    covariance = source_centered.T @ target_centered
    u, singular, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T

    scale = 1.0
    if with_scale:
        scale = float(np.sum(singular) / np.sum(source_centered**2))
    translation = target_center - scale * rotation @ source_center
    return rotation, translation, scale


def quaternion_angle(q0, q1):
    """Return the angle between two xyzw quaternions in radians."""
    dot = abs(float(np.dot(q0, q1)))
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


def transform_poses(rotation, translation, scale, positions, quaternions):
    """Apply an Umeyama alignment to positions and orientations."""
    transformed_positions = scale * positions @ rotation.T + translation
    rotation_matrix = rotation
    transformed_quaternions = []
    for quaternion in quaternions:
        # Convert xyzw to a 3x3 rotation matrix, then apply the alignment.
        x, y, z, w = quaternion
        matrix = np.asarray(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
                 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
                 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w),
                 1 - 2 * (x * x + y * y)],
            ]
        )
        aligned_matrix = rotation_matrix @ matrix
        # Recover a unit quaternion from the aligned rotation matrix.
        trace = float(np.trace(aligned_matrix))
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            w2 = 0.25 * s
            x2 = (aligned_matrix[2, 1] - aligned_matrix[1, 2]) / s
            y2 = (aligned_matrix[0, 2] - aligned_matrix[2, 0]) / s
            z2 = (aligned_matrix[1, 0] - aligned_matrix[0, 1]) / s
        elif aligned_matrix[0, 0] > aligned_matrix[1, 1] and aligned_matrix[0, 0] > aligned_matrix[2, 2]:
            s = math.sqrt(max(0.0, 1.0 + aligned_matrix[0, 0] - aligned_matrix[1, 1] - aligned_matrix[2, 2])) * 2.0
            w2 = (aligned_matrix[2, 1] - aligned_matrix[1, 2]) / s
            x2 = 0.25 * s
            y2 = (aligned_matrix[0, 1] + aligned_matrix[1, 0]) / s
            z2 = (aligned_matrix[0, 2] + aligned_matrix[2, 0]) / s
        elif aligned_matrix[1, 1] > aligned_matrix[2, 2]:
            s = math.sqrt(max(0.0, 1.0 + aligned_matrix[1, 1] - aligned_matrix[0, 0] - aligned_matrix[2, 2])) * 2.0
            w2 = (aligned_matrix[0, 2] - aligned_matrix[2, 0]) / s
            x2 = (aligned_matrix[0, 1] + aligned_matrix[1, 0]) / s
            y2 = 0.25 * s
            z2 = (aligned_matrix[1, 2] + aligned_matrix[2, 1]) / s
        else:
            s = math.sqrt(max(0.0, 1.0 + aligned_matrix[2, 2] - aligned_matrix[0, 0] - aligned_matrix[1, 1])) * 2.0
            w2 = (aligned_matrix[1, 0] - aligned_matrix[0, 1]) / s
            x2 = (aligned_matrix[0, 2] + aligned_matrix[2, 0]) / s
            y2 = (aligned_matrix[1, 2] + aligned_matrix[2, 1]) / s
            z2 = 0.25 * s
        q = np.asarray([x2, y2, z2, w2], dtype=np.float64)
        transformed_quaternions.append(q / np.linalg.norm(q))
    return transformed_positions, np.asarray(transformed_quaternions)


def relative_pose_error(positions_est, quaternions_est,
                        positions_gt, quaternions_gt,
                        target_distance, timestamps):
    """Compute relative pose errors for pairs separated by target distance."""
    translation_errors = []
    rotation_errors = []
    n = len(timestamps)
    for i in range(n - 1):
        j = i + 1
        while j < n and np.linalg.norm(positions_est[j] - positions_est[i]) < target_distance:
            j += 1
        if j >= n:
            break

        # Relative estimated transformation.
        t_est_i, q_est_i = positions_est[i], quaternions_est[i]
        t_est_j, q_est_j = positions_est[j], quaternions_est[j]
        # Convert xyzw to matrix and invert pose i.
        x, y, z, w = q_est_i
        r_est_i = np.asarray([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        x, y, z, w = q_est_j
        r_est_j = np.asarray([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        r_rel_est = r_est_i.T @ r_est_j
        t_rel_est = r_est_i.T @ (t_est_j - t_est_i)

        x, y, z, w = quaternions_gt[i]
        r_gt_i = np.asarray([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        x, y, z, w = quaternions_gt[j]
        r_gt_j = np.asarray([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        r_rel_gt = r_gt_i.T @ r_gt_j
        t_rel_gt = r_gt_i.T @ (positions_gt[j] - positions_gt[i])

        error_rotation = r_rel_est.T @ r_rel_gt
        angle = math.acos(min(1.0, max(-1.0, (np.trace(error_rotation) - 1.0) / 2.0)))
        translation_errors.append(np.linalg.norm(t_rel_est - t_rel_gt))
        rotation_errors.append(math.degrees(angle))
    return np.asarray(translation_errors), np.asarray(rotation_errors)


def relative_time_error(positions_est, quaternions_est,
                        positions_gt, quaternions_gt,
                        delta_time, timestamps):
    """Compute relative pose errors for pairs separated by a time delta."""
    translation_errors = []
    rotation_errors = []
    n = len(timestamps)

    def quaternion_to_matrix(q):
        x, y, z, w = q
        return np.asarray([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

    for i in range(n - 1):
        j = i + 1
        while j < n and timestamps[j] - timestamps[i] < delta_time:
            j += 1
        if j >= n:
            break

        r_est_i = quaternion_to_matrix(quaternions_est[i])
        r_est_j = quaternion_to_matrix(quaternions_est[j])
        r_rel_est = r_est_i.T @ r_est_j
        t_rel_est = r_est_i.T @ (positions_est[j] - positions_est[i])

        r_gt_i = quaternion_to_matrix(quaternions_gt[i])
        r_gt_j = quaternion_to_matrix(quaternions_gt[j])
        r_rel_gt = r_gt_i.T @ r_gt_j
        t_rel_gt = r_gt_i.T @ (positions_gt[j] - positions_gt[i])

        error_rotation = r_rel_est.T @ r_rel_gt
        angle = math.acos(min(1.0, max(
            -1.0, (np.trace(error_rotation) - 1.0) / 2.0
        )))
        translation_errors.append(np.linalg.norm(t_rel_est - t_rel_gt))
        rotation_errors.append(math.degrees(angle))
    return np.asarray(translation_errors), np.asarray(rotation_errors)


def rms(values):
    if len(values) == 0:
        return float("nan")
    return float(np.sqrt(np.mean(np.square(values))))


def report_error(name, translation_errors, rotation_errors, report_lines):
    report_lines.append("- {} translation RMSE: {:.6f} m".format(
        name, rms(translation_errors)))
    report_lines.append("- {} translation mean: {:.6f} m".format(
        name, float(np.mean(translation_errors)) if len(translation_errors) else float("nan")))
    report_lines.append("- {} rotation RMSE: {:.6f} deg".format(
        name, rms(rotation_errors)))


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--groundtruth", required=True)
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--relative-distances", default="1,5,10")
    parser.add_argument("--relative-time-deltas", default="1,5,10")
    parser.add_argument("--output-period", type=float, default=0.05)
    parser.add_argument("--output-report", default=None)
    return parser.parse_args()


def main():
    args = parse_args()
    gt_t, gt_p, gt_q = load_tum(args.groundtruth)
    est_t, est_p, est_q = load_tum(args.trajectory)

    relative_distances = [float(value) for value in args.relative_distances.split(",")]
    relative_time_deltas = [float(value) for value in args.relative_time_deltas.split(",")]
    report_lines = []

    finite_pose = (
        np.isfinite(est_t)
        & np.isfinite(est_p).all(axis=1)
        & np.isfinite(est_q).all(axis=1)
    )
    has_nan_inf = not bool(np.all(finite_pose))
    valid = finite_pose & (est_t > 0.0)
    valid_t = est_t[valid]
    valid_p = est_p[valid]
    valid_q = est_q[valid]

    if len(valid_t) == 0:
        report_lines.append("success: False")
        report_lines.append("coverage: 0.000000")
        report_lines.append("has_nan_inf: True")
        report_lines.append("failure_time: START")
        text = "\n".join(report_lines)
        print(text)
        if args.output_report:
            with open(args.output_report, "w", encoding="utf-8") as file:
                file.write(text + "\n")
        return

    interpolated_p, interpolated_q = interpolate_groundtruth(
        gt_t, gt_p, gt_q, valid_t
    )
    in_gt_range = np.isfinite(interpolated_p).all(axis=1)
    matched_t = valid_t[in_gt_range]
    est_matched_p = valid_p[in_gt_range]
    est_matched_q = valid_q[in_gt_range]
    gt_matched_p = interpolated_p[in_gt_range]
    gt_matched_q = interpolated_q[in_gt_range]

    if len(matched_t) < 3:
        report_lines.append("matched poses: {}".format(len(matched_t)))
        report_lines.append("success: False")
        report_lines.append("coverage: 0.000000")
        text = "\n".join(report_lines)
        print(text)
        if args.output_report:
            with open(args.output_report, "w", encoding="utf-8") as file:
                file.write(text + "\n")
        return

    rotation, translation, scale = align_umeyama(
        est_matched_p, gt_matched_p, with_scale=False
    )
    aligned_p_se3, aligned_q_se3 = transform_poses(
        rotation, translation, 1.0, est_matched_p, est_matched_q
    )
    ates = np.linalg.norm(aligned_p_se3 - gt_matched_p, axis=1)
    ates_r = np.asarray([
        quaternion_angle(aligned_q_se3[i], gt_matched_q[i])
        for i in range(len(matched_t))
    ])

    rotation_sim3, translation_sim3, scale_sim3 = align_umeyama(
        est_matched_p, gt_matched_p, with_scale=True
    )
    aligned_p_sim3, aligned_q_sim3 = transform_poses(
        rotation_sim3, translation_sim3, scale_sim3, est_matched_p, est_matched_q
    )
    ates_sim3 = np.linalg.norm(aligned_p_sim3 - gt_matched_p, axis=1)
    ates_r_sim3 = np.asarray([
        quaternion_angle(aligned_q_sim3[i], gt_matched_q[i])
        for i in range(len(matched_t))
    ])

    intervals = np.diff(matched_t)
    max_pose_interval = float(np.max(intervals)) if len(intervals) else float("nan")
    gt_span = float(gt_t[-1] - gt_t[0])
    est_span = float(matched_t[-1] - matched_t[0]) if len(matched_t) else 0.0
    coverage = est_span / gt_span if gt_span > 0.0 else 0.0
    interval_threshold = 2.0 * args.output_period
    success_reasons = []
    if has_nan_inf:
        success_reasons.append("has_nan_inf=True")
    if coverage < 0.9:
        success_reasons.append("coverage={:.6f}<0.90".format(coverage))
    if not np.isfinite(max_pose_interval) or \
            max_pose_interval > interval_threshold:
        success_reasons.append(
            "max_pose_interval={:.6f}>{:.6f}".format(
                max_pose_interval, interval_threshold
            )
        )
    success = not success_reasons

    report_lines.append("matched poses: {}".format(len(matched_t)))
    report_lines.append("ATE_SE3 translation RMSE: {:.6f} m".format(rms(ates)))
    report_lines.append("ATE_SE3 translation mean: {:.6f} m".format(float(np.mean(ates))))
    report_lines.append("ATE_SE3 rotation RMSE: {:.6f} rad".format(rms(ates_r)))
    report_lines.append("ATE_Sim3 translation RMSE: {:.6f} m".format(rms(ates_sim3)))
    report_lines.append("ATE_Sim3 rotation RMSE: {:.6f} rad".format(rms(ates_r_sim3)))
    report_lines.append("ATE_Sim3 scale: {:.6f}".format(scale_sim3))

    for distance in relative_distances:
        translation_errors, rotation_errors = relative_pose_error(
            aligned_p_se3, aligned_q_se3, gt_matched_p, gt_matched_q,
            distance, matched_t
        )
        report_lines.append("RPE_{:g}m translation RMSE: {:.6f} m".format(
            distance, rms(translation_errors)))
        report_lines.append("RPE_{:g}m rotation RMSE: {:.6f} deg".format(
            distance, rms(rotation_errors)))
    rpe_r_rotations = []
    for distance in relative_distances:
        _, rotation_errors = relative_pose_error(
            aligned_p_se3, aligned_q_se3, gt_matched_p, gt_matched_q,
            distance, matched_t
        )
        rpe_r_rotations.extend(rotation_errors.tolist())
    report_lines.append("RPE_R translation RMSE: N/A")
    report_lines.append("RPE_R rotation RMSE: {:.6f} deg".format(
        rms(np.asarray(rpe_r_rotations))))

    for delta in relative_time_deltas:
        translation_errors, rotation_errors = relative_time_error(
            aligned_p_se3, aligned_q_se3, gt_matched_p, gt_matched_q,
            delta, matched_t
        )
        report_lines.append("RTE_{:g}s translation RMSE: {:.6f} m".format(
            delta, rms(translation_errors)))
        report_lines.append("RTE_{:g}s rotation RMSE: {:.6f} deg".format(
            delta, rms(rotation_errors)))

    report_lines.append("coverage: {:.6f}".format(coverage))
    report_lines.append("max_pose_interval: {:.6f} s".format(max_pose_interval))
    report_lines.append("has_nan_inf: {}".format(has_nan_inf))
    report_lines.append("success: {}".format(success))
    report_lines.append("success_reasons: {}".format(
        "none" if not success_reasons else "; ".join(success_reasons)
    ))
    report_lines.append(
        "success_checks: has_nan_inf={}, coverage={:.6f}, "
        "max_pose_interval={:.6f} (threshold={:.6f})".format(
            has_nan_inf, coverage, max_pose_interval, interval_threshold
        )
    )
    report_lines.append("failure_time: {}".format(
        "N/A" if np.all(finite_pose) else "FAIL@{:.9f}".format(
            float(est_t[~finite_pose][0]))
    ))

    text = "\n".join(report_lines)
    print(text)
    if args.output_report:
        with open(args.output_report, "w", encoding="utf-8") as file:
            file.write(text + "\n")


if __name__ == "__main__":
    main()
