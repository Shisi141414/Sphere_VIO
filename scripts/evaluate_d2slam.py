#!/usr/bin/env python3
"""Compute simple trajectory metrics for a D2SLAM groundtruth file and a
Sphere-VIO odometry.csv output.

The groundtruth format is:
  timestamp x y z qx qy qz qw

The odometry CSV format is:
  timestamp x y z qw qx qy qz ...
"""

import argparse
import csv

import numpy as np


def load_groundtruth(path):
    timestamps = []
    positions = []
    for line in open(path, "r", encoding="utf-8"):
        values = line.split()
        if len(values) < 8:
            continue
        timestamps.append(float(values[0]))
        positions.append([float(values[1]), float(values[2]), float(values[3])])
    return np.asarray(timestamps), np.asarray(positions)


def load_odometry(path):
    timestamps = []
    positions = []
    with open(path, "r", encoding="utf-8") as file:
        reader = csv.reader(file)
        next(reader, None)
        for row in reader:
            if len(row) < 8:
                continue
            timestamp = float(row[0])
            if timestamp <= 0.0:
                continue
            timestamps.append(timestamp)
            positions.append([float(row[1]), float(row[2]), float(row[3])])
    return np.asarray(timestamps), np.asarray(positions)


def synchronize(gt_time, gt_pos, est_time, est_pos, tolerance=0.05):
    gt_times = []
    gt_matches = []
    est_times = []
    est_matches = []
    j = 0
    for i, stamp in enumerate(est_time):
        while j < len(gt_time) and gt_time[j] < stamp - tolerance:
            j += 1
        if j >= len(gt_time):
            break
        if abs(gt_time[j] - stamp) <= tolerance:
            gt_times.append(gt_time[j])
            gt_matches.append(gt_pos[j])
            est_times.append(stamp)
            est_matches.append(est_pos[i])
    return (np.asarray(gt_times), np.asarray(gt_matches),
            np.asarray(est_times), np.asarray(est_matches))


def align_umeyama(source, target):
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    source_centered = source - source_center
    target_centered = target - target_center
    covariance = source_centered.T @ target_centered
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    return rotation, translation


def relative_error(gt_pos, est_pos, delta_time, timestamps):
    errors = []
    j = 0
    for i in range(len(timestamps)):
        target = timestamps[i] + delta_time
        while j < len(timestamps) and timestamps[j] < target:
            j += 1
        if j >= len(timestamps):
            break
        if abs(timestamps[j] - target) <= 0.2 * delta_time:
            gt_delta = gt_pos[j] - gt_pos[i]
            est_delta = est_pos[j] - est_pos[i]
            errors.append(np.linalg.norm(est_delta - gt_delta))
    return np.asarray(errors)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--groundtruth", required=True)
    parser.add_argument("--odometry", required=True)
    parser.add_argument("--relative-delta", type=float, default=1.0)
    return parser.parse_args()


def main():
    args = parse_args()
    gt_time, gt_pos = load_groundtruth(args.groundtruth)
    est_time, est_pos = load_odometry(args.odometry)
    _, gt_pos, matched_times, est_pos = synchronize(
        gt_time, gt_pos, est_time, est_pos
    )
    if len(gt_pos) < 3:
        print("Too few matched poses for evaluation")
        return

    rotation, translation = align_umeyama(est_pos, gt_pos)
    aligned = est_pos @ rotation.T + translation
    errors = np.linalg.norm(aligned - gt_pos, axis=1)
    rel_errors = relative_error(
        gt_pos, aligned, args.relative_delta, matched_times
    )

    print("matched poses:", len(gt_pos))
    print("ATE translation RMSE (m):", np.sqrt(np.mean(errors ** 2)))
    print("ATE translation mean (m):", np.mean(errors))
    print("ATE translation median (m):", np.median(errors))
    print("ATE translation max (m):", np.max(errors))
    if len(rel_errors):
        print(
            "relative translation RMSE (m, dt={:.2f}s):".format(
                args.relative_delta
            ),
            np.sqrt(np.mean(rel_errors ** 2)),
        )


if __name__ == "__main__":
    main()
