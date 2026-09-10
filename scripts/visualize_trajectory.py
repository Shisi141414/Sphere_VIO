#!/usr/bin/env python3
"""Draw an estimated trajectory and groundtruth in an OpenCV window.

The window is useful with Xlaunch on Windows. The estimate is aligned to the
groundtruth with Sim(3) so the long-term metric-scale drift does not hide the
actual trajectory shape.
"""

import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import evaluate_sphere_vio as evaluator


def make_trajectory_image(gt_positions, estimated_positions, size=900):
    """Render XY, XZ, and YZ projections side by side on a dark canvas."""
    all_points = np.vstack([gt_positions, estimated_positions])
    center = all_points.mean(axis=0)
    radius = max(1e-6, float(np.max(np.abs(all_points - center), axis=0).max()))

    panels = []
    projections = [
        (0, 1, "XY (top)"),
        (0, 2, "XZ (front)"),
        (1, 2, "YZ (side)"),
    ]
    for axis_x, axis_y, title in projections:
        panel = np.full((size, size, 3), 30, dtype=np.uint8)
        margin = 30
        scale = (size - 2 * margin) / (2.0 * radius)
        origin_x = size / 2.0
        origin_y = size / 2.0

        def point_to_pixel(point):
            x = (point[axis_x] - center[axis_x]) * scale + origin_x
            y = origin_y - (point[axis_y] - center[axis_y]) * scale
            return int(round(x)), int(round(y))

        for point in gt_positions:
            x, y = point_to_pixel(point)
            cv2.circle(panel, (x, y), 1, (0, 180, 0), -1)
        for index in range(1, len(gt_positions)):
            cv2.line(panel, point_to_pixel(gt_positions[index - 1]),
                     point_to_pixel(gt_positions[index]), (0, 120, 0), 1)

        for point in estimated_positions:
            x, y = point_to_pixel(point)
            cv2.circle(panel, (x, y), 1, (0, 0, 255), -1)
        for index in range(1, len(estimated_positions)):
            cv2.line(panel, point_to_pixel(estimated_positions[index - 1]),
                     point_to_pixel(estimated_positions[index]), (100, 80, 255), 1)

        if len(gt_positions):
            start = point_to_pixel(gt_positions[0])
            cv2.circle(panel, start, 5, (255, 255, 255), -1)
        if len(estimated_positions):
            start = point_to_pixel(estimated_positions[0])
            cv2.circle(panel, start, 5, (0, 255, 255), -1)

        cv2.putText(panel, title, (12, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(panel, "GT", (12, size - 12), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 180, 0), 1, cv2.LINE_AA)
        cv2.putText(panel, "EST", (70, size - 12), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (100, 80, 255), 1, cv2.LINE_AA)
        panels.append(panel)

    return np.hstack(panels)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--groundtruth", required=True)
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--output-image", default=None)
    parser.add_argument("--size", type=int, default=520)
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    gt_t, gt_p, gt_q = evaluator.load_tum(args.groundtruth)
    est_t, est_p, est_q = evaluator.load_tum(args.trajectory)

    finite = (
        np.isfinite(est_t)
        & np.isfinite(est_p).all(axis=1)
        & np.isfinite(est_q).all(axis=1)
    )
    valid = finite & (est_t > 0.0)
    est_t = est_t[valid]
    est_p = est_p[valid]
    est_q = est_q[valid]

    interpolated_p, _ = evaluator.interpolate_groundtruth(
        gt_t, gt_p, gt_q, est_t
    )
    in_range = np.isfinite(interpolated_p).all(axis=1)
    est_p = est_p[in_range]
    gt_matched = interpolated_p[in_range]
    est_t_matched = est_t[in_range]

    if len(gt_matched) < 3:
        print(
            "Error: trajectory and groundtruth timestamps do not overlap.",
            file=sys.stderr,
        )
        print(
            "Groundtruth range: {:.6f} .. {:.6f}".format(
                gt_t[0], gt_t[-1]
            ),
            file=sys.stderr,
        )
        if len(est_t_matched):
            print(
                "Trajectory range: {:.6f} .. {:.6f}".format(
                    est_t_matched[0], est_t_matched[-1]
                ),
                file=sys.stderr,
            )
        return 2

    rotation, translation, scale = evaluator.align_umeyama(
        est_p, gt_matched, with_scale=True
    )
    aligned = scale * est_p @ rotation.T + translation

    canvas = make_trajectory_image(gt_matched, aligned, size=args.size)
    if not args.headless:
        cv2.namedWindow("Sphere-VIO trajectory", cv2.WINDOW_NORMAL)
        cv2.imshow("Sphere-VIO trajectory", canvas)
    if args.output_image:
        cv2.imwrite(args.output_image, canvas)
        print("Wrote {}".format(args.output_image))
    if args.headless:
        return
    print("Press Q or Esc in the OpenCV window to exit.")
    while True:
        key = cv2.waitKey(100) & 0xFF
        if key in (ord("q"), ord("Q"), 27):
            break


if __name__ == "__main__":
    main()
