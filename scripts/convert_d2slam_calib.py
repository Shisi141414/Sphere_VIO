#!/usr/bin/env python3
"""Convert a D2SLAM Kalibr-style camchain into Sphere-VIO cameras.yaml.

The D2SLAM repository stores one omni+radtan model per camera under keys cam0
through cam3.  Sphere-VIO expects a list ordered by its CameraId with the same
model fields and a cyclic T_cn_cnm1 entry for every camera.  This script also
maps D2SLAM cam0..cam3 to the project names left/right/bleft/bright used by the
offline converter and config/offline.yaml.
"""

import argparse

import numpy as np
import yaml


PROJECT_NAMES = ["left", "right", "bleft", "bright"]
RAW_TOPICS = [
    "/fisheye/left/image_raw",
    "/fisheye/right/image_raw",
    "/fisheye/bleft/image_raw",
    "/fisheye/bright/image_raw",
]


def parse_matrix(node):
    return np.asarray(node, dtype=np.float64).reshape(4, 4)


def matrix_as_list(matrix):
    return [float(value) for value in matrix.flatten()]


def convert(input_path, output_path):
    with open(input_path, "r", encoding="utf-8") as file:
        source = yaml.safe_load(file)

    transforms = {}
    for camera_index in range(4):
        camera = source["cam{}".format(camera_index)]
        # D2SLAM stores p_imu = T_cam_imu * p_cam, while Sphere-VIO's loader
        # expects p_cam = T_cam_imu * p_imu. Invert each raw entry once here.
        transforms[camera_index] = np.linalg.inv(parse_matrix(camera["T_cam_imu"]))

    cameras = []
    for camera_index in range(4):
        camera = source["cam{}".format(camera_index)]
        previous = (camera_index + 3) % 4
        # The loader validates T_cn_cnm1 against T_cam_imu. Reconstruct it
        # analytically so the chain is internally consistent even when the
        # source omits one entry (D2SLAM does not provide cam0 in this file).
        t_cn_cnm1 = transforms[camera_index] @ np.linalg.inv(transforms[previous])
        cameras.append(
            {
                "camera_id": camera_index,
                "name": PROJECT_NAMES[camera_index],
                "kalibr_camera_id": camera_index,
                "topic_raw": RAW_TOPICS[camera_index],
                "topic_compressed": camera["rostopic"],
                "resolution": camera["resolution"],
                "model": "omni",
                "intrinsics": camera["intrinsics"],
                "distortion_model": "radtan",
                "distortion_coeffs": camera["distortion_coeffs"],
                "T_cam_imu": {
                    "rows": 4,
                    "cols": 4,
                    "data": matrix_as_list(transforms[camera_index]),
                },
                "T_cn_cnm1": {
                    "rows": 4,
                    "cols": 4,
                    "data": matrix_as_list(t_cn_cnm1),
                },
                "timeshift_cam_imu": None,
                "timeshift_available": False,
            }
        )

    with open(output_path, "w", encoding="utf-8") as file:
        file.write(
            "# Converted from D2SLAM 7-inch-n3 quadcam calibration.\n"
            "# Source: HKUST-Aerial-Robotics/D2SLAM "
            "config/quadcam/quad_cam_calib-camchain-imucam-7-inch-n3.yaml\n"
        )
        yaml.safe_dump({"cameras": cameras}, file, sort_keys=False)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert D2SLAM quadcam calibration to Sphere-VIO cameras.yaml."
    )
    parser.add_argument(
        "--input",
        required=True,
        help="Path to D2SLAM quad_cam_calib-camchain-imucam-7-inch-n3.yaml",
    )
    parser.add_argument(
        "--output",
        default="config/cameras_d2slam.yaml",
        help="Output cameras.yaml",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    convert(args.input, args.output)
    print("Wrote {}".format(args.output))


if __name__ == "__main__":
    main()
