#!/usr/bin/env python3
"""Convert a D2SLAM full quad-image rosbag into Sphere-VIO topic layout.

D2SLAM full `-sync.bag` files store four 1280x800 camera frames side by side in
one `/arducam/image/compressed` message (a 5120x800 JPEG).  Sphere-VIO instead
expects four independent mono8 image topics:

  /fisheye/left/image_raw
  /fisheye/right/image_raw
  /fisheye/bleft/image_raw
  /fisheye/bright/image_raw

This script streams the input bag, splits the stitched image, and writes a new
bag with those four topics plus `/imu_data_raw`.  It does NOT recover camera
intrinsics/extrinsics; those still need to come from the D2SLAM calibration
config and be converted into `config/cameras.yaml`.
"""

import argparse

import cv2
import numpy as np
import rosbag
import sensor_msgs.msg
import std_msgs.msg


CAMERA_TOPICS = [
    "/fisheye/left/image_raw",
    "/fisheye/right/image_raw",
    "/fisheye/bleft/image_raw",
    "/fisheye/bright/image_raw",
]


def make_image_message(stamp, frame_id, mono_image):
    """Create a raw mono8 sensor_msgs/Image message."""
    message = sensor_msgs.msg.Image()
    message.header = std_msgs.msg.Header()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height = mono_image.shape[0]
    message.width = mono_image.shape[1]
    message.encoding = "mono8"
    message.is_bigendian = 0
    message.step = message.width
    message.data = mono_image.tobytes()
    return message


def convert_bag(input_path, output_path, image_topic, imu_topic,
                start_offset=0.0, max_frames=None):
    """Read a full D2SLAM bag and write a four-topic Sphere-VIO bag."""
    image_count = 0
    bag_start = None
    output_end = None

    with rosbag.Bag(input_path, "r") as input_bag:
        with rosbag.Bag(output_path, "w") as output_bag:
            for topic, message, stamp in input_bag.read_messages():
                stamp_sec = stamp.to_sec()
                if bag_start is None:
                    bag_start = stamp_sec
                if output_end is not None and stamp_sec > output_end:
                    break
                if stamp_sec < bag_start + start_offset:
                    continue

                if topic == image_topic:
                    if max_frames is not None and image_count >= max_frames:
                        if output_end is None:
                            output_end = stamp_sec + 1e-6
                        continue
                    # D2SLAM uses a JPEG-compressed stitched image. cv2 decodes
                    # it as BGR even though the four sub-images are grayscale.
                    packed = np.frombuffer(message.data, dtype=np.uint8)
                    decoded = cv2.imdecode(packed, cv2.IMREAD_GRAYSCALE)
                    if decoded is None or decoded.shape[1] % 4 != 0:
                        continue

                    camera_width = decoded.shape[1] // 4
                    for camera_id in range(4):
                        sub_image = decoded[
                            :, camera_id * camera_width:(camera_id + 1) *
                            camera_width]
                        output_bag.write(
                            CAMERA_TOPICS[camera_id],
                            make_image_message(
                                message.header.stamp,
                                "camera{}".format(camera_id),
                                sub_image,
                            ),
                            stamp,
                        )
                    image_count += 1
                    if max_frames is not None and image_count >= max_frames:
                        output_end = stamp_sec + 1e-6
                elif topic == imu_topic:
                    output_bag.write("/imu_data_raw", message, stamp)
                else:
                    output_bag.write(topic, message, stamp)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert D2SLAM full quad rosbag to Sphere-VIO topics."
    )
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--start-offset", type=float, default=0.0)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument(
        "--image-topic",
        default="/arducam/image/compressed",
    )
    parser.add_argument(
        "--imu-topic",
        default="/dji_sdk_1/dji_sdk/imu",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    convert_bag(
        args.input,
        args.output,
        args.image_topic,
        args.imu_topic,
        args.start_offset,
        args.max_frames,
    )
    print("Wrote {}".format(args.output))


if __name__ == "__main__":
    main()
