#!/usr/bin/env python3
"""Generate a small deterministic ROS1 bag for Sphere-VIO smoke tests.

This is not a replacement for the real four-fisheye drone dataset.  It writes
four synchronized mono8 image topics and one IMU topic using the same names as
config/offline.yaml, so the three offline runners and the visualizer can be
exercised end to end without real sensor data.
"""

import argparse
import math

import cv2
import numpy as np
import rospy
import rosbag
import sensor_msgs.msg
import std_msgs.msg


CAMERA_TOPICS = [
    "/fisheye/left/image_raw",
    "/fisheye/right/image_raw",
    "/fisheye/bleft/image_raw",
    "/fisheye/bright/image_raw",
]
IMU_TOPIC = "/imu_data_raw"


def make_image(camera_id, frame_index, width, height):
    """Create a high-contrast synthetic fisheye-like image.

    The scene is deliberately texture-rich so FAST and LK have something to
    track.  Each camera has a different dominant gradient direction and each
    frame translates the checkerboard by a few pixels, producing a small but
    consistent apparent motion.
    """
    xx, yy = np.meshgrid(np.arange(width, dtype=np.float32),
                         np.arange(height, dtype=np.float32))
    shift = frame_index * 4.0
    if camera_id == 0:
        pattern = (xx + shift) * 0.6 + yy * 0.1
    elif camera_id == 1:
        pattern = -xx * 0.4 + (yy + shift) * 0.5
    elif camera_id == 2:
        pattern = (xx + shift) * 0.35 - yy * 0.35
    else:
        pattern = -(xx + shift) * 0.45 - (yy + shift) * 0.3

    checker = ((pattern / 24.0).astype(np.int32) % 2).astype(np.uint8) * 255
    image = np.clip(checker.astype(np.float32) +
                    (pattern.astype(np.float32) % 32.0), 0, 255).astype(np.uint8)

    # Draw a few stable dark/light blobs with motion; this gives LK well-defined
    # corners while keeping the bag tiny compared with a real fisheye stream.
    for i in range(12):
        cx = int((width * 0.12 + i * width * 0.075 + shift * 1.5) % width)
        cy = int((height * 0.15 + i * height * 0.07) % height)
        radius = 18 + (i % 4) * 5
        color = 20 if i % 2 == 0 else 235
        cv2.circle(image, (cx, cy), radius, color, -1, cv2.LINE_AA)
    return image


def make_image_message(stamp_sec, camera_id, frame_index, width, height):
    """Wrap a generated mono8 image in sensor_msgs/Image."""
    image = make_image(camera_id, frame_index, width, height)
    message = sensor_msgs.msg.Image()
    message.header = std_msgs.msg.Header()
    message.header.stamp = rospy.Time.from_sec(stamp_sec)
    message.header.frame_id = "camera{}".format(camera_id)
    message.height = height
    message.width = width
    message.encoding = "mono8"
    message.is_bigendian = 0
    message.step = width
    message.data = image.tobytes()
    return message


def make_imu_message(stamp_sec, sample_index):
    """Create a slowly varying IMU sample at the requested timestamp."""
    message = sensor_msgs.msg.Imu()
    message.header = std_msgs.msg.Header()
    message.header.stamp = rospy.Time.from_sec(stamp_sec)
    message.header.frame_id = "imu"

    phase = sample_index * 0.02
    message.linear_acceleration.x = 0.10 * math.sin(phase)
    message.linear_acceleration.y = 0.08 * math.cos(phase * 0.7)
    message.linear_acceleration.z = 9.81 + 0.05 * math.sin(phase * 0.3)
    message.angular_velocity.x = 0.02 * math.sin(phase * 0.5)
    message.angular_velocity.y = 0.03 * math.cos(phase * 0.8)
    message.angular_velocity.z = 0.01 * math.sin(phase * 1.1)

    # The covariance values are deliberately zero; the current offline runners
    # only inspect timestamps and raw vector values.
    message.orientation_covariance[0] = -1.0
    message.linear_acceleration_covariance[0] = -1.0
    message.angular_velocity_covariance[0] = -1.0
    return message


def build_message_list(args):
    """Build all image and IMU messages and sort them by timestamp."""
    messages = []
    image_count = int(round(args.duration * args.image_rate))

    for frame_index in range(image_count):
        image_stamp = args.start_time + frame_index / args.image_rate
        for camera_id in range(4):
            message = make_image_message(
                image_stamp, camera_id, frame_index, args.width, args.height
            )
            messages.append((image_stamp, CAMERA_TOPICS[camera_id], message))

    # Generate IMU samples from the first image time to the end of the last
    # image interval.  The first interval is intentionally covered as well.
    imu_count = int(math.ceil(args.duration * args.imu_rate))
    for sample_index in range(imu_count + 1):
        imu_stamp = args.start_time + sample_index / args.imu_rate
        message = make_imu_message(imu_stamp, sample_index)
        messages.append((imu_stamp, IMU_TOPIC, message))

    messages.sort(key=lambda entry: entry[0])
    return messages


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate a small deterministic Sphere-VIO smoke-test bag."
    )
    parser.add_argument("--out", default="/tmp/sphere_vio_synthetic.bag")
    parser.add_argument("--duration", type=float, default=1.2)
    parser.add_argument("--image-rate", type=float, default=10.0)
    parser.add_argument("--imu-rate", type=float, default=200.0)
    parser.add_argument("--width", type=int, default=1088)
    parser.add_argument("--height", type=int, default=880)
    parser.add_argument("--start-time", type=float, default=1700000000.0)
    return parser.parse_args()


def main():
    args = parse_args()
    messages = build_message_list(args)

    with rosbag.Bag(args.out, "w") as bag:
        for _, topic, message in messages:
            bag.write(topic, message, message.header.stamp)

    print("Wrote {} messages to {}".format(len(messages), args.out))
    print("Image frames: {}".format(int(round(args.duration * args.image_rate))))
    print("Image size: {}x{} mono8".format(args.width, args.height))


if __name__ == "__main__":
    main()
