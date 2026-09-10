#pragma once

#include <array>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

namespace sphere_vio {

// Decodes the 4-in-1 compressed image used by D2SLAM full rosbags. The four
// sub-images are assumed to be arranged horizontally and have equal width.
// The returned Image messages share the input header timestamp so the existing
// FrameAssembler can treat them as a single synchronized four-camera frame.
bool splitD2SlamStitchedImage(
    const sensor_msgs::CompressedImage& compressed,
    std::array<sensor_msgs::ImagePtr, 4>* images);

}  // namespace sphere_vio
