#include "sphere_vio/ros/d2slam_stitched.hpp"

#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <boost/make_shared.hpp>

namespace sphere_vio {

bool splitD2SlamStitchedImage(
    const sensor_msgs::CompressedImage& compressed,
    std::array<sensor_msgs::ImagePtr, 4>* images) {
  if (!images) return false;
  images->fill(nullptr);

  const std::vector<unsigned char> encoded(compressed.data.begin(),
                                           compressed.data.end());
  if (encoded.empty()) return false;

  const cv::Mat decoded =
      cv::imdecode(cv::Mat(encoded), cv::IMREAD_GRAYSCALE);
  if (decoded.empty() || decoded.rows <= 0 ||
      decoded.cols % 4 != 0) {
    return false;
  }

  const int sub_width = decoded.cols / 4;
  for (std::size_t camera_id = 0U; camera_id < images->size();
       ++camera_id) {
    const int x = static_cast<int>(camera_id) * sub_width;
    cv::Mat sub_image =
        decoded(cv::Rect(x, 0, sub_width, decoded.rows)).clone();

    sensor_msgs::ImagePtr image = boost::make_shared<sensor_msgs::Image>();
    image->header = compressed.header;
    image->header.frame_id =
        "camera" + std::to_string(static_cast<unsigned int>(camera_id));
    image->height = sub_image.rows;
    image->width = sub_image.cols;
    image->encoding = "mono8";
    image->is_bigendian = 0;
    image->step = static_cast<unsigned int>(sub_image.step[0]);
    image->data.assign(sub_image.datastart, sub_image.dataend);
    (*images)[camera_id] = image;
  }
  return true;
}

}  // namespace sphere_vio
