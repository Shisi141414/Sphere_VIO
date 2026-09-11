#pragma once

#include <string>

#include <opencv2/core.hpp>

#include "sphere_vio/camera/camera_rig.hpp"
#include "sphere_vio/common/types.hpp"
#include "sphere_vio/frontend/omni_rectifier.hpp"
#include "sphere_vio/frontend/orb_descriptor_extractor.hpp"

namespace sphere_vio {

struct SuperPointExtractorOptions {
  std::string model_path;
  int input_width = 320;
  int input_height = 160;
  int output_width = 800;
  int output_height = 400;
  float score_threshold = 0.15F;
  int nms_radius = 20;
  std::size_t maximum_points_per_camera = 150U;
  std::size_t grid_rows = 4U;
  std::size_t grid_columns = 6U;
  bool require_cuda = true;
};

// SuperPoint frontend helper.
//
// It runs one batch of four rectified camera images through the D2SLAM
// compatible `superpoint_v1_sim_int32.onnx` model (inputs: "input", outputs:
// "scores"/"descriptors"), thresholds and NMS-suppresses the score map, and
// returns CameraDescriptorSet objects whose descriptors are L2-normalized
// floats while pixels/bearings remain raw fisheye coordinates.
//
// The heavy implementation is compiled only when ONNX Runtime CUDA is enabled
// with `-DSPHERE_VIO_WITH_ONNXRUNTIME_CUDA=ON`. Without that build flag every
// call fails loudly instead of silently degrading to ORB.
class SuperPointExtractor {
 public:
  explicit SuperPointExtractor(SuperPointExtractorOptions options = {});
  ~SuperPointExtractor();

  // The object owns ONNX Runtime session/environment handles. Copying them
  // would double-free the runtime, so the class is move-only in spirit and
  // non-copyable in practice (callers hold it through unique_ptr).
  SuperPointExtractor(const SuperPointExtractor&) = delete;
  SuperPointExtractor& operator=(const SuperPointExtractor&) = delete;

  const SuperPointExtractorOptions& options() const { return options_; }
  bool available() const;
  std::string error() const { return error_; }

  // `timestamp` is only stored in the produced descriptor sets for diagnostics
  // and statistics; matching and geometry never use it.
  bool extract(const OmniRectifier& rectifier,
               const CameraRig& camera_rig,
               Timestamp timestamp,
               const std::array<cv::Mat, 4>& rectified_images,
               std::array<CameraDescriptorSet, 4>* descriptor_sets);

 private:
  SuperPointExtractorOptions options_;
  std::string error_;
  void* session_ = nullptr;
  void* environment_ = nullptr;
};

}  // namespace sphere_vio
