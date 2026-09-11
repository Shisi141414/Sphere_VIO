#include <array>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "sphere_vio/frontend/superpoint_extractor.hpp"

namespace sphere_vio {
namespace {

// 两个测试都同时覆盖“默认构建（无 ONNX Runtime CUDA）”和“显式 CUDA 构建”。
//
// - 默认构建：类被编译成 stub，构造后 error_ 非空、available() 恒为 false。
// - CUDA 构建：没有合法模型路径（或没有 GPU）时，构造函数抛出的 ONNX 异常
//   同样被转成 error_，available() 仍为 false。
// 因此“明确失败、绝不静默降级”的契约可以用同一组断言在两种构建下验证。

TEST(SuperPointExtractorTest, InvalidOptionsAreRejected) {
  SuperPointExtractorOptions options;
  options.nms_radius = 0;  // 非法半径必须在构造阶段被拒绝。
  const SuperPointExtractor extractor(options);
  EXPECT_FALSE(extractor.available());
  EXPECT_FALSE(extractor.error().empty());

  std::array<CameraDescriptorSet, 4> descriptor_sets;
  const std::array<cv::Mat, 4> images{{}};
  const OmniRectifier rectifier(CameraRig{}, OmniRectifierOptions{});
  EXPECT_FALSE(extractor.extract(rectifier, CameraRig{}, 1.0, images,
                                 &descriptor_sets));
}

TEST(SuperPointExtractorTest, DefaultBuildWithoutModelIsUnavailable) {
  // 默认选项数值合法，但 model_path 为空。默认构建没有 ONNX 支持，CUDA 构建
  // 会因为空路径/缺失 GPU 初始化失败，两种情况下都不能 available()。
  const SuperPointExtractor extractor;
  EXPECT_FALSE(extractor.available());
  EXPECT_FALSE(extractor.error().empty());

  std::array<CameraDescriptorSet, 4> descriptor_sets;
  const std::array<cv::Mat, 4> images{{}};
  const OmniRectifier rectifier(CameraRig{}, OmniRectifierOptions{});
  EXPECT_FALSE(extractor.extract(rectifier, CameraRig{}, 1.0, images,
                                 &descriptor_sets));
}

}  // namespace
}  // namespace sphere_vio
