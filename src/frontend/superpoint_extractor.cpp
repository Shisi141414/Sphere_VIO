#include "sphere_vio/frontend/superpoint_extractor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

// ONNX Runtime 的头文件只在显式打开 CUDA 构建开关时引入。这样默认（CPU /
// ROS Noetic）构建完全不依赖 ONNX Runtime，SuperPoint 类仍然可以链接，但
// available() 恒为 false、extract() 会给出明确错误，绝不会悄悄退回 ORB。
#ifdef SPHERE_VIO_WITH_ONNXRUNTIME_CUDA
#include <onnxruntime_cxx_api.h>
#endif

namespace sphere_vio {
namespace {

constexpr std::size_t kCameraCount = 4U;

// ---------------------------------------------------------------------------
// 配置合法性检查。所有几何量（宽高、阈值、半径、数量）都必须是有限且非负的。
// ---------------------------------------------------------------------------
bool validOptions(const SuperPointExtractorOptions& options) {
  return options.input_width > 0 && options.input_height > 0 &&
         options.output_width > 0 && options.output_height > 0 &&
         std::isfinite(options.score_threshold) &&
         options.score_threshold >= 0.0F && options.nms_radius > 0 &&
         options.maximum_points_per_camera > 0U && options.grid_rows > 0U &&
         options.grid_columns > 0U;
}

// 以下候选点筛选、网格均匀化和描述子采样只在 CUDA 构建中被调用；包在条件
// 编译里可以避免默认构建出现 “unused function” 告警（项目目前是 C++14，
// 无法使用 C++17 的 [[maybe_unused]] 属性）。
#ifdef SPHERE_VIO_WITH_ONNXRUNTIME_CUDA

// 一个候选角点：位于校正画布上的行列坐标与对应的响应分数。
struct HeatmapCandidate {
  int row = 0;
  int column = 0;
  float score = 0.0F;
};

// 稳定的降序排序键。先比分数，分数相同时按行列坐标确定唯一顺序，保证每次
// 运行输出的角点与描述子完全一致（确定性），便于回归测试与结果复现。
struct CandidateOrder {
  bool operator()(const HeatmapCandidate& lhs,
                  const HeatmapCandidate& rhs) const {
    if (lhs.score != rhs.score) return lhs.score > rhs.score;
    if (lhs.row != rhs.row) return lhs.row < rhs.row;
    return lhs.column < rhs.column;
  }
};

// ---------------------------------------------------------------------------
// 半径邻域极大值抑制 + 网格均匀化，全部只依赖 OpenCV，方便与模型推理部分
// 分开验证。
// ---------------------------------------------------------------------------
// 用最大滤波实现 NMS：先对分数图做半径为 nms_radius 的膨胀（取邻域最大值），
// 再保留“自己的分数不小于邻域最大值”的像素。这样时间复杂度是 O(H*W) 量级，
// 而不是对每个候选点枚举邻域的 O(N*N)。
std::vector<HeatmapCandidate> nmsCandidates(
    const cv::Mat& scores, float score_threshold, int nms_radius) {
  std::vector<HeatmapCandidate> candidates;
  if (scores.empty() || scores.type() != CV_32FC1 || nms_radius <= 0) {
    return candidates;
  }
  const int kernel = 2 * nms_radius + 1;
  const cv::Mat structuring =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel, kernel));
  cv::Mat dilated;
  cv::dilate(scores, dilated, structuring);
  candidates.reserve(static_cast<std::size_t>(scores.rows * scores.cols / 8));
  for (int row = 0; row < scores.rows; ++row) {
    const float* score_row = scores.ptr<float>(row);
    const float* dilated_row = dilated.ptr<float>(row);
    for (int column = 0; column < scores.cols; ++column) {
      const float score = score_row[column];
      // 阈值门控 + 局部最大值门控。减去一个小 epsilon 使并排相等分数的
      // 像素只保留排序后确定的一个（由下面的稳定排序保证）。
      if (score > score_threshold &&
          score >= dilated_row[column] - 1e-6F) {
        candidates.push_back(HeatmapCandidate{row, column, score});
      }
    }
  }
  std::stable_sort(candidates.begin(), candidates.end(), CandidateOrder{});
  return candidates;
}

// 网格均匀化：把校正画布分成 grid_rows x grid_columns 个单元，先让每个单元
// 最多拿到 ceil(max_points / 单元数) 个最强角点（保证空间覆盖），配额没用完
// 时再按全局分数补足到 maximum_points_per_camera 个（保证数量）。
void uniformizeCandidates(std::size_t maximum_points, std::size_t grid_rows,
                          std::size_t grid_columns, int width, int height,
                          std::vector<HeatmapCandidate>* candidates) {
  if (!candidates || candidates->empty() || width <= 0 || height <= 0 ||
      grid_rows == 0U || grid_columns == 0U) {
    return;
  }
  const std::size_t cell_count = grid_rows * grid_columns;
  const std::size_t per_cell =
      (maximum_points + cell_count - 1U) / cell_count;
  std::vector<std::size_t> accepted_per_cell(cell_count, 0U);
  std::vector<bool> chosen(candidates->size(), false);
  std::vector<HeatmapCandidate> selected;
  selected.reserve(maximum_points);

  // 第一遍：按分数从高到低，逐单元限流。
  for (std::size_t index = 0U; index < candidates->size(); ++index) {
    const HeatmapCandidate& candidate = (*candidates)[index];
    const std::size_t row_cell = static_cast<std::size_t>(
        static_cast<std::uint64_t>(candidate.row) *
        static_cast<std::uint64_t>(grid_rows) /
        static_cast<std::uint64_t>(height));
    const std::size_t column_cell = static_cast<std::size_t>(
        static_cast<std::uint64_t>(candidate.column) *
        static_cast<std::uint64_t>(grid_columns) /
        static_cast<std::uint64_t>(width));
    if (row_cell >= grid_rows || column_cell >= grid_columns) continue;
    const std::size_t cell = row_cell * grid_columns + column_cell;
    if (accepted_per_cell[cell] >= per_cell) continue;
    ++accepted_per_cell[cell];
    chosen[index] = true;
    selected.push_back(candidate);
    if (selected.size() >= maximum_points) break;
  }

  // 第二遍：如果某些单元没有足够角点，用全局剩余最强角点补足总数。
  for (std::size_t index = 0U; index < candidates->size(); ++index) {
    if (selected.size() >= maximum_points) break;
    if (chosen[index]) continue;
    selected.push_back((*candidates)[index]);
  }
  *candidates = std::move(selected);
}

// 对描述子网格做双线性采样。描述子来自 D2SLAM 兼容模型（通常是原图 1/8
// 分辨率的稠密张量），因此把校正画布坐标按比例映射到描述子网格再插值。
void sampleDescriptor(const float* descriptor_data, int channels,
                      int descriptor_width, int descriptor_height,
                      double column, double row, int canvas_width,
                      int canvas_height, std::vector<float>* descriptor) {
  descriptor->assign(static_cast<std::size_t>(channels), 0.0F);
  if (!descriptor_data || channels <= 0 || descriptor_width <= 0 ||
      descriptor_height <= 0 || canvas_width <= 0 || canvas_height <= 0) {
    return;
  }
  // 角点位于像素中心（column+0.5, row+0.5），映射到描述子网格坐标。
  const double x = (column + 0.5) / canvas_width * descriptor_width - 0.5;
  const double y = (row + 0.5) / canvas_height * descriptor_height - 0.5;
  // 越界一小段时做边框复制，避免在画布边缘丢掉角点。
  const double clamped_x =
      std::max(0.0, std::min(static_cast<double>(descriptor_width - 1), x));
  const double clamped_y =
      std::max(0.0, std::min(static_cast<double>(descriptor_height - 1), y));
  const int x0 = static_cast<int>(std::floor(clamped_x));
  const int y0 = static_cast<int>(std::floor(clamped_y));
  const int x1 = std::min(x0 + 1, descriptor_width - 1);
  const int y1 = std::min(y0 + 1, descriptor_height - 1);
  const double tx = clamped_x - static_cast<double>(x0);
  const double ty = clamped_y - static_cast<double>(y0);
  const double w00 = (1.0 - tx) * (1.0 - ty);
  const double w10 = tx * (1.0 - ty);
  const double w01 = (1.0 - tx) * ty;
  const double w11 = tx * ty;
  for (int channel = 0; channel < channels; ++channel) {
    const std::size_t base =
        static_cast<std::size_t>(channel * descriptor_height *
                                 descriptor_width);
    const std::size_t y0_offset =
        base + static_cast<std::size_t>(y0 * descriptor_width);
    const std::size_t y1_offset =
        base + static_cast<std::size_t>(y1 * descriptor_width);
    (*descriptor)[static_cast<std::size_t>(channel)] = static_cast<float>(
        w00 * descriptor_data[y0_offset + x0] +
        w10 * descriptor_data[y0_offset + x1] +
        w01 * descriptor_data[y1_offset + x0] +
        w11 * descriptor_data[y1_offset + x1]);
  }
}

// L2 归一化。零向量没有方向，直接返回 false 让调用方丢弃这个点，避免把零
// 描述子交给匹配器产生无意义的最邻近。
bool l2Normalize(std::vector<float>* descriptor) {
  if (!descriptor) return false;
  double squared = 0.0;
  for (const float value : *descriptor) squared += value * value;
  const float norm = static_cast<float>(std::sqrt(squared));
  if (!std::isfinite(norm) || norm < 1e-8F) return false;
  for (float& value : *descriptor) value /= norm;
  return true;
}

#endif  // SPHERE_VIO_WITH_ONNXRUNTIME_CUDA

}  // namespace

SuperPointExtractor::SuperPointExtractor(SuperPointExtractorOptions options)
    : options_(std::move(options)) {
  if (!validOptions(options_)) {
    error_ = "invalid SuperPoint options";
    return;
  }
#ifdef SPHERE_VIO_WITH_ONNXRUNTIME_CUDA
  try {
    // Environment 是整个推理进程的全局句柄，Session 是单个模型的句柄。二者
    // 都放在堆上并只存裸指针，这样本头文件无需暴露 ONNX Runtime 头文件。
    std::unique_ptr<Ort::Env> environment(
        new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "sphere_vio_superpoint"));
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(
        GraphOptimizationLevel::ORT_ENABLE_ALL);
    // 显式要求 CUDA Provider。没有 NVIDIA 驱动 / CUDA 运行库时这里会抛异常，
    // 构造函数把异常转成 error_，extract() 随后明确失败，而不是静默退到 CPU。
    Ort::ThrowOnError(
        OrtSessionOptionsAppendExecutionProvider_CUDA(session_options, 0));
    std::unique_ptr<Ort::Session> session(new Ort::Session(
        *environment, options_.model_path.c_str(), session_options));
    // 全部初始化成功后才转移所有权；任一步失败时 unique_ptr 自动释放，不会
    // 泄漏已经创建的 Environment。
    environment_ = environment.release();
    session_ = session.release();
  } catch (const Ort::Exception& exception) {
    error_ = std::string("ONNX Runtime CUDA initialization failed: ") +
             exception.what();
  }
#else
  error_ = "SuperPoint mode was requested, but Sphere-VIO was built without "
           "ONNX Runtime CUDA (set -DSPHERE_VIO_WITH_ONNXRUNTIME_CUDA=ON)";
#endif
}

SuperPointExtractor::~SuperPointExtractor() {
#ifdef SPHERE_VIO_WITH_ONNXRUNTIME_CUDA
  // 释放顺序与构造相反：先销毁 Session 再销毁全局 Environment。
  delete static_cast<Ort::Session*>(session_);
  delete static_cast<Ort::Env*>(environment_);
#endif
  session_ = nullptr;
  environment_ = nullptr;
}

bool SuperPointExtractor::available() const {
  return error_.empty() && session_ != nullptr && environment_ != nullptr;
}

bool SuperPointExtractor::extract(
    const OmniRectifier& rectifier, const CameraRig& camera_rig,
    Timestamp timestamp, const std::array<cv::Mat, kCameraCount>& rectified_images,
    std::array<CameraDescriptorSet, kCameraCount>* descriptor_sets) {
  if (!descriptor_sets) return false;
  *descriptor_sets = {};
  if (!validOptions(options_)) {
    error_ = "invalid SuperPoint options";
    return false;
  }
#ifndef SPHERE_VIO_WITH_ONNXRUNTIME_CUDA
  // 默认构建里这些参数没有消费者；显式 (void) 化可以避免 -Wextra 的
  // unused-parameter 告警刷屏。
  (void)rectifier;
  (void)camera_rig;
  (void)timestamp;
  (void)rectified_images;
  error_ = "SuperPoint mode was requested, but Sphere-VIO was built without "
           "ONNX Runtime CUDA (set -DSPHERE_VIO_WITH_ONNXRUNTIME_CUDA=ON)";
  return false;
#else
  if (!available()) return false;
  if (!std::isfinite(timestamp) || camera_rig.size() < kCameraCount) {
    error_ = "SuperPoint extract received invalid timestamp or camera rig";
    return false;
  }
  // 校正画布尺寸必须与提取器配置一致，否则“画布像素 -> 原始鱼眼像素”的映射
  // 和描述子网格映射都会错位。
  if (rectifier.width() != options_.output_width ||
      rectifier.height() != options_.output_height) {
    error_ = "rectifier canvas size does not match SuperPoint options";
    return false;
  }
  for (std::size_t camera_id = 0U; camera_id < kCameraCount; ++camera_id) {
    const cv::Mat& image = rectified_images[camera_id];
    if (image.empty() || (image.type() != CV_8UC1 && image.type() != CV_8UC3) ||
        image.cols != options_.output_width ||
        image.rows != options_.output_height || !camera_rig.hasCamera(
            static_cast<CameraId>(camera_id))) {
      error_ = "SuperPoint extract received an invalid rectified image";
      return false;
    }
  }

  Ort::Session& session = *static_cast<Ort::Session*>(session_);
  Ort::AllocatorWithDefaultOptions allocator;

  // ---- 校验模型输入：名字必须是 "input"，张量必须是 4 维 NCHW。 ----
  if (session.GetInputCount() < 1U || session.GetOutputCount() < 2U) {
    error_ = "SuperPoint model must have one input and two outputs";
    return false;
  }
  const std::string input_name = [&]() {
    Ort::AllocatedStringPtr name = session.GetInputNameAllocated(0U, allocator);
    return std::string(name.get());
  }();
  if (input_name != "input") {
    error_ = "unexpected model input name: '" + input_name + "'";
    return false;
  }
  Ort::TypeInfo input_type = session.GetInputTypeInfo(0U);
  const auto input_tensor_info = input_type.GetTensorTypeAndShapeInfo();
  const std::vector<std::int64_t> input_shape = input_tensor_info.GetShape();
  if (input_shape.size() != 4U ||
      !(input_shape[0] == 1 || input_shape[0] == 4 || input_shape[0] == -1 ||
        input_shape[0] == 0) ||
      !(input_shape[1] == 1 || input_shape[1] == 3)) {
    error_ = "SuperPoint model input must be 4D NCHW with static 1 or 3 "
             "channels";
    return false;
  }
  const int channels = static_cast<int>(input_shape[1]);
  const bool batch_is_four = input_shape[0] != 1;
  // 动态维度（-1/0）允许我们按配置缩放；静态维度必须与配置一致，否则直接报错，
  // 不让模型悄悄接受不同分辨率的输入。
  if ((input_shape[2] > 0 && input_shape[2] != options_.input_height) ||
      (input_shape[3] > 0 && input_shape[3] != options_.input_width)) {
    error_ = "SuperPoint model input resolution does not match configuration";
    return false;
  }
  if (input_tensor_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
    error_ = "SuperPoint model input must be float32 (found element type " +
             std::to_string(input_tensor_info.GetElementType()) + ")";
    return false;
  }

  // ---- 把四张校正图缩放成 NCHW 的 float32 批张量，值域 [0,1]。 ----
  const std::size_t per_image = static_cast<std::size_t>(channels) *
                                options_.input_height * options_.input_width;
  std::vector<float> input_data(kCameraCount * per_image, 0.0F);
  for (std::size_t camera_id = 0U; camera_id < kCameraCount; ++camera_id) {
    cv::Mat gray;
    if (rectified_images[camera_id].type() == CV_8UC3) {
      cv::cvtColor(rectified_images[camera_id], gray, cv::COLOR_BGR2GRAY);
    } else {
      gray = rectified_images[camera_id];
    }
    cv::Mat resized;
    cv::resize(gray, resized,
               cv::Size(options_.input_width, options_.input_height), 0.0, 0.0,
               cv::INTER_LINEAR);
    float* destination =
        input_data.data() + camera_id * per_image;
    for (int row = 0; row < resized.rows; ++row) {
      const std::uint8_t* source = resized.ptr<std::uint8_t>(row);
      float* channel_destination =
          destination + static_cast<std::size_t>(row * options_.input_width);
      for (int column = 0; column < resized.cols; ++column) {
        const float normalized =
            static_cast<float>(source[column]) / 255.0F;
        // 单通道直接写入，三通道把同一灰度复制三份（模型通常只取亮度信息）。
        for (int channel = 0; channel < channels; ++channel) {
          channel_destination[channel * options_.input_height *
                                  options_.input_width +
                              column] = normalized;
        }
      }
    }
  }

  // ---- 输出名字校验 + 一次/四次推理。 ----
  const std::string output_name_0 = [&]() {
    Ort::AllocatedStringPtr name = session.GetOutputNameAllocated(0U, allocator);
    return std::string(name.get());
  }();
  const std::string output_name_1 = [&]() {
    Ort::AllocatedStringPtr name = session.GetOutputNameAllocated(1U, allocator);
    return std::string(name.get());
  }();
  if ((output_name_0 != "scores" || output_name_1 != "descriptors") &&
      (output_name_0 != "descriptors" || output_name_1 != "scores")) {
    error_ = "unexpected SuperPoint model outputs: '" + output_name_0 +
             "' / '" + output_name_1 + "'";
    return false;
  }
  const bool scores_first = output_name_0 == "scores";

  Ort::MemoryInfo memory_info =
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  const char* input_names[] = {"input"};
  const char* output_names[] = {"scores", "descriptors"};

  // 模型的 batch 维度声明为 1 时逐张推理，声明为 4/动态时一次批处理。
  const std::size_t batch_size = batch_is_four ? kCameraCount : 1U;
  std::vector<cv::Mat> score_maps(kCameraCount);
  std::vector<const float*> descriptor_maps(kCameraCount, nullptr);
  int descriptor_channels = 0;
  int descriptor_width = 0;
  int descriptor_height = 0;

  for (std::size_t batch_start = 0U; batch_start < kCameraCount;
       batch_start += batch_size) {
    std::vector<std::int64_t> batch_shape{
        static_cast<std::int64_t>(batch_size), channels,
        options_.input_height, options_.input_width};
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, input_data.data() + batch_start * per_image,
        batch_size * per_image, batch_shape.data(), batch_shape.size());
    std::vector<Ort::Value> outputs;
    try {
      outputs = session.Run(Ort::RunOptions{nullptr}, input_names, &input_tensor,
                            1U, output_names, 2U);
    } catch (const Ort::Exception& exception) {
      error_ = std::string("SuperPoint inference failed: ") + exception.what();
      return false;
    }
    if (outputs.size() != 2U) {
      error_ = "SuperPoint model returned an unexpected number of outputs";
      return false;
    }
    Ort::Value& scores_value = outputs[scores_first ? 0U : 1U];
    Ort::Value& descriptor_value = outputs[scores_first ? 1U : 0U];

    // 分数图：[N,H,W] 或 [N,1,H,W]，必须是 float32。
    auto scores_info = scores_value.GetTensorTypeAndShapeInfo();
    std::vector<std::int64_t> scores_shape = scores_info.GetShape();
    if (scores_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        !(scores_shape.size() == 3U || scores_shape.size() == 4U)) {
      error_ = "SuperPoint scores output must be a float32 heatmap";
      return false;
    }
    const int score_height = static_cast<int>(
        scores_shape.size() == 3U ? scores_shape[1] : scores_shape[2]);
    const int score_width = static_cast<int>(
        scores_shape.size() == 3U ? scores_shape[2] : scores_shape[3]);
    if (score_height <= 0 || score_width <= 0) {
      error_ = "SuperPoint scores output has an invalid resolution";
      return false;
    }
    const float* scores_data = scores_value.GetTensorData<float>();
    const std::size_t score_plane = static_cast<std::size_t>(
        score_height * score_width);
    for (std::size_t offset = 0U; offset < batch_size; ++offset) {
      const std::size_t camera_id = batch_start + offset;
      cv::Mat plane(score_height, score_width, CV_32FC1,
                    const_cast<float*>(scores_data + offset * score_plane));
      if (plane.cols != options_.output_width ||
          plane.rows != options_.output_height) {
        // 模型可能按输入分辨率输出分数图；统一放大回校正画布再做 NMS。
        cv::resize(plane, score_maps[camera_id],
                   cv::Size(options_.output_width, options_.output_height),
                   0.0, 0.0, cv::INTER_LINEAR);
      } else {
        score_maps[camera_id] = plane.clone();
      }
    }

    // 描述子：[N,C,Hd,Wd]，float32。记录网格尺寸，后面按画布比例采样。
    auto descriptor_info = descriptor_value.GetTensorTypeAndShapeInfo();
    std::vector<std::int64_t> descriptor_shape = descriptor_info.GetShape();
    if (descriptor_info.GetElementType() !=
            ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        descriptor_shape.size() != 4U) {
      error_ = "SuperPoint descriptors output must be a float32 NCHW tensor";
      return false;
    }
    if (descriptor_channels == 0) {
      descriptor_channels = static_cast<int>(descriptor_shape[1]);
      descriptor_height = static_cast<int>(descriptor_shape[2]);
      descriptor_width = static_cast<int>(descriptor_shape[3]);
      if (descriptor_channels <= 0 || descriptor_height <= 0 ||
          descriptor_width <= 0) {
        error_ = "SuperPoint descriptors output has invalid dimensions";
        return false;
      }
    }
    const float* descriptor_data = descriptor_value.GetTensorData<float>();
    const std::size_t descriptor_plane = static_cast<std::size_t>(
        descriptor_channels * descriptor_height * descriptor_width);
    for (std::size_t offset = 0U; offset < batch_size; ++offset) {
      descriptor_maps[batch_start + offset] =
          descriptor_data + offset * descriptor_plane;
    }
  }

  // ---- 每个相机独立做 NMS、网格均匀化、像素/方位转换与描述子采样。 ----
  for (std::size_t camera_id = 0U; camera_id < kCameraCount; ++camera_id) {
    cv::Mat mask;
    if (!rectifier.rectifiedMask(static_cast<CameraId>(camera_id), &mask)) {
      error_ = "rectifier could not produce a mask for camera " +
               std::to_string(camera_id);
      return false;
    }
    std::vector<HeatmapCandidate> candidates = nmsCandidates(
        score_maps[camera_id], options_.score_threshold, options_.nms_radius);
    uniformizeCandidates(options_.maximum_points_per_camera, options_.grid_rows,
                         options_.grid_columns, options_.output_width,
                         options_.output_height, &candidates);

    CameraDescriptorSet& descriptor_set =
        (*descriptor_sets)[camera_id];
    descriptor_set.camera_id = static_cast<CameraId>(camera_id);
    descriptor_set.timestamp = timestamp;
    descriptor_set.descriptor_format = DescriptorFormat::kL2;
    const RigCamera* rig_camera =
        camera_rig.camera(static_cast<CameraId>(camera_id));

    for (const HeatmapCandidate& candidate : candidates) {
      // 校正图有效掩膜之外的点（例如镜头外黑色区域）没有几何意义，跳过。
      if (candidate.row < 0 || candidate.row >= mask.rows ||
          candidate.column < 0 || candidate.column >= mask.cols ||
          mask.at<std::uint8_t>(candidate.row, candidate.column) == 0U) {
        continue;
      }
      const Eigen::Vector2d rectified_pixel(candidate.column, candidate.row);
      Eigen::Vector2d raw_pixel;
      if (!rectifier.rawPixelFromRectified(
              static_cast<CameraId>(camera_id), rectified_pixel, &raw_pixel)) {
        continue;
      }
      Eigen::Vector3d bearing_c;
      Eigen::Vector3d bearing_b;
      if (!rig_camera || !rig_camera->model ||
          !rig_camera->model->unproject(raw_pixel, &bearing_c) ||
          !camera_rig.cameraBearingToBody(
              static_cast<CameraId>(camera_id), bearing_c, &bearing_b)) {
        continue;
      }
      std::vector<float> descriptor;
      sampleDescriptor(descriptor_maps[camera_id], descriptor_channels,
                       descriptor_width, descriptor_height,
                       static_cast<double>(candidate.column),
                       static_cast<double>(candidate.row),
                       options_.output_width, options_.output_height,
                       &descriptor);
      if (!l2Normalize(&descriptor)) continue;

      // FeatureId 在单帧内按相机划分编号区间，保证跨相机全局唯一且非零。
      const FeatureId feature_id =
          (static_cast<FeatureId>(camera_id + 1U) << 24) |
          static_cast<FeatureId>(descriptor_set.feature_ids.size() + 1U);
      descriptor_set.feature_ids.push_back(feature_id);
      descriptor_set.pixels.push_back(raw_pixel);
      descriptor_set.bearings_c.push_back(bearing_c);
      descriptor_set.bearings_b.push_back(bearing_b);
      if (descriptor_set.descriptors.empty()) {
        descriptor_set.descriptors.create(0, descriptor_channels, CV_32FC1);
      }
      cv::Mat row(1, descriptor_channels, CV_32FC1,
                  const_cast<float*>(descriptor.data()));
      descriptor_set.descriptors.push_back(row.clone());
    }
    // 所有点都被掩膜/几何/描述子丢弃时返回空集合，这是合法结果而不是错误；
    // 匹配器会正常处理空集合。
    if (descriptor_set.feature_ids.empty()) {
      descriptor_set.descriptors.release();
    }
  }
  return true;
#endif
}

}  // namespace sphere_vio
