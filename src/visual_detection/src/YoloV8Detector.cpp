#include "dog_detector/YoloV8Detector.h"

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

namespace dog_detector {
namespace {

// 将后处理得到的坐标限制在原图范围内，避免 NMS 后出现越界框。
float clampFloat(float value, float low, float high) {
  return std::max(low, std::min(value, high));
}

// 命令行传入的 device 做大小写归一，兼容 CUDA/cuda/GPU/gpu。
std::string normalizedDevice(std::string device) {
  std::transform(device.begin(), device.end(), device.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return device;
}

}  // namespace

YoloV8Detector::YoloV8Detector(YoloV8Config config)
    : config_(std::move(config)),
      env_(ORT_LOGGING_LEVEL_WARNING, "dog_detector"),
      session_options_() {
  if (config_.class_names.empty()) {
    throw std::runtime_error("YoloV8Detector requires at least one class name");
  }

  // ORT 图优化会融合部分算子，CPU/CUDA 都能受益。
  session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  config_.device = normalizedDevice(config_.device);
  if (config_.device == "cuda" || config_.device == "gpu") {
    // 显式启用 CUDAExecutionProvider；只链接 GPU 版 ORT 不会自动走 GPU。
    OrtCUDAProviderOptions cuda_options{};
    cuda_options.device_id = config_.cuda_device_id;
    session_options_.AppendExecutionProvider_CUDA(cuda_options);
  } else if (config_.device != "cpu") {
    throw std::runtime_error("unsupported inference device: " + config_.device);
  }

  try {
    session_ = std::make_unique<Ort::Session>(env_, config_.model_path.c_str(), session_options_);
  } catch (const Ort::Exception& error) {
    if (config_.device == "cuda" || config_.device == "gpu") {
      // CUDA provider 的动态库、驱动或 GPU 不可见时，错误会在创建 Session 时暴露。
      throw std::runtime_error("failed to create ONNX Runtime CUDA session. Check GPU visibility, "
                               "CUDA/cuDNN compatibility, and LD_LIBRARY_PATH for ONNX Runtime provider "
                               "libraries. Original error: " +
                               std::string(error.what()));
    }
    throw;
  } catch (const std::bad_alloc& error) {
    throw std::runtime_error("stage=create_session failed with std::bad_alloc while loading model: " +
                             config_.model_path);
  }

  // 这个 YOLOv8 模型的输入/输出名固定为 images/output0。某些环境下读取 ORT
  // metadata 会触发 std::bad_alloc，因此这里直接使用配置名，避免额外 metadata 访问。
  input_name_storage_.push_back(config_.input_name);
  output_name_storage_.push_back(config_.output_name);
  input_names_.push_back(input_name_storage_.front().c_str());
  output_names_.push_back(output_name_storage_.front().c_str());
}

std::vector<float> YoloV8Detector::preprocess(const cv::Mat& frame, LetterBoxInfo* info) const {
  if (frame.empty()) {
    throw std::runtime_error("empty frame passed to detector");
  }

  info->original_size = frame.size();
  // letterbox 保持原始宽高比，把空白区域填成 YOLO 常用的 114。
  const float scale = std::min(static_cast<float>(config_.input_width) / frame.cols,
                               static_cast<float>(config_.input_height) / frame.rows);
  const int resized_w = static_cast<int>(std::round(frame.cols * scale));
  const int resized_h = static_cast<int>(std::round(frame.rows * scale));
  info->scale = scale;
  info->pad_x = (config_.input_width - resized_w) / 2;
  info->pad_y = (config_.input_height - resized_h) / 2;

  cv::Mat resized;
  cv::resize(frame, resized, cv::Size(resized_w, resized_h));

  cv::Mat padded(config_.input_height, config_.input_width, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(padded(cv::Rect(info->pad_x, info->pad_y, resized_w, resized_h)));

  cv::Mat rgb;
  cv::cvtColor(padded, rgb, cv::COLOR_BGR2RGB);

  // ONNX 导出的 YOLOv8 通常使用 NCHW float32，像素值归一化到 [0, 1]。
  std::vector<float> blob(3 * config_.input_height * config_.input_width);
  const int channel_size = config_.input_height * config_.input_width;
  for (int y = 0; y < config_.input_height; ++y) {
    const auto* row = rgb.ptr<cv::Vec3b>(y);
    for (int x = 0; x < config_.input_width; ++x) {
      const int offset = y * config_.input_width + x;
      blob[offset] = row[x][0] / 255.0F;
      blob[channel_size + offset] = row[x][1] / 255.0F;
      blob[channel_size * 2 + offset] = row[x][2] / 255.0F;
    }
  }
  return blob;
}

std::vector<Detection> YoloV8Detector::detect(const cv::Mat& frame) {
  LetterBoxInfo letterbox;
  auto input = preprocess(frame, &letterbox);

  // 输入 tensor 使用 CPU 内存；开启 CUDA provider 后 ORT 会自行搬运到 GPU。
  std::array<int64_t, 4> input_shape{
      1, 3, static_cast<int64_t>(config_.input_height), static_cast<int64_t>(config_.input_width)};
  Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  auto input_tensor = Ort::Value::CreateTensor<float>(memory_info,
                                                      input.data(),
                                                      input.size(),
                                                      input_shape.data(),
                                                      input_shape.size());

  // 只取第一个输出。Ultralytics YOLOv8 ONNX 默认输出名是 output0。
  auto outputs = session_->Run(Ort::RunOptions{nullptr},
                               input_names_.data(),
                               &input_tensor,
                               1,
                               output_names_.data(),
                               output_names_.size());
  if (outputs.empty() || !outputs.front().IsTensor()) {
    throw std::runtime_error("ONNX model returned no tensor output");
  }

  auto output_info = outputs.front().GetTensorTypeAndShapeInfo();
  return postprocess(outputs.front().GetTensorData<float>(), output_info.GetShape(), letterbox);
}

std::vector<Detection> YoloV8Detector::postprocess(const float* output,
                                                   const std::vector<int64_t>& shape,
                                                   const LetterBoxInfo& info) const {
  if (shape.size() != 3 || shape[0] != 1) {
    throw std::runtime_error("unsupported YOLO output shape; expected [1, C, N] or [1, N, C]");
  }

  const int class_count = static_cast<int>(config_.class_names.size());
  const int64_t dim1 = shape[1];
  const int64_t dim2 = shape[2];
  // 兼容 [1, C, N] 和 [1, N, C] 两类导出布局。
  const bool channels_first = (dim1 == class_count + 4 || dim1 == class_count + 5 ||
                               (dim1 < dim2 && dim1 <= 256));
  const int64_t prediction_count = channels_first ? dim2 : dim1;
  const int64_t attribute_count = channels_first ? dim1 : dim2;
  if (attribute_count < 4 + class_count) {
    throw std::runtime_error("YOLO output has fewer attributes than expected classes");
  }

  const bool has_objectness = (attribute_count >= 5 + class_count);
  const int class_offset = has_objectness ? 5 : 4;
  // 不同输出布局只在取值函数里区分，后续逻辑保持一致。
  auto value_at = [&](int64_t prediction, int64_t attribute) -> float {
    return channels_first ? output[attribute * prediction_count + prediction]
                          : output[prediction * attribute_count + attribute];
  };

  std::vector<cv::Rect> boxes;
  std::vector<float> scores;
  std::vector<int> class_ids;

  for (int64_t i = 0; i < prediction_count; ++i) {
    // 对每个候选框选择分数最高的类别。
    int best_class = -1;
    float best_score = -std::numeric_limits<float>::infinity();
    for (int c = 0; c < class_count; ++c) {
      const float score = value_at(i, class_offset + c);
      if (score > best_score) {
        best_score = score;
        best_class = c;
      }
    }

    const float objectness = has_objectness ? value_at(i, 4) : 1.0F;
    const float confidence = objectness * best_score;
    if (confidence < config_.confidence_threshold) {
      continue;
    }

    const float cx = value_at(i, 0);
    const float cy = value_at(i, 1);
    const float w = value_at(i, 2);
    const float h = value_at(i, 3);

    // 模型输出是 letterbox 输入坐标，需要去 padding 再除以缩放比例还原到原图。
    float x1 = (cx - w * 0.5F - info.pad_x) / info.scale;
    float y1 = (cy - h * 0.5F - info.pad_y) / info.scale;
    float x2 = (cx + w * 0.5F - info.pad_x) / info.scale;
    float y2 = (cy + h * 0.5F - info.pad_y) / info.scale;

    x1 = clampFloat(x1, 0.0F, static_cast<float>(info.original_size.width - 1));
    y1 = clampFloat(y1, 0.0F, static_cast<float>(info.original_size.height - 1));
    x2 = clampFloat(x2, 0.0F, static_cast<float>(info.original_size.width - 1));
    y2 = clampFloat(y2, 0.0F, static_cast<float>(info.original_size.height - 1));

    const int x = static_cast<int>(std::round(x1));
    const int y = static_cast<int>(std::round(y1));
    const int width = std::max(1, static_cast<int>(std::round(x2 - x1)));
    const int height = std::max(1, static_cast<int>(std::round(y2 - y1)));

    boxes.emplace_back(x, y, width, height);
    scores.push_back(confidence);
    class_ids.push_back(best_class);
  }

  std::vector<Detection> detections;
  for (int class_id = 0; class_id < class_count; ++class_id) {
    // 按类别分别做 NMS，避免不同类别之间互相抑制。
    std::vector<cv::Rect> class_boxes;
    std::vector<float> class_scores;
    std::vector<int> original_indices;
    for (size_t i = 0; i < class_ids.size(); ++i) {
      if (class_ids[i] == class_id) {
        class_boxes.push_back(boxes[i]);
        class_scores.push_back(scores[i]);
        original_indices.push_back(static_cast<int>(i));
      }
    }

    std::vector<int> kept;
    cv::dnn::NMSBoxes(class_boxes,
                      class_scores,
                      config_.confidence_threshold,
                      config_.nms_threshold,
                      kept);
    for (int kept_index : kept) {
      const int source_index = original_indices[kept_index];
      Detection detection;
      detection.class_id = class_id;
      detection.class_name = config_.class_names[class_id];
      detection.confidence = scores[source_index];
      detection.box = cv::Rect2f(boxes[source_index]);
      detections.push_back(std::move(detection));
    }
  }

  std::sort(detections.begin(), detections.end(), [](const Detection& a, const Detection& b) {
    return a.confidence > b.confidence;
  });
  return detections;
}

}  // namespace dog_detector
