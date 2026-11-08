#pragma once

#include "dog_detector/DetectionTypes.h"

#include <onnxruntime_cxx_api.h>
#include <opencv2/core.hpp>

#include <memory>
#include <string>
#include <vector>

namespace dog_detector {

// YOLOv8 ONNX 推理配置。默认使用 CUDA，可通过 --device cpu 切换。
struct YoloV8Config {
  std::string model_path = "models/best.onnx";
  int input_width = 640;
  int input_height = 640;
  float confidence_threshold = 0.25F;
  float nms_threshold = 0.45F;
  std::string device = "cuda";
  int cuda_device_id = 0;
  std::string input_name = "images";
  std::string output_name = "output0";
};

// 封装 YOLOv8 ONNX Runtime 推理：预处理、执行模型、后处理和 NMS。
class YoloV8Detector {
 public:
  explicit YoloV8Detector(YoloV8Config config);

  // 输入 BGR 图像帧，输出原图坐标系中的检测框。
  std::vector<Detection> detect(const cv::Mat& frame);

 private:
  // letterbox 信息用于把模型输入坐标还原回原图坐标。
  struct LetterBoxInfo {
    float scale = 1.0F;
    int pad_x = 0;
    int pad_y = 0;
    cv::Size original_size;
  };

  // BGR -> RGB、letterbox、归一化，并整理成 NCHW float tensor。
  std::vector<float> preprocess(const cv::Mat& frame, LetterBoxInfo* info) const;
  // 兼容 YOLOv8 常见输出形状 [1,C,N] 和 [1,N,C]，并做 NMS。
  std::vector<Detection> postprocess(const float* output,
                                     const std::vector<int64_t>& shape,
                                     const LetterBoxInfo& info) const;

  YoloV8Config config_;
  Ort::Env env_;
  Ort::SessionOptions session_options_;
  std::unique_ptr<Ort::Session> session_;
  std::vector<std::string> input_name_storage_;
  std::vector<std::string> output_name_storage_;
  std::vector<const char*> input_names_;
  std::vector<const char*> output_names_;
};

}  // namespace dog_detector
