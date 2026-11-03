#pragma once

#include <string>
#include <vector>

namespace dog_detector {

// 从 YOLO data.yaml 中读取 names 字段，返回按 class_id 排序的类别名。
std::vector<std::string> loadClassNames(const std::string& yaml_path);

}  // namespace dog_detector
