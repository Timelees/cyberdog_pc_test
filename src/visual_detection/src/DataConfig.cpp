#include "dog_detector/DataConfig.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace dog_detector {
namespace {

// 去掉 YAML 行首尾空白，便于后续做简单的 key:value 解析。
std::string trim(std::string value) {
  auto first = std::find_if_not(value.begin(), value.end(),
                                [](unsigned char c) { return std::isspace(c); });
  auto last = std::find_if_not(value.rbegin(), value.rend(),
                               [](unsigned char c) { return std::isspace(c); }).base();
  if (first >= last) {
    return {};
  }
  return {first, last};
}

}  // namespace

std::vector<std::string> loadClassNames(const std::string& yaml_path) {
  std::ifstream input(yaml_path);
  if (!input.is_open()) {
    throw std::runtime_error("failed to open label yaml: " + yaml_path);
  }

  std::map<int, std::string> names;
  std::string line;
  // 这里只需要 data.yaml 的 names 段，所以用轻量解析避免引入额外 YAML 依赖。
  bool in_names = false;

  while (std::getline(input, line)) {
    // 支持行尾注释，例如 data.yaml 中的路径说明。
    auto comment = line.find('#');
    if (comment != std::string::npos) {
      line = line.substr(0, comment);
    }
    line = trim(line);
    if (line.empty()) {
      continue;
    }
    if (line == "names:" || line.rfind("names:", 0) == 0) {
      // 从 names: 行之后开始解析类别 id 和类别名。
      in_names = true;
      continue;
    }
    if (!in_names) {
      continue;
    }

    auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }

    const auto key = trim(line.substr(0, colon));
    auto value = trim(line.substr(colon + 1));
    // 只接受数字 class_id，忽略其他 YAML 字段。
    if (key.empty() || value.empty() ||
        !std::all_of(key.begin(), key.end(), [](unsigned char c) { return std::isdigit(c); })) {
      continue;
    }
    if ((value.front() == '\'' && value.back() == '\'') ||
        (value.front() == '"' && value.back() == '"')) {
      // 兼容 0: 'soccer' 和 0: "soccer" 两种写法。
      value = value.substr(1, value.size() - 2);
    }
    names[std::stoi(key)] = value;
  }

  if (names.empty()) {
    throw std::runtime_error("no class names found in: " + yaml_path);
  }

  // 使用 map 保证按类别 id 排序，最终 vector 下标就是 class_id。
  std::vector<std::string> ordered(names.rbegin()->first + 1);
  for (const auto& [id, name] : names) {
    ordered[id] = name;
  }
  return ordered;
}

}  // namespace dog_detector
