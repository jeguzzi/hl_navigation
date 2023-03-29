#ifndef HL_NAVIGATION_YAML_YAML_H
#define HL_NAVIGATION_YAML_YAML_H

#include <iostream>
#include <string>
#include <memory>

#include "yaml-cpp/yaml.h"

namespace YAML {

template <typename T>
inline std::shared_ptr<T> load(const Node &node) {
  return node.as<std::shared_ptr<T>>();
}

template <typename T>
inline std::shared_ptr<T> load(const std::string &value) {
  YAML::Node node;
  try {
    node = YAML::Load(value);
  } catch (const YAML::ParserException &e) {
    std::cerr << e.what() << std::endl;
    return nullptr;
  }
  return load<T>(node);
}

template <typename T>
std::string dump(const T *obj) {
  if (!obj) return "";
  YAML::Emitter out;
  out << YAML::Node(*obj);
  return std::string(out.c_str());
}

}  // namespace YAML

#endif  // HL_NAVIGATION_YAML_YAML_H
