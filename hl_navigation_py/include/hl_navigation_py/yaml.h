#ifndef HL_NAVIGATION_PY_YAML_H
#define HL_NAVIGATION_PY_YAML_H

#include <pybind11/pybind11.h>

#include "yaml-cpp/yaml.h"

namespace py = pybind11;

namespace YAML {

template <typename T>
py::object make_type_from_yaml_py(const Node &node) {
  if (node.IsMap() && node["type"]) {
    std::string type = node["type"].as<std::string>();
    auto v = T::make_type(type);
    return v;
  }
  return py::none();
}

template <typename T>
py::object load_node(const Node &node) {
  auto obj = make_type_from_yaml_py<T>(node);
  if (!obj.is_none()) {
    convert<typename T::Native>::decode(node, obj.template cast<T &>());
  }
  return obj;
}

template <typename T>
py::object load_py(const std::string &value) {
  try {
    Node node = YAML::Load(value);
    return load_node<T>(node);
  } catch (const YAML::ParserException &ex) {
    return py::none();
  }
}

}  // namespace YAML

#endif  // HL_NAVIGATION_PY_YAML_H