#ifndef HL_NAVIGATION_SIM_YAML_BASIC_H
#define HL_NAVIGATION_SIM_YAML_BASIC_H

#include "hl_navigation/common.h"
#include "hl_navigation/states/geometric.h"
#include "yaml-cpp/yaml.h"

using namespace hl_navigation;

namespace YAML {

template <>
struct convert<Vector2> {
  static Node encode(const Vector2& rhs) {
    Node node;
    node.push_back(rhs[0]);
    node.push_back(rhs[1]);
    return node;
  }
  static bool decode(const Node& node, Vector2& rhs) {
    if (!node.IsSequence() || node.size() != 2) {
      return false;
    }
    // std::cout << "convert to Vector2 from " << node << std::endl;
    rhs[0] = node[0].as<float>();
    rhs[1] = node[1].as<float>();
    return true;
  }
};

template <>
struct convert<LineSegment> {
  static Node encode(const LineSegment& rhs) {
    Node node;
    node["p1"] = rhs.p1;
    node["p2"] = rhs.p2;
    return node;
  }
  static bool decode(const Node& node, LineSegment& rhs) {
    if (!node.IsSequence() || node.size() != 2) {
      return false;
    }
    rhs.p1 = node[0].as<Vector2>();
    rhs.p2 = node[1].as<Vector2>();
    rhs.update();
    return true;
  }
};

template <>
struct convert<Disc> {
  static Node encode(const Disc& rhs) {
    Node node;
    node["position"] = rhs.position;
    node["radius"] = rhs.radius;
    return node;
  }
  static bool decode(const Node& node, Disc& rhs) {
    if (!node.IsMap()) {
      return false;
    }
    rhs.position = node["position"].as<Vector2>();
    rhs.radius = node["radius"].as<float>();
    return true;
  }
};

}  // namespace YAML

#endif  // HL_NAVIGATION_SIM_YAML_BASIC_H
