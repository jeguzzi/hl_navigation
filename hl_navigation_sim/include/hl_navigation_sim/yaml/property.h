#ifndef HL_NAVIGATION_SIM_YAML_PROPERTY_H
#define HL_NAVIGATION_SIM_YAML_PROPERTY_H

#include "hl_navigation/property.h"
#include "hl_navigation_sim/world.h"
#include "yaml-cpp/yaml.h"

using namespace hl_navigation;
using namespace hl_navigation_sim;

namespace YAML {

// HACK(Jerome): don't know why I have to define this, as it is already covered in yaml-cpp
// but without it the compiler complains. 

template <>
struct convert<std::vector<bool>> {
  static Node encode(const std::vector<bool>& rhs) {
    Node node;
    for (const bool & item : rhs) {
      node.push_back(item);
    }
    return node;
  }
  static bool decode(const Node& node, std::vector<bool>& rhs) {
    if (node.Type() != YAML::NodeType::Sequence) return false;
    for (const auto& c : node) {
      rhs.push_back(c.as<bool>());
    }
    return true;
  }
};

Property::Field decode_property(const Property& property, const Node& node) {
  return std::visit(
      [&node](auto&& arg) -> Property::Field {
        using T = std::decay_t<decltype(arg)>;
        return node.as<T>();
      },
      property.default_value);
}

Node encode_property(const Property::Field& value) {
  return std::visit([](auto&& arg) { return Node(arg); }, value);
}

template <>
struct convert<Property::Field> {
  static Node encode(const Property::Field & rhs) {
    return encode_property(rhs);
  }
};


}  // namespace YAML

#endif  // HL_NAVIGATION_SIM_YAML_PROPERTY_H