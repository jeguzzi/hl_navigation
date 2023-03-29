#ifndef HL_NAVIGATION_SIM_YAML_SAMPLER_H
#define HL_NAVIGATION_SIM_YAML_SAMPLER_H

#include <iostream>
#include <memory>

#include "hl_navigation/yaml/property.h"
#include "hl_navigation_sim/sampling/agent.h"
#include "hl_navigation_sim/sampling/register.h"
#include "hl_navigation_sim/sampling/sampler.h"
#include "hl_navigation_sim/yaml/world.h"
#include "yaml-cpp/yaml.h"

using hl_navigation::Property;
using hl_navigation_sim::AgentSampler;
using hl_navigation_sim::BehaviorSampler;
using hl_navigation_sim::Constant;
using hl_navigation_sim::KinematicSampler;
using hl_navigation_sim::PropertySampler;
using hl_navigation_sim::Regular;
using hl_navigation_sim::Sampler;
using hl_navigation_sim::SamplerFromRegister;
using hl_navigation_sim::Sequence;
using hl_navigation_sim::StateEstimationSampler;
using hl_navigation_sim::TaskSampler;
using hl_navigation_sim::World;

namespace YAML {

std::unique_ptr<PropertySampler> property_sampler(const Node& node,
                                                  const Property& property) {
  try {
    const auto value = decode_property(property, node);
    return std::make_unique<Constant<Property::Field>>(value);
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
  }
  if (node.Type() == YAML::NodeType::Sequence) {
    std::vector<Property::Field> values;
    for (const YAML::Node& c : node) {
      const auto value = decode_property(property, c);
      values.push_back(value);
    }
    if (values.size()) {
      return std::make_unique<Sequence<Property::Field>>(values);
    }
  }
  return nullptr;
}

template <typename T>
std::unique_ptr<Sampler<T>> read_regular_sampler(const Node& node) {
  if (node["start"] && node["end"] && node["number"]) {
    return std::make_unique<Regular<T>>(node["start"].as<T>(),
                                        node["end"].as<T>(),
                                        node["number"].as<unsigned>());
  }
  return nullptr;
}

template <typename T>
std::unique_ptr<Sampler<T>> read_sampler(const Node& node) {
  try {
    return std::make_unique<Constant<T>>(node.as<T>());
  } catch (YAML::BadConversion) {
  }
  if (node.Type() == YAML::NodeType::Sequence) {
    try {
      return std::make_unique<Sequence<T>>(node.as<std::vector<T>>());
    } catch (YAML::BadConversion) {
    }
  }
  if (node.Type() == YAML::NodeType::Map) {
    if (node["regular"]) {
      return read_regular_sampler<T>(node["regular"]);
    }
  }
  return nullptr;
}

template <typename T>
struct convert<Constant<T>> {
  static Node encode(const Constant<T>& rhs) {
    Node node;
    node["sampler"] = "constant";
    node["value"] = rhs.value;
    return node;
    // return Node(rhs.value);
  }
};

template <typename T>
struct convert<Sequence<T>> {
  static Node encode(const Sequence<T>& rhs) {
    Node node;
    node["sampler"] = "sequence";
    node["values"] = rhs.values;
    return node;
    // return Node(rhs.values);
  }
};

template <typename T>
struct convert<Regular<T>> {
  static Node encode(const Regular<T>& rhs) {
    Node node;
    node["start"] = rhs.start;
    node["step"] = rhs.step;
    node["number"] = rhs.number;
    node["sampler"] = "regular";
    return node;
  }
};

template <typename T>
struct convert<std::shared_ptr<Sampler<T>>> {
  static Node encode(const std::shared_ptr<Sampler<T>>& rhs) {
    if (!rhs) return Node();
    if (const Constant<T>* sampler = dynamic_cast<Constant<T>*>(rhs.get())) {
      return Node(*sampler);
    }
    if (const Sequence<T>* sampler = dynamic_cast<Sequence<T>*>(rhs.get())) {
      return Node(*sampler);
    }
    if (const Regular<T>* sampler = dynamic_cast<Regular<T>*>(rhs.get())) {
      return Node(*sampler);
    }
    return Node();
  }
};

template <typename T>
bool decode_sr(const Node& node, SamplerFromRegister<T>* rhs) {
  if (!node.IsMap() || !node["type"]) {
    return false;
  }
  rhs->type = node["type"].as<std::string>();
  const auto& properties = T::type_properties();
  if (!properties.count(rhs->type)) {
    std::cerr << "No type " << rhs->type << " in register "
              << get_type_name<T>() << std::endl;
    for (const auto& s : T::types()) {
      std::cout << s << std::endl;
    }
    return false;
  }
  for (const auto& [name, property] : properties.at(rhs->type)) {
    if (node[name]) {
      rhs->properties[name] = property_sampler(node[name], property);
    }
  }
  return true;
}

template <typename T>
Node encode_sr(const SamplerFromRegister<T>& rhs) {
  Node node;
  node["type"] = rhs.type;
  for (const auto& [name, property_sampler] : rhs.properties) {
    node[name] = property_sampler;
  }
  return node;
}

template <typename T>
struct convert<BehaviorSampler<T>> {
  static Node encode(const BehaviorSampler<T>& rhs) {
    Node node = encode_sr<T>(rhs);
    // std::cout << "W" << rhs.type << std::endl;
    if (rhs.optimal_speed) {
      node["optimal_speed"] = rhs.optimal_speed;
    }
    if (rhs.optimal_angular_speed) {
      node["optimal_angular_speed"] = rhs.optimal_angular_speed;
    }
    if (rhs.rotation_tau) {
      node["rotation_tau"] = rhs.rotation_tau;
    }
    if (rhs.safety_margin) {
      node["safety_margin"] = rhs.safety_margin;
    }
    if (rhs.horizon) {
      node["horizon"] = rhs.horizon;
    }
    return node;
  }
  static bool decode(const Node& node, BehaviorSampler<T>& rhs) {
    bool r = decode_sr<T>(node, &rhs);
    if (!r) return false;
    if (node["optimal_speed"]) {
      rhs.optimal_speed = read_sampler<float>(node["optimal_speed"]);
    }
    if (node["optimal_angular_speed"]) {
      rhs.optimal_angular_speed =
          read_sampler<float>(node["optimal_angular_speed"]);
    }
    if (node["rotation_tau"]) {
      rhs.rotation_tau = read_sampler<float>(node["rotation_tau"]);
    }
    if (node["safety_margin"]) {
      rhs.safety_margin = read_sampler<float>(node["safety_margin"]);
    }
    if (node["horizon"]) {
      rhs.horizon = read_sampler<float>(node["horizon"]);
    }
    // std::cout << "R" << rhs.type << std::endl;
    return true;
  }
};

template <typename T>
struct convert<KinematicSampler<T>> {
  static Node encode(const KinematicSampler<T>& rhs) {
    Node node = encode_sr<T>(rhs);
    if (rhs.max_speed) {
      node["max_speed"] = rhs.max_speed;
    }
    if (rhs.max_angular_speed) {
      node["max_angular_speed"] = rhs.max_angular_speed;
    }
    return node;
  }
  static bool decode(const Node& node, KinematicSampler<T>& rhs) {
    bool r = decode_sr<T>(node, &rhs);
    if (!r) return false;
    if (node["max_speed"]) {
      rhs.max_speed = read_sampler<float>(node["max_speed"]);
    }
    if (node["max_angular_speed"]) {
      rhs.max_angular_speed = read_sampler<float>(node["max_angular_speed"]);
    }
    return true;
  }
};

template <typename T>
struct convert<SamplerFromRegister<T>> {
  static Node encode(const SamplerFromRegister<T>& rhs) {
    return encode_sr<T>(rhs);
  }
  static bool decode(const Node& node, SamplerFromRegister<T>& rhs) {
    bool r = decode_sr<T>(node, &rhs);
    return r;
  }
};

template <typename W>
struct convert<AgentSampler<W>> {
  using B = typename W::A::B;
  using K = typename W::A::K;
  using T = typename W::A::T;
  using S = typename W::A::S;

  static Node encode(const AgentSampler<W>& rhs) {
    Node node;
    node["navigation_behavior"] = rhs.behavior;
    node["kinematic"] = rhs.kinematic;
    node["task"] = rhs.task;
    node["state_estimation"] = rhs.state_estimation;
    if (rhs.x) {
      node["x"] = rhs.x;
    }
    if (rhs.y) {
      node["y"] = rhs.y;
    }
    if (rhs.theta) {
      node["theta"] = rhs.theta;
    }
    if (rhs.radius) {
      node["radius"] = rhs.radius;
    }
    if (rhs.control_period) {
      node["control_period"] = rhs.control_period;
    }
    if (rhs.number) {
      node["number"] = rhs.number;
    }
    return node;
  }
  static bool decode(const Node& node, AgentSampler<W>& rhs) {
    if (!node.IsMap()) {
      return false;
    }
    if (node["navigation_behavior"]) {
      rhs.behavior = node["navigation_behavior"].as<BehaviorSampler<B>>();
    }
    if (node["kinematic"]) {
      rhs.kinematic = node["kinematic"].as<KinematicSampler<K>>();
    }
    if (node["task"]) {
      rhs.task = node["task"].as<TaskSampler<T>>();
    }
    if (node["state_estimation"]) {
      rhs.state_estimation =
          node["state_estimation"].as<StateEstimationSampler<S>>();
    }
    if (node["x"]) {
      rhs.x = read_sampler<float>(node["x"]);
    }
    if (node["y"]) {
      rhs.y = read_sampler<float>(node["y"]);
    }
    if (node["theta"]) {
      rhs.theta = read_sampler<float>(node["theta"]);
    }
    if (node["radius"]) {
      rhs.radius = read_sampler<float>(node["radius"]);
    }
    if (node["control_period"]) {
      rhs.control_period = read_sampler<float>(node["control_period"]);
    }
    if (node["number"]) {
      rhs.number = node["number"].as<unsigned>(0);
    }
    return true;
  }
};

}  // namespace YAML

#endif  // HL_NAVIGATION_SIM_YAML_SAMPLER_H
