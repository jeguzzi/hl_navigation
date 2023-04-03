#ifndef HL_NAVIGATION_SIM_YAML_WORLD_H
#define HL_NAVIGATION_SIM_YAML_WORLD_H

#include <memory>

#include "hl_navigation/yaml/core.h"
#include "hl_navigation/yaml/property.h"
#include "hl_navigation/yaml/register.h"
#include "hl_navigation_sim/world.h"
#include "yaml-cpp/yaml.h"

using hl_navigation::Behavior;
using hl_navigation::Disc;
using hl_navigation::Kinematic;
using hl_navigation::LineSegment;
using hl_navigation::Vector2;
using hl_navigation_sim::Agent;
using hl_navigation_sim::StateEstimation;
using hl_navigation_sim::Task;
using hl_navigation_sim::World;
using hl_navigation_sim::Obstacle;
using hl_navigation_sim::Wall;

namespace YAML {

template <>
struct convert<Wall> {
  static Node encode(const Wall& rhs) {
    return convert<LineSegment>::encode(rhs.line);
  }
  static bool decode(const Node& node, Wall& rhs) {
    return convert<LineSegment>::decode(node, rhs.line);
  }
};

template <>
struct convert<Obstacle> {
  static Node encode(const Obstacle& rhs) {
    return convert<Disc>::encode(rhs.disc);
  }
  static bool decode(const Node& node, Obstacle& rhs) {
    return convert<Disc>::decode(node, rhs.disc);
  }
};


template <>
struct convert<Task> {
  static Node encode(const Task& rhs) {
    Node node;
    encode_type_and_properties<Task>(node, rhs);
    return node;
  }
  static bool decode(const Node& node, Task& rhs) {
    decode_properties(node, rhs);
    return true;
  }
};

template <>
struct convert<std::shared_ptr<Task>> {
  static Node encode(const std::shared_ptr<Task>& rhs) {
    return convert<Task>::encode(*rhs);
  }
  static bool decode(const Node& node, std::shared_ptr<Task>& rhs) {
    rhs = make_type_from_yaml<Task>(node);
    if (rhs) {
      convert<Task>::decode(node, *rhs);
    }
    return true;
  }
};

template <>
struct convert<StateEstimation> {
  static Node encode(const StateEstimation& rhs) {
    Node node;
    encode_type_and_properties<StateEstimation>(node, rhs);
    return node;
  }
  static bool decode(const Node& node, StateEstimation& rhs) {
    decode_properties(node, rhs);
    return true;
  }
};

template <>
struct convert<std::shared_ptr<StateEstimation>> {
  static Node encode(const std::shared_ptr<StateEstimation>& rhs) {
    return convert<StateEstimation>::encode(*rhs);
  }
  static bool decode(const Node& node, std::shared_ptr<StateEstimation>& rhs) {
    rhs = make_type_from_yaml<StateEstimation>(node);
    if (rhs) {
      convert<StateEstimation>::decode(node, *rhs);
    }
    return true;
  }
};

template <>
struct convert<Agent> {
  static Node encode(const Agent& rhs) {
    Node node;
    if (rhs.nav_behavior) {
      node["navigation_behavior"] = *(rhs.nav_behavior);
    }
    if (rhs.kinematic) {
      node["kinematic"] = *(rhs.kinematic);
    }
    if (rhs.task) {
      node["task"] = *(rhs.task);
    }
    if (rhs.state_estimation) {
      node["state_estimation"] = *(rhs.state_estimation);
    }
    node["position"] = rhs.pose.position;
    node["orientation"] = rhs.pose.orientation;
    node["velocity"] = rhs.twist.velocity;
    node["angular_speed"] = rhs.twist.angular_speed;
    node["radius"] = rhs.radius;
    node["control_period"] = rhs.control_period;
    node["type"] = rhs.type;
    return node;
  }
  static bool decode(const Node& node, Agent& rhs) {
    if (!node.IsMap()) {
      return false;
    }
    if (node["navigation_behavior"]) {
      rhs.nav_behavior =
          node["navigation_behavior"].as<std::shared_ptr<Behavior>>();
    }
    if (node["kinematic"]) {
      rhs.kinematic = node["kinematic"].as<std::shared_ptr<Kinematic>>();
    }
    if (node["task"]) {
      rhs.task = node["task"].as<std::shared_ptr<Task>>();
    }
    if (node["state_estimation"]) {
      rhs.state_estimation =
          node["state_estimation"].as<std::shared_ptr<StateEstimation>>();
    }
    if (node["position"]) {
      rhs.pose.position = node["position"].as<Vector2>();
    }
    if (node["orientation"]) {
      rhs.pose.orientation = node["orientation"].as<float>();
    }
    if (node["velocity"]) {
      rhs.twist.velocity = node["velocity"].as<Vector2>();
    }
    if (node["angular_speed"]) {
      rhs.twist.angular_speed = node["angular_speed"].as<float>();
    }
    if (node["radius"]) {
      rhs.radius = node["radius"].as<float>();
    }
    if (node["control_period"]) {
      rhs.control_period = node["control_period"].as<float>();
    }
    if (node["type"]) {
      rhs.type = node["type"].as<std::string>();
    }
    return true;
  }
};

template <>
struct convert<std::shared_ptr<Agent>> {
  static Node encode(const std::shared_ptr<Agent>& rhs) {
    if (rhs) {
      return convert<Agent>::encode(*rhs);
    }
    return Node();
  }
  static bool decode(const Node& node, std::shared_ptr<Agent>& rhs) {
    rhs = std::make_shared<Agent>();
    if (convert<Agent>::decode(node, *rhs)) {
      return true;
    }
    rhs = nullptr;
    return false;
  }
};

template <typename T = Agent>
struct convert_world {
  static Node encode(const World& rhs) {
    Node node;
    node["obstacles"] = rhs.obstacles;
    node["walls"] = rhs.walls;
    node["agents"] = rhs.agents;
    return node;
  }
  static bool decode(const Node& node, World& rhs) {
    if (!node.IsMap()) {
      return false;
    }
    if (node["agents"]) {
      if (node["agents"].IsSequence()) {
        for (const auto& c : node["agents"]) {
          // TODO
          rhs.agents.push_back(c.as<std::shared_ptr<T>>());
        }
      }
    }
    if (node["obstacles"]) {
      for (const auto& c : node["obstacles"]) {
        rhs.add_obstacle(c.as<Disc>());
      }
    }
    if (node["walls"]) {
      for (const auto& c : node["walls"]) {
        rhs.add_wall(c.as<LineSegment>());
      }
    }
    return true;
  }
};

template <>
struct convert<World> {
  static Node encode(const World& rhs) { return convert_world<>::encode(rhs); }
  static bool decode(const Node& node, World& rhs) {
    return convert_world<>::decode(node, rhs);
  }
};

}  // namespace YAML

#endif  // HL_NAVIGATION_SIM_YAML_WORLD_H
