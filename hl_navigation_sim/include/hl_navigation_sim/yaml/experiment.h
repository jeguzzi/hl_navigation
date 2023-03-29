#ifndef HL_NAVIGATION_SIM_YAML_EXPERIMENT_H
#define HL_NAVIGATION_SIM_YAML_EXPERIMENT_H

#include <iostream>
#include <memory>

#include "hl_navigation/yaml/yaml.h"
#include "hl_navigation_sim/experiment.h"
#include "hl_navigation_sim/scenario.h"
#include "hl_navigation_sim/yaml/scenario.h"
#include "yaml-cpp/yaml.h"

using hl_navigation_sim::Experiment;
using hl_navigation_sim::Scenario;

namespace YAML {

struct convert_experiment {
  static Node encode(const Experiment& rhs) {
    Node node;
    node["time_step"] = rhs.time_step;
    node["steps"] = rhs.steps;
    node["runs"] = rhs.runs;
    node["save_directory"] = rhs.save_directory.string();
    node["record_pose"] = rhs.trace.record_pose;
    node["record_twist"] = rhs.trace.record_twist;
    node["record_cmd"] = rhs.trace.record_cmd;
    node["record_target"] = rhs.trace.record_target;
    node["name"] = rhs.name;
    return node;
  }
  static bool decode(const Node& node, Experiment& rhs) {
    if (!node.IsMap()) {
      return false;
    }
    if (node["time_step"]) {
      rhs.time_step = node["time_step"].as<float>(0.1f);
    }
    if (node["steps"]) {
      rhs.steps = node["steps"].as<unsigned>(0);
    }
    if (node["runs"]) {
      rhs.runs = node["runs"].as<unsigned>(1);
    }
    if (node["save_directory"]) {
      rhs.save_directory = node["save_directory"].as<std::string>("");
    }
    if (node["record_pose"]) {
      rhs.trace.record_pose = node["record_pose"].as<bool>();
    }
    if (node["record_twist"]) {
      rhs.trace.record_twist = node["record_twist"].as<bool>();
    }
    if (node["record_cmd"]) {
      rhs.trace.record_cmd = node["record_cmd"].as<bool>();
    }
    if (node["record_target"]) {
      rhs.trace.record_target = node["record_target"].as<bool>();
    }
    if (node["name"]) {
      rhs.name = node["name"].as<std::string>();
    }
    return true;
  }
};

template <>
struct convert<Experiment> {
  static Node encode(const Experiment& rhs) {
    Node node = convert_experiment::encode(rhs);
    if (rhs.scenario) {
      node["world"] = *(rhs.scenario);
    }
    return node;
  }
  static bool decode(const Node& node, Experiment& rhs) {
    if (convert_experiment::decode(node, rhs)) {
      if (node["world"]) {
        rhs.scenario = node["world"].as<std::shared_ptr<Scenario>>();
      }
      return true;
    }
    return false;
  }
};

}  // namespace YAML

#endif  // HL_NAVIGATION_SIM_YAML_EXPERIMENT_H
