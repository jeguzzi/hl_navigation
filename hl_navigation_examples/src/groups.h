/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_EXAMPLES_GROUP_H
#define HL_NAVIGATION_EXAMPLES_GROUP_H

#include "hl_navigation_sim/sampling/agent.h"
#include "hl_navigation_sim/sampling/sampler.h"
#include "hl_navigation_sim/tasks/waypoints.h"

namespace sim = hl_navigation_sim;
namespace nav = hl_navigation;

template <typename T>
std::unique_ptr<sim::PropertySampler> make_const_property(const T& value) {
  return std::make_unique<sim::PropertySampler>(sim::ConstantSampler<T>(value));
}

template <typename T>
std::unique_ptr<sim::Sampler<T>> make_const(const T& value) {
  return std::make_unique<sim::ConstantSampler<T>>(value);
}

/*
 * Equivalent YAML:
 *
 *  number: 1
 *  kinematics:
 *    type: Holonomic
 *    max_speed: 1.0
 *  behavior:
 *    type: Dummy
 *  task:
 *    type: Waypoints
 *    waypoints: [[1.0, 0.0]]
 *    tolerance: 0.1
 *  radius: 0.1
 *  control_period: 0.1
 */

inline sim::AgentSampler<sim::World> agents() {
  sim::AgentSampler<sim::World> group;
  group.name = "agents";
  group.number = 1;
  group.behavior = sim::BehaviorSampler<>("Dummy");
  group.kinematics = sim::KinematicsSampler<>("Holonomic");
  group.kinematics.max_speed = make_const<float>(1.0f);
  group.task = sim::TaskSampler<>("WaypointsTask");
  group.task.properties["waypoints"] =
      make_const_property<sim::Waypoints>({{1.0f, 0.0f}});
  group.radius = make_const<float>(0.1f);
  group.control_period = make_const<float>(0.1f);
  return group;
}

/*
 * Equivalent YAML:
 *
 *  number: 2
 *  kinematics:
 *    type: TwoWheeled
 *    max_speed: 1.0
 *    wheel_axis: 0.12
 *  behavior:
 *    type: HL
 *  state_estimation:
 *    type: Bounded
 *    range_of_view: 10.0
 *  task:
 *    type: Waypoints
 *    waypoints: [[1.0, 0.0], [-1.0, 0.0]]
 *    tolerance: 0.1
 *    loop: true
 *  x:
 *    sampler: regular
 *    from: 0
 *    to: 10
 *    number: 2
 *  y: 0
 *  theta: 0
 *  control_period: 0.1
 */

inline sim::AgentSampler<sim::World> robots() {
  sim::AgentSampler<sim::World> group;
  group.name = "robots";
  group.id = make_const(1);
  group.number = 2;
  group.behavior = sim::BehaviorSampler<>("HL");
  group.behavior.safety_margin = make_const<float>(0.5f);
  group.behavior.properties["tau"] = make_const_property<float>(0.25f);
  group.kinematics = sim::KinematicsSampler<>("TwoWheeled");
  group.kinematics.max_speed = make_const<float>(1.0f);
  group.kinematics.properties["wheel_axis"] = make_const_property<float>(0.12f);
  group.state_estimation =
      sim::StateEstimationSampler<>("BoundedStateEstimation");
  group.state_estimation.properties["range_of_view"] =
      make_const_property<float>(10.0f);
  group.task = sim::TaskSampler<>("WaypointsTask");
  group.task.properties["loop"] = make_const_property<bool>(true);
  group.task.properties["waypoints"] =
      make_const_property<sim::Waypoints>({{1.0f, 0.0f}, {-1.0f, 0.0f}});
  group.radius = make_const<float>(0.1f);
  group.position = std::make_unique<sim::RegularSampler<Vector2>>(
      sim::RegularSampler<Vector2>::make_with_interval(
          {0.0f, 0.0f}, {10.0f, 0.0f}, group.number));
  group.orientation = make_const<float>(0.0f);
  group.control_period = make_const<float>(0.1f);
  return group;
}

#endif  // HL_NAVIGATION_EXAMPLES_GROUP_H