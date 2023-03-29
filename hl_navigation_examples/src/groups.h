/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_EXAMPLES_GROUP_H
#define HL_NAVIGATION_EXAMPLES_GROUP_H

#include "hl_navigation_sim/sampling/agent.h"
#include "hl_navigation_sim/sampling/sampler.h"

namespace sim = hl_navigation_sim;
namespace nav = hl_navigation;
using ConstProperty = sim::Constant<nav::Property::Field>;


/*
 * Equivalent YAML:
 *  
 *  number: 1
 *  kinematic:
 *    type: Holonomic
 *    max_speed: 1.0
 *  navigation_behavior:
 *    type: Dummy
 *  task:
 *    type: WayPoints
 *    waypoints: [[1.0, 0.0]]
 *    tolerance: 0.1
 *  radius: 0.1
 *  control_period: 0.1
 */

inline sim::AgentSampler<sim::World> agents() {
  sim::AgentSampler<sim::World> group;
  group.number = 1;
  group.behavior = sim::BehaviorSampler<>("Dummy");
  group.kinematic = sim::KinematicSampler<>("Holonomic");
  group.kinematic.max_speed = std::make_unique<sim::Constant<float>>(1.0f);
  group.task = sim::TaskSampler<>("WayPointsTask");
  group.task.properties["waypoints"] =
      std::make_unique<ConstProperty>(sim::Waypoints{{1.0f, 0.0f}});
  group.radius = std::make_unique<sim::Constant<float>>(0.1f);
  group.control_period = std::make_unique<sim::Constant<float>>(0.1f);
  return group;
}

/*
 * Equivalent YAML:
 * 
 *  number: 2
 *  kinematic:
 *    type: TwoWheeled
 *    max_speed: 1.0
 *    wheel_axis: 0.12
 *  navigation_behavior:
 *    type: HL
 *  state_estimation:
 *    type: Bounded
 *    range_of_view: 10.0
 *  task:
 *    type: WayPoints
 *    waypoints: [[1.0, 0.0], [-1.0, 0.0]]
 *    tolerance: 0.1
 *    loop: true
 *  x:
 *    sampler: regular
 *    start: 0
 *    end: 10
 *    number: 2
 *  y: 0
 *  theta: 0
 *  control_period: 0.1
 */


inline sim::AgentSampler<sim::World> robots() {
  sim::AgentSampler<sim::World> group;
  group.number = 2;
  group.behavior = sim::BehaviorSampler<>("HL");
  group.behavior.safety_margin = std::make_unique<sim::Constant<float>>(0.5f);
  group.behavior.properties["tau"] = std::make_unique<ConstProperty>(0.25f);
  group.kinematic = sim::KinematicSampler<>("TwoWheeled");
  group.kinematic.max_speed = std::make_unique<sim::Constant<float>>(1.0f);
  group.kinematic.properties["wheel_axis"] =
      std::make_unique<ConstProperty>(0.12f);
  group.state_estimation = sim::StateEstimationSampler<>("BoundedStateEstimation");
  group.state_estimation.properties["range_of_view"] =
      std::make_unique<ConstProperty>(10.0f);
  group.task = sim::TaskSampler<>("WayPointsTask");
  group.task.properties["loop"] = std::make_unique<ConstProperty>(true);
  group.task.properties["waypoints"] =
      std::make_unique<ConstProperty>(sim::Waypoints{{1.0f, 0.0f}, {-1.0f, 0.0f}});
  group.radius = std::make_unique<sim::Constant<float>>(0.1f);
  group.x = std::make_unique<sim::Regular<float>>(0.0f, 10.0f, group.number);
  group.y = std::make_unique<sim::Constant<float>>(0.0f);
  group.theta = std::make_unique<sim::Constant<float>>(0.0f);
  group.control_period = std::make_unique<sim::Constant<float>>(0.1f);
  return group;
}


#endif // HL_NAVIGATION_EXAMPLES_GROUP_H