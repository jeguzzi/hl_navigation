/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "hl_navigation/behavior.h"
#include <iostream>

namespace hl_navigation {

Twist2 Behavior::twist_towards_velocity(const Vector2& absolute_velocity,
                                        bool relative) {
  float delta_angle = 0.0f;
  Twist2 twist;
  if (relative) {
    twist.velocity = to_relative(absolute_velocity);
    twist.relative = true;
  } else {
    twist.velocity = absolute_velocity;
    twist.relative = false;
  }
  switch (get_heading_behavior()) {
    case Heading::velocity:
      delta_angle = polar_angle(absolute_velocity) - pose.orientation;
      break;
    case Heading::target_angle:
      delta_angle = target_pose.orientation - pose.orientation;
      break;
    case Heading::target_point:
      delta_angle =
          polar_angle(target_pose.position - pose.position) - pose.orientation;
      break;
    default:
      delta_angle = 0.0;
      break;
  }
  twist.angular_speed =
      std::clamp(normalize(delta_angle) / rotation_tau, -optimal_angular_speed,
                 optimal_angular_speed);
  return twist;
}



Twist2 Behavior::cmd_twist_towards_target([[maybe_unused]] float dt, bool relative) {
  desired_velocity = compute_desired_velocity();
  Twist2 desired_twist = twist_towards_velocity(desired_velocity, true);
  Twist2 twist = kinematic->feasible(desired_twist);
  return to_frame(twist, relative);
}

Twist2 Behavior::cmd_twist_towards_target_orientation([[maybe_unused]] float dt, bool relative) {
  return {Vector2::Zero(),
          std::clamp(normalize(target_pose.orientation - pose.orientation),
                     -kinematic->get_max_angular_speed(),
                     kinematic->get_max_angular_speed()),
          relative};
}

Twist2 Behavior::cmd_twist_towards_stopping([[maybe_unused]] float dt, bool relative) {
  return {Vector2::Zero(), 0.0, relative};
}

Twist2 Behavior::cmd_twist(float dt, bool relative, Mode mode,
                           bool set_as_actuated) {
  Twist2 twist;
  switch (mode) {
    case Mode::move:
      twist = cmd_twist_towards_target(dt, relative);
      break;
    case Mode::turn:
      twist = cmd_twist_towards_target_orientation(dt, relative);
      break;
    case Mode::stop:
      twist = cmd_twist_towards_stopping(dt, relative);
      break;
    case Mode::follow:
      throw "Not implemented yet.";
      break;
  }
  if (set_as_actuated) {
    actuated_twist = twist;
  }
  return twist;
}

std::map<std::string, Behavior::BehaviorFactory> Behavior::factory = {};

}  // namespace hl_navigation
