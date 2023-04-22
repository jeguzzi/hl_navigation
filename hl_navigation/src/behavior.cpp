/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "hl_navigation/behavior.h"
#include <iostream>
#include <stdexcept>

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
      delta_angle = orientation_of(absolute_velocity) - pose.orientation;
      break;
    case Heading::target_angle:
      delta_angle = target_pose.orientation - pose.orientation;
      break;
    case Heading::target_point:
      delta_angle =
          orientation_of(target_pose.position - pose.position) - pose.orientation;
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
  desired_velocity = compute_desired_velocity(dt);
  Twist2 desired_twist = twist_towards_velocity(desired_velocity, true);
  Twist2 twist = kinematics->feasible(desired_twist);
  return to_frame(twist, relative);
}

Twist2 Behavior::cmd_twist_towards_target_orientation([[maybe_unused]] float dt, bool relative) {
  return {Vector2::Zero(),
          std::clamp(normalize(target_pose.orientation - pose.orientation),
                     -kinematics->get_max_angular_speed(),
                     kinematics->get_max_angular_speed()),
          relative};
}

Twist2 Behavior::cmd_twist_towards_stopping([[maybe_unused]] float dt, bool relative) {
  return {Vector2::Zero(), 0.0, relative};
}

Twist2 Behavior::cmd_twist(float dt, Mode mode, bool relative,
                           bool set_as_actuated) {
  if (!kinematics) {
    std::cerr << "Missing kinematics" << std::endl;
    return {};
  }
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
      // TODO(Jerome): Placeholder
      target_pose.position = pose.position + target_twist.velocity * 1e3;
      twist = cmd_twist_towards_target(dt, relative);
      //throw std::runtime_error("Not implemented yet.");
      break;
  }
  if (set_as_actuated) {
    actuated_twist = twist;
  }
  return twist;
}

}  // namespace hl_navigation
