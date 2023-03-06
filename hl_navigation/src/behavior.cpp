/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "hl_navigation/behavior.h"
#include <iostream>

namespace hl_navigation {

// TODO(J 2023): review
float obstacle_margin(float distance, float radius, float obstacle_radius,
                      float safety_margin, float social_margin) {
  // social_margin is now a property of the obstacle!!, so could be less than
  // safety margin
  social_margin = std::max(social_margin, safety_margin);

  float far_margin = 1.0f;
  float distance_to_be_separated = safety_margin + radius + obstacle_radius;
  float distance_to_be_far = far_margin + radius + obstacle_radius;

  if (distance < distance_to_be_separated) {
    return safety_margin;
  } else if (distance > distance_to_be_far) {
    return social_margin;
  } else {
    return (social_margin - safety_margin) /
               (distance_to_be_far - distance_to_be_separated) *
               (distance - distance_to_be_separated) +
           safety_margin;
  }
}

// TODO(J 2023): review
Vector2 obstacle_relative_position(const Vector2& position,
                                   Vector2& obstacle_position, float radius,
                                   float obstacle_radius, float& distance) {
  Vector2 relative_position = obstacle_position - position;
  distance = relative_position.norm();
  float min_distance = (radius + obstacle_radius) + 0.002;
  if (distance < min_distance) {
    // too near,  cannot penetrate in an obstacle (footbot or human)
    relative_position = relative_position / distance * min_distance;
    obstacle_position = position + relative_position;
    distance = min_distance;
  }
  return relative_position;
}

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
