/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "hl_navigation/behaviors/HL.h"

#include <assert.h>

#include <algorithm>

#include "hl_navigation/collision_computation.h"

static float relax(float x0, float x1, float tau, float dt) {
  if (tau == 0) return x1;
  // float dt=0.1;
  return exp(-dt / tau) * (x0 - x1) + x1;
}

static std::vector<float> relax(const std::vector<float> &v0,
                                const std::vector<float> &v1, float tau,
                                float dt) {
  if (tau == 0) return v1;
  auto v2 = std::vector<float>(v0.size());
  for (size_t i = 0; i < v0.size(); i++) {
    v2[i] = relax(v0[i], v1[i], tau, dt);
  }
  return v2;
}

static hl_navigation::Twist2 relax(const hl_navigation::Twist2 &v0,
                                   const hl_navigation::Twist2 &v1, float tau,
                                   float dt) {
  assert(v1.relative == v0.relative);
  if (tau == 0) {
    return v1;
  }
  return {{relax(v0.velocity[0], v1.velocity[0], tau, dt),
           relax(v0.velocity[1], v1.velocity[1], tau, dt)},
          relax(v0.angular_speed, v1.angular_speed, tau, dt),
          v1.relative};
}

namespace hl_navigation {

// TODO(J 2023): review that we modify the obstacle position
static DiscCache make_obstacle_cache(const Vector2 &position,
                                     Vector2 obstacle_position,
                                     const Vector2 &obstacle_velocity,
                                     float radius, float obstacle_radius,
                                     float safety_margin, float social_margin) {
  float distance;
  Vector2 relative_position = obstacle_relative_position(
      position, obstacle_position, radius, obstacle_radius, distance);
  float margin = radius + obstacle_radius +
                 obstacle_margin(distance, radius, obstacle_radius,
                                 safety_margin, social_margin);
  return {relative_position, margin, obstacle_velocity};
}

// TODO(J 2023): review that we modify the obstacle position
static DiscCache make_obstacle_cache(const Vector2 &position,
                                     Vector2 obstacle_position, float radius,
                                     float obstacle_radius, float safety_margin,
                                     [[maybe_unused]] float social_margin) {
  float distance;
  Vector2 relative_position = obstacle_relative_position(
      position, obstacle_position, radius, obstacle_radius, distance);
  float margin = safety_margin + radius + obstacle_radius;
  return {relative_position, margin};
}

HLBehavior::~HLBehavior() = default;

CollisionComputation::CollisionMap HLBehavior::get_collision_distance(
    bool assuming_static) {
  prepare();
  return collision_computation.get_free_distance_for_sector(
      pose.orientation - aperture, 2 * aperture, resolution, effective_horizon,
      assuming_static, optimal_speed);
}

static inline float distance_from_target(Radians angle, float free_distance,
                                         float horizon) {
  if (cos(angle) * horizon < free_distance) {
    return fabs(sin(angle) * horizon);
  }
  return sqrt(horizon * horizon + free_distance * free_distance -
              2 * free_distance * horizon * cos(angle));
}

// TODO(J:revision2023): check why we need effective_horizon
// output is in absolute frame
Vector2 HLBehavior::compute_desired_velocity() {
  prepare();
  const Vector2 delta_target = target_pose.position - pose.position;
  const Radians start_angle = polar_angle(delta_target);
  const Radians relative_start_angle = start_angle - pose.orientation;
  const Radians da = get_angular_resolution();
  // float max_distance = agentToTarget.norm();
  // effective_horizon = horizon;
  // effective_horizon=fmin(horizon,D);
  // Vector2 effectiveTarget = agentToTarget / D * effective_horizon;
  const float max_distance = effective_horizon - radius;
  const Radians max_angle{1.6f};  // Radians::PI_OVER_TW0;
  Radians angle = 0.0f;
  // relative to target direction
  Radians optimal_angle = 0.0f;
  float optimal_distance_from_target = max_distance;
  int out[2] = {0, 0};
  bool found = false;
  while (angle < max_angle && !(out[0] == 2 && out[1] == 2)) {
    float distance_from_target_lower_bound = fabs(sin(angle) * max_distance);
    // distance_from_target_lower_bound=2*D*Sin(0.5*optimal_angle);
    if (optimal_distance_from_target <= distance_from_target_lower_bound) {
      break;
    }
    for (int i = 0; i < 2; ++i) {
      const float s_angle = i == 0 ? angle : -angle;
      const bool inside =
          abs(normalize(relative_start_angle + s_angle)) < aperture;
      if (out[i] == 1 && !inside) out[i] = 2;
      if (out[i] == 0 && inside) out[i] = 1;
      if (inside) {
        const float free_distance = collision_computation.dynamic_free_distance(
            start_angle + s_angle, max_distance, optimal_speed);
        const float dist =
            distance_from_target(angle, free_distance, max_distance);
        if (dist < optimal_distance_from_target) {
          optimal_distance_from_target = dist;
          optimal_angle = s_angle;
          found = true;
        }
      }
      if (!angle) break;
    }
    angle += da;
  }
  if (!found) return Vector2::Zero();
  const float static_distance = collision_computation.static_free_distance(
      start_angle + optimal_angle, max_distance);
  const float desired_speed = fmin(optimal_speed, static_distance / eta);
  return desired_speed * unit(start_angle + optimal_angle);
}

void HLBehavior::prepare() {
  effective_horizon = horizon;
  if (GeometricState::changed() ||
      Behavior::changed(POSITION | ORIENTATION | RADIUS | HORIZON |
                        SAFETY_MARGIN)) {
    std::vector<DiscCache> ns;
    ns.reserve(get_neighbors().size());
    for (const Disc &d : get_neighbors()) {
      const auto c =
          make_obstacle_cache(pose.position, d.position, d.velocity, radius,
                              d.radius, safety_margin, d.social_margin);
      if (collision_computation.dynamic_may_collide(c, effective_horizon,
                                                    optimal_speed)) {
        ns.push_back(c);
      }
    }
    std::vector<DiscCache> ss;
    ss.reserve(get_static_obstacles().size());

    for (const Disc &d : get_static_obstacles()) {
      const auto c =
          make_obstacle_cache(pose.position, d.position, radius, d.radius,
                              safety_margin, d.social_margin);
      if (collision_computation.static_may_collide(c, effective_horizon)) {
        ns.push_back(c);
      }
    }
    collision_computation.setup(pose, radius + safety_margin,
                                get_line_obstacles(), std::move(ss),
                                std::move(ns));
  }
  GeometricState::reset_changes();
  Behavior::reset_changes();
}

Twist2 HLBehavior::relax(const Twist2 &value, float dt) const {
  if (kinematic->is_wheeled()) {
    auto wheel_speeds = wheel_speeds_from_twist(value);
    auto actuated_wheel_speeds = wheel_speeds_from_twist(actuated_twist);
    return twist_from_wheel_speeds(
        ::relax(actuated_wheel_speeds, wheel_speeds, tau, dt));
  } else {
    // TODO(Jerome old): same than before when I relaxed the absolute velocity,
    // not the relative but different than original paper CHANGED(J 2023): relax
    // in arbitrary frame
    return ::relax(actuated_twist.relative == value.relative
                       ? actuated_twist
                       : to_frame(actuated_twist, value.relative),
                   value, tau, dt);
  }
}

Twist2 HLBehavior::cmd_twist(float dt, bool relative, Mode mode,
                             bool set_as_actuated) {
  Twist2 twist = Behavior::cmd_twist(dt, relative, mode, false);
  if (tau > 0) {
    twist = relax(twist, dt);
  }
  if (set_as_actuated) {
    actuated_twist = twist;
  }
  return twist;
}

const char *HLBehavior::name = register_type<HLBehavior>("HL");

}  // namespace hl_navigation
