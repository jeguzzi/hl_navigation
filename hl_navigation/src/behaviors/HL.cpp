/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "hl_navigation/behaviors/HL.h"

#include <assert.h>

#include <algorithm>

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

static void set_dx(DiscCache &agent, const Vector2 &p) {
  agent.dx = p;
  agent.center_distance = p.norm();

  // visibleDistance=center_distance-agent_sensing_margin;
  // penetration=(social_margin-center_distance);

  agent.C = p.squaredNorm() - agent.sensing_margin * agent.sensing_margin;
  agent.gamma = polar_angle(-p);
  agent.visible_angle = M_PI_2;
  // Radians alpha=ASin(agent.agent_sensing_margin/agent.center_distance);
  // agent.beta1=agent.gamma-alpha;
  // agent.beta2=agent.gamma+alpha;
}

// TODO(J 2023): review that we modify the obstacle position
static DiscCache make_obstacle_cache(const Vector2 &position,
                                     Vector2 &obstacle_position,
                                     const Vector2 &obstacle_velocity,
                                     float radius, float obstacle_radius,
                                     float safety_margin, float social_margin) {
  /// TODO(J old): eliminare radius (non lo uso)
  DiscCache agent;
  agent.radius = radius;
  float distance;
  Vector2 relative_position = obstacle_relative_position(
      position, obstacle_position, radius, obstacle_radius, distance);
  agent.sensing_margin = radius + obstacle_radius +
                         obstacle_margin(distance, radius, obstacle_radius,
                                         safety_margin, social_margin);
  agent.va = obstacle_velocity;
  set_dx(agent, -relative_position);
  agent.position = obstacle_position;
  return agent;
}

// TODO(J 2023): review that we modify the obstacle position
static DiscCache make_obstacle_cache(const Vector2 &position,
                                     Vector2 &obstacle_position, float radius,
                                     float obstacle_radius, float safety_margin,
                                     float social_margin) {
  DiscCache obstacle;
  obstacle.radius = obstacle_radius;
  float distance;
  Vector2 relative_position = obstacle_relative_position(
      position, obstacle_position, radius, obstacle_radius, distance);
  // obstacle.agent_sensing_margin = obstacle_radius;
  obstacle.sensing_margin = safety_margin + radius + obstacle_radius;
  obstacle.position = obstacle_position;
  set_dx(obstacle, -relative_position);
  return obstacle;
}

static float penetration(const DiscCache *agent) {
  if (agent->C > 0) {
    return 0;
  } else {
    return agent->center_distance - agent->sensing_margin;
  }
}

HLBehavior::~HLBehavior() = default;

void HLBehavior::dump_neighbors_cache() const {
  printf("--------------------- NEIGHBORS -------------------\n");
  for (const auto &neighbor : neighbors_cache) {
    printf("dx=(%.2f,%.2f), va=(%.2f, %.2f), C=%.2f\r\n", neighbor.dx.x(),
           neighbor.dx.y(), neighbor.va.x(), neighbor.va.y(), neighbor.C);
  }
  printf("--------------------- ********* -------------------\n");
}

void HLBehavior::dump_static_obstacles_cache() const {
  // if(!consoleDebugging) return;
  printf("----------------- Static Obstacles ----------------\n");
  printf("----------------- **************** ----------------\n");
  printf("static_obstacles_cache=[");
  for (const auto &obstacles : static_obstacles_cache) {
    printf("%.3f,%.3f,", obstacles.position.x(), obstacles.position.y());
  }
  printf("];\n");
  printf("----------------- **************** ----------------\n");
}

float HLBehavior::feared_distance_to_collision_at_relative_angle(
    Radians relative_angle) {
  if (abs(normalize(relative_angle)) >= aperture) return UNKNOWN_DIST;
  int k = index_of_relative_angle(relative_angle);
  return static_distance_cache[k];
}

unsigned int HLBehavior::index_of_relative_angle(Radians relative_angle) {
  int k = floor((normalize(relative_angle) + aperture) / (2 * aperture) *
                resolution);
  k = k % resolution;
  if (k < 0) k += resolution;
  return k;
}

float HLBehavior::distance_to_collision_at_relative_angle(
    Radians relative_angle) {
  if (abs(normalize(relative_angle)) >= aperture) return UNKNOWN_DIST;
  int k = index_of_relative_angle(relative_angle);
  if (distance_cache[k] < 0) {
    distance_cache[k] = compute_distance_to_collision_at_relative_angle(
        relative_angle, static_distance_cache + k);
  }
  return distance_cache[k];
}

float HLBehavior::distance_to_segment(const LineSegment &line,
                                      Radians absolute_angle) {
  Vector2 delta = pose.position - line.p1;
  float r = radius + safety_margin;
  float y = delta.dot(line.e2);
  float x = delta.dot(line.e1);
  Vector2 e = unit(absolute_angle);
  float d = line.e2.dot(e);
  if (y * d >= 0) {
    // moving away
    return NO_COLLISION;
  }
  if (abs(y) < r && x > -r && x < line.length + r) {
    // already colliding
    return 0.0;
  }
  float distance = -y / d - r;
  x = line.e1.dot(distance * e + delta);
  if (x < -r || x > line.length + r) {
    // will not collide
    return NO_COLLISION;
  }
  return distance;
}

// TODO(J:revision2023): check why we need effective_horizon

float HLBehavior::compute_distance_to_collision_at_relative_angle(
    Radians relative_angle, float *static_cache) {
  Radians vangle = relative_angle + pose.orientation;

  //!!! Change horizon -> effective_horizon !!!

  // HACK(J): disabled effective_horizon to test function in python
  effective_horizon = horizon;
  float min_distance = effective_horizon - radius;

  ////
  float distance;
  float static_distance;
  *static_cache = min_distance;

  for (const auto &segment : line_obstacles) {
    static_distance = distance_to_segment(segment, vangle);
    // printf("static_distance %.4f\n", static_distance);
    if (static_distance >= 0) {
      *static_cache = fmin(*static_cache, static_distance);
    }
    distance = static_distance;
    if (distance < 0) continue;
    min_distance = fmin(min_distance, distance);
    if (min_distance == 0) return 0;
  }

  for (const auto &obstacle : static_obstacles_cache) {
    static_distance = static_dist_for_angle(&obstacle, vangle);
    if (!(static_distance < 0))
      *static_cache = fmin(*static_cache, static_distance);

    distance = static_distance;

    if (distance < 0) continue;
    min_distance = fmin(min_distance, distance);

    if (min_distance == 0) return 0;
  }

  for (const auto &neighbor : neighbors_cache) {
    static_distance = static_dist_for_angle(&neighbor, vangle);
    if (!(static_distance < 0))
      *static_cache = fmin(*static_cache, static_distance);

    distance = dist_for_angle(&neighbor, vangle);

    if (distance < 0) continue;
    min_distance = fmin(min_distance, distance);

    if (min_distance == 0) return 0;
  }

  return min_distance;
}

HLBehavior::CollisionMap HLBehavior::get_collision_distance(bool assuming_static) {
  if (!cache_is_valid()) prepare();
  HLBehavior::CollisionMap d;
  d.reserve(resolution);
  Radians a = -aperture;
  Radians da = 2.0f * aperture / static_cast<float>(resolution);
  while (a < aperture) {
    d.push_back(std::make_tuple(
        a, assuming_static ? feared_distance_to_collision_at_relative_angle(a)
                           : distance_to_collision_at_relative_angle(a)));
    a += da;
  }
  return d;
}

Vector2 HLBehavior::compute_repulsive_force(bool &inside_obstacle) {
  Vector2 repulsive_force(0, 0);
  inside_obstacle = false;
  for (const auto &obstacle : static_obstacles_cache) {
    float d = penetration(&obstacle);
    if (d) {
      repulsive_force += d * unit(obstacle.gamma);
      inside_obstacle = true;
    }
  }
  for (const auto &neighbor : neighbors_cache) {
    float d = penetration(&neighbor);
    if (d) {
      repulsive_force += d * unit(neighbor.gamma);
      inside_obstacle = true;
    }
  }
  return repulsive_force;
}

// Absolute!
Vector2 HLBehavior::compute_desired_velocity() {
  if (!cache_is_valid()) prepare();
  // setup();

  // debugBehaviors();
  // debugStaticObstacles();

  Vector2 agentToTarget = target_pose.position - pose.position;
  Radians a0 = polar_angle(agentToTarget) - pose.orientation;
  Radians da = get_angular_resolution();
  float D = agentToTarget.norm();
  effective_horizon = horizon;
  // effective_horizon=fmin(horizon,D);

  // Vector2 effectiveTarget = agentToTarget / D * effective_horizon;

  // printf("HLBehavior: updateDesiredVelocity to %.2f, h = %.2f
  // \r\n",a0.GetValue(),horizon);

  // initDistance_cache();

  Radians searchAngle = 0.0;

  float minPossibleDistanceToTarget;
  D = effective_horizon;
  float min_distanceToTarget = D;
  float d;
  float distanceToTarget;
  Radians nearestAngle = a0;

  int leftOut = 0;
  int rightOut = 0;

  Radians maxAngle(1.6);
  // Radians::PI_OVER_TW0;

  while (searchAngle < maxAngle && !(leftOut == 2 && rightOut == 2)) {
    // new in paper
    minPossibleDistanceToTarget = fabs(sin(searchAngle) * D);
    // minPossibleDistanceToTarget=2*D*Sin(0.5*searchAngle);
    if (min_distanceToTarget < minPossibleDistanceToTarget) break;
    d = distance_to_collision_at_relative_angle(a0 + searchAngle);

    if (d == UNKNOWN_DIST && leftOut == 1) leftOut = 2;
    if (d != UNKNOWN_DIST && leftOut == 0) leftOut = 1;

    d = fmin(D, d);

    if (d != UNKNOWN_DIST) {
      if (cos(searchAngle) * D < d) {
        distanceToTarget = fabs(sin(searchAngle) * D);
      } else {
        distanceToTarget = sqrt(D * D + d * d - 2 * d * D * cos(searchAngle));
      }
      if (distanceToTarget < min_distanceToTarget) {
        min_distanceToTarget = distanceToTarget;
        nearestAngle = a0 + searchAngle;
      }
    }
    if (searchAngle > 0.0) {
      d = distance_to_collision_at_relative_angle(a0 - searchAngle);
      if (d == UNKNOWN_DIST && rightOut == 1) rightOut = 2;
      if (d != UNKNOWN_DIST && rightOut == 0) rightOut = 1;

      d = fmin(D, d);

      if (d != UNKNOWN_DIST) {
        if (cos(searchAngle) * D < d) {
          distanceToTarget = fabs(sin(searchAngle) * D);
        } else {
          distanceToTarget = sqrt(D * D + d * d - 2 * d * D * cos(searchAngle));
        }
        if (distanceToTarget < min_distanceToTarget) {
          min_distanceToTarget = distanceToTarget;
          nearestAngle = a0 - searchAngle;
        }
      }
    }
    searchAngle += da;
  }

  float nearestCollision =
      feared_distance_to_collision_at_relative_angle(nearestAngle);

  float newTargetSpeed;
  if (nearestCollision > 0) {
    newTargetSpeed = fmin(optimal_speed, nearestCollision / eta);
  } else {
    newTargetSpeed = 0;
  }

  // desiredAngle = nearestAngle;
  // desiredSpeed = newTargetSpeed;
  // TODO(Jerome): verify that all desired velocities are in the fixed frame
  return newTargetSpeed * unit(nearestAngle + pose.orientation);
}

bool HLBehavior::cache_is_valid() const {
  return !changed(HORIZON | NEIGHBORS | STATIC_OBSTACLES | POSITION | RADIUS |
                  SAFETY_MARGIN);
}

void HLBehavior::prepare() {
  for (unsigned int k = 0; k < resolution; k++)
    distance_cache[k] = UNKNOWN_DIST;
  effective_horizon = horizon;
  neighbors_cache.clear();
  static_obstacles_cache.clear();
  for (Disc &d : neighbors) {
    neighbors_cache.push_back(
        make_obstacle_cache(pose.position, d.position, d.velocity, radius,
                            d.radius, safety_margin, d.social_margin));
  }
  for (Disc &d : static_obstacles) {
    static_obstacles_cache.push_back(
        make_obstacle_cache(pose.position, d.position, radius, d.radius,
                            safety_margin, d.social_margin));
  }
  reset_changes();
}

float HLBehavior::dist_for_angle(const DiscCache *agent, Radians alpha) {
  if (agent->C < 0) {
    if (abs(normalize(alpha - agent->gamma)) < agent->visible_angle) return 0;
    return NO_COLLISION;
  }
  Vector2 dv = optimal_speed * unit(alpha) - agent->va;
  float A = dv.squaredNorm();
  // TODO(J): use dot prod
  // TODO(J 2023): maybe precompute minimal distance
  // a = optimal_speed + agent_speed
  // A = a * a
  // B = -dist * a
  // C = dist * dist - r * r
  // D = dist * dist * A - dist * dist * A + r * r * A = r * r * a * a
  // min_dist = optimal_speed * (-B - sqrt(D)) / A = optimal_speed * (dist * a -
  // r * a) (a * a)
  //          = optimal_speed * (dist - r) / (optimal_speed + agent_speed)
  float B = agent->dx.x() * dv.x() + agent->dx.y() * dv.y();

  if (B > 0) return NO_COLLISION;
  float D = B * B - A * agent->C;

  if (D < 0) return NO_COLLISION;

  return optimal_speed * (-B - sqrt(D)) / A;
}

float HLBehavior::static_dist_for_angle(const DiscCache *agent, Radians alpha) {
  if (agent->C < 0) {
    if (abs(normalize(alpha - agent->gamma)) < agent->visible_angle) return 0;
    return NO_COLLISION;
  }
  float B = agent->dx.x() * cos(alpha) + agent->dx.y() * sin(alpha);
  if (B > 0) return NO_COLLISION;
  float D = B * B - agent->C;
  if (D < 0) return NO_COLLISION;
  return -B - sqrt(D);
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
