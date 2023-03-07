/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "hl_navigation/collision_computation.h"

namespace hl_navigation {

DiscCache::DiscCache(Vector2 relative_position, float margin, Vector2 velocity)
    : relative_position(relative_position),
      velocity(velocity),
      distance(relative_position.norm() - margin),
      C(relative_position.squaredNorm() - margin * margin),
      gamma(polar_angle(relative_position)),
      visible_angle(M_PI_2) {}

CollisionComputation::CollisionMap
CollisionComputation::get_free_distance_for_sector(Radians from, Radians length,
                                                   size_t resolution,
                                                   float max_distance,
                                                   bool dynamic, float speed) {
  CollisionComputation::CollisionMap d;
  if (resolution < 1) return d;
  d.reserve(resolution);
  Radians a = from;
  Radians da = length / static_cast<float>(resolution - 1);
  for (size_t i = 0; i < resolution; i++, a += da) {
    d.push_back(std::make_tuple(
        a, dynamic ? dynamic_free_distance(a, max_distance, speed)
                   : static_free_distance(a, max_distance, true)));
  }
  return d;
}

// angle is absolute
// max_distance = horizon - radius
float CollisionComputation::static_free_distance(Radians angle,
                                                 float max_distance,
                                                 bool include_neighbors) {
  max_distance =
      static_free_distance_to_collection(angle, max_distance, line_obstacles);
  if (max_distance == 0) return 0;
  max_distance = static_free_distance_to_collection(angle, max_distance,
                                                    static_obstacles_cache);
  if (!include_neighbors || max_distance == 0) return max_distance;
  return static_free_distance_to_collection(angle, max_distance,
                                            neighbors_cache);
}

// angle is absolute
float CollisionComputation::dynamic_free_distance(Radians angle,
                                                  float max_distance,
                                                  float speed) {
  max_distance = static_free_distance(angle, max_distance, false);
  if (max_distance == 0) return 0;
  return dynamic_free_distance_to_collection(angle, max_distance, speed,
                                             neighbors_cache);
}

float CollisionComputation::static_free_distance_to(const LineSegment &line,
                                                    Radians alpha) {
  Vector2 delta = pose.position - line.p1;
  float y = delta.dot(line.e2);
  float x = delta.dot(line.e1);
  Vector2 e = unit(alpha);
  float d = line.e2.dot(e);
  if (y * d >= 0) {
    // moving away
    return no_collision;
  }
  if (abs(y) < margin && x > -margin && x < line.length + margin) {
    // already colliding
    return 0.0;
  }
  float distance = -y / d - margin;
  x = line.e1.dot(distance * e + delta);
  if (x < -margin || x > line.length + margin) {
    // will not collide
    return no_collision;
  }
  return distance;
}

float CollisionComputation::static_free_distance_to(const DiscCache &disc,
                                                    Radians alpha) {
  if (disc.C < 0) {
    if (abs(normalize(alpha - disc.gamma)) < disc.visible_angle) return 0;
    return no_collision;
  }
  const float B = disc.relative_position.x() * cos(alpha) +
                  disc.relative_position.y() * sin(alpha);
  if (B < 0) return no_collision;
  const float D = B * B - disc.C;
  if (D < 0) return no_collision;
  return B - sqrt(D);
}

float CollisionComputation::dynamic_free_distance_to(const DiscCache &disc,
                                                     Radians alpha,
                                                     float speed) {
  if (disc.C < 0) {
    if (abs(normalize(alpha - disc.gamma)) < disc.visible_angle) return 0;
    return no_collision;
  }
  Vector2 dv = speed * unit(alpha) - disc.velocity;
  const float A = dv.squaredNorm();
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
  const float B =
      disc.relative_position.x() * dv.x() + disc.relative_position.y() * dv.y();

  if (B < 0) return no_collision;
  const float D = B * B - A * disc.C;

  if (D < 0) return no_collision;

  return speed * (B - sqrt(D)) / A;
}

bool CollisionComputation::dynamic_may_collide(const DiscCache &c,
                                               float max_distance,
                                               float speed) {
  return (speed * c.distance / (speed + c.velocity.norm())) < max_distance;
}

bool CollisionComputation::static_may_collide(const DiscCache &c,
                                              float max_distance) {
  return c.distance < max_distance;
}

}  // namespace hl_navigation
