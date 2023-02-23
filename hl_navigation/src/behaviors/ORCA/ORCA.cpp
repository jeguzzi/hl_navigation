/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "hl_navigation/behaviors/ORCA.h"

#include "RVO/Agent.h"
#include "RVO/Definitions.h"
#include "RVO/Obstacle.h"
#include "RVO/Vector2.h"

// TODO(J:revision2023): TIME_STEP (should pass/set) and timeHorizon (should not
// duplicate)

#define DEFAULT_TIME_STEP 0.1

namespace hl_navigation {

ORCABehavior::ORCABehavior(std::shared_ptr<Kinematic> kinematic, float radius)
    : Behavior(kinematic, radius),
      use_effective_center(false),
      _RVOAgent(std::make_unique<RVO::Agent>(nullptr)) {
  _RVOAgent->maxNeighbors_ = 1000;
  _RVOAgent->timeStep_ = DEFAULT_TIME_STEP;
  _RVOAgent->timeHorizon_ = 10.0;
}

ORCABehavior::~ORCABehavior() = default;

// TODO(J: revision): why do I need rvo_neighbors. It is not enught to use
// _RVOAgent->rvo_neighbors_? No, because insertObstacleNeighbor may remove some
// entry

void ORCABehavior::set_line_obstacles(const std::vector<LineSegment> &value) {
  Behavior::set_line_obstacles(value);
  rvo_obstacles.clear();
  for (auto &line : line_obstacles) {
    add_line_obstacle(line);
  }
}

void ORCABehavior::add_line_obstacle(const LineSegment &line) {
  Vector2 pa = line.p1;  // - line.e1 * safetyMargin - line.e2 * safetyMargin;
  Vector2 pb = line.p2;  // + line.e1 * safetyMargin + line.e2 * safetyMargin;
  auto a = std::make_unique<RVO::Obstacle>();
  auto b = std::make_unique<RVO::Obstacle>();
  a->point_ = RVO::Vector2(pa[0], pa[1]);
  a->prevObstacle = b.get();
  a->nextObstacle = b.get();
  a->isConvex_ = true;
  a->unitDir_ = RVO::Vector2(line.e1[0], line.e1[1]);
  b->point_ = RVO::Vector2(pb[0], pb[1]);
  b->prevObstacle = a.get();
  b->nextObstacle = a.get();
  b->isConvex_ = true;
  b->unitDir_ = -a->unitDir_;
  rvo_obstacles.push_back(std::move(a));
  rvo_obstacles.push_back(std::move(b));
}

// TODO(old) add non penetration check!

void ORCABehavior::add_neighbor(const Disc &d) {
  auto a = std::make_unique<RVO::Agent>(nullptr);
  Vector2 p = d.position;
  a->velocity_ = RVO::Vector2((float)d.velocity.x(), (float)d.velocity.y());

  a->position_ = RVO::Vector2((float)p.x(), (float)p.y());
  float distance;

  Vector2 relative_position =
      obstacle_relative_position(pose.position, p, radius, d.radius, distance);
  a->radius_ = d.radius + fmin(distance - d.radius - _RVOAgent->radius_ - 0.001,
                               obstacle_margin(distance, radius, d.radius,
                                               safety_margin, d.social_margin));
  a->prefVelocity_ = a->velocity_;
  _RVOAgent->insertAgentNeighbor(a.get(), rangeSq);
  rvo_neighbors.push_back(std::move(a));
}

void ORCABehavior::set_time_horizon(float value) {
  _RVOAgent->timeHorizon_ = value;
}
float ORCABehavior::get_time_horizon() const { return _RVOAgent->timeHorizon_; }
void ORCABehavior::set_time_step(float value) { _RVOAgent->timeStep_ = value; }
float ORCABehavior::get_time_step() const { return _RVOAgent->timeStep_; }

bool ORCABehavior::cache_is_valid() const {
  return !changed(POSITION | NEIGHBORS | STATIC_OBSTACLES | LINE_OBSTACLES |
                  RADIUS | OPTIMAL_SPEED | RADIUS | TARGET_POSITION |
                  SAFETY_MARGIN | HORIZON);
}

// TODO(J): still need the float casting?
void ORCABehavior::prepare() {
  if (use_effective_center && kinematic->is_wheeled() &&
      kinematic->dof() == 2) {
    Wheeled *wk = dynamic_cast<Wheeled *>(kinematic.get());
    D = wk->get_axis() * 0.5;
    RVO::Vector2 delta =
        RVO::Vector2(cosf(pose.orientation), sinf(pose.orientation)) * D;
    _RVOAgent->position_ =
        RVO::Vector2(pose.position.x(), pose.position.y()) + delta;
    _RVOAgent->radius_ = radius + D;
    _RVOAgent->velocity_ = RVO::Vector2(
        twist.velocity.x() - sin(pose.orientation) * D * twist.angular_speed,
        twist.velocity.y() + cos(pose.orientation) * D * twist.angular_speed);
    _RVOAgent->maxSpeed_ =
        optimal_speed / sqrt(1 + RVO::sqr(0.5 * wk->get_axis() / D));
  } else {
    _RVOAgent->radius_ = radius;
    _RVOAgent->velocity_ = RVO::Vector2(twist.velocity.x(), twist.velocity.y());
    _RVOAgent->position_ = RVO::Vector2(pose.position.x(), pose.position.y());
    _RVOAgent->maxSpeed_ = optimal_speed;
  }
  // TODO(Jerome): why 2 * ... verify
  _RVOAgent->neighborDist_ = 2 * horizon;

  RVO::Vector2 t =
      RVO::Vector2(target_pose.position.x(), target_pose.position.y()) -
      _RVOAgent->position_;
  _RVOAgent->prefVelocity_ = t * _RVOAgent->maxSpeed_ / abs(t);

  rangeSq = (horizon * 2) * (horizon * 2);

  _RVOAgent->agentNeighbors_.clear();
  _RVOAgent->obstacleNeighbors_.clear();
  rvo_neighbors.clear();

  for (auto &obstacle : rvo_obstacles) {
    _RVOAgent->insertObstacleNeighbor(obstacle.get(), rangeSq);
  }
  for (const auto &n : neighbors) {
    add_neighbor(n);
  }
  for (const auto &o : static_obstacles) {
    add_neighbor(o);
  }
  reset_changes();
}

Vector2 ORCABehavior::compute_desired_velocity() {
  if (!cache_is_valid()) prepare();
  _RVOAgent->computeNewVelocity();
  return Vector2(_RVOAgent->newVelocity_.x(), _RVOAgent->newVelocity_.y());
}

// TODO(J 2023): review ... should I check feasibility?
Twist2 ORCABehavior::twist_towards_velocity(const Vector2 &absolute_velocity,
                                            bool relative) {
  if (use_effective_center && kinematic->is_wheeled() &&
      kinematic->dof() == 2) {
    Radians angle = polar_angle(absolute_velocity) - pose.orientation;
    float speed = absolute_velocity.norm();
    if (speed) {
      Wheeled *wk = dynamic_cast<Wheeled *>(kinematic.get());
      WheelSpeeds speeds = {
          speed * (cosf(angle) - wk->get_axis() * 0.5f / D * sinf(angle)),
          speed * (cosf(angle) + wk->get_axis() * 0.5f / D * sinf(angle))};
      Twist2 twist = wk->twist(speeds);
      if (!relative) {
        return to_absolute(twist);
      }
      return twist;
    } else {
      return {{}, 0.0f, relative};
    }
  }
  return Behavior::twist_towards_velocity(absolute_velocity, relative);
}

Twist2 ORCABehavior::cmd_twist_towards_target(float dt, bool relative) {
  set_time_step(dt);
  return Behavior::cmd_twist_towards_target(dt, relative);
}

const char *ORCABehavior::name = register_type<ORCABehavior>("ORCA");

}  // namespace hl_navigation
