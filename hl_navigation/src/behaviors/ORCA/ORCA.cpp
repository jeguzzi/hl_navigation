/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "hl_navigation/behaviors/ORCA.h"

#include "RVO/Agent.h"
#include "RVO/Definitions.h"
#include "RVO/Obstacle.h"
#include "RVO/Vector2.h"

#define DEFAULT_TIME_STEP 0.1

namespace hl_navigation {

ORCABehavior::ORCABehavior(std::shared_ptr<Kinematics> kinematics, float radius)
    : Behavior(kinematics, radius),
      state(),
      use_effective_center(false),
      _RVOAgent(std::make_unique<RVO::Agent>(nullptr)) {
  _RVOAgent->maxNeighbors_ = 1000;
  _RVOAgent->timeStep_ = DEFAULT_TIME_STEP;
  _RVOAgent->timeHorizon_ = 10.0f;
}

ORCABehavior::~ORCABehavior() = default;

// DONE(J: revision): why do I need rvo_neighbors. It is not enough to use
// _RVOAgent->rvo_neighbors_? No, because insertObstacleNeighbor may remove some
// entry

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

void ORCABehavior::add_obstacle(const Disc &obstacle, float rangeSq,
                                bool push_away, float epsilon) {
    auto a = std::make_unique<RVO::Agent>(nullptr);
    a->velocity_ = RVO::Vector2(0, 0);
    a->prefVelocity_ = a->velocity_;
    Vector2 p = obstacle.position;
    const float margin = obstacle.radius + safety_margin + radius;
    const Vector2 delta = obstacle.position - pose.position;
    const float distance = delta.norm() - margin;
    if (push_away && distance < epsilon) {
      p += delta / delta.norm() * (-distance + epsilon);
    }
    a->position_ = RVO::Vector2((float)p.x(), (float)p.y());
    a->radius_ = obstacle.radius + safety_margin;
    _RVOAgent->insertAgentNeighbor(a.get(), rangeSq);
    rvo_neighbors.push_back(std::move(a));
}

void ORCABehavior::add_neighbor(const Neighbor &neighbor, float rangeSq,
                                bool push_away, float epsilon) {
  auto a = std::make_unique<RVO::Agent>(nullptr);
  a->velocity_ =
      RVO::Vector2((float)neighbor.velocity.x(), (float)neighbor.velocity.y());
  a->prefVelocity_ = a->velocity_;

  Vector2 p = neighbor.position;
  const float margin = neighbor.radius + safety_margin + radius;
  const Vector2 delta = neighbor.position - pose.position;
  float distance = delta.norm() - margin;
  if (push_away && distance < epsilon) {
    p += delta / delta.norm() * (-distance + epsilon);
    distance = epsilon;
  }
  a->position_ = RVO::Vector2((float)p.x(), (float)p.y());
  a->radius_ = neighbor.radius + safety_margin + social_margin.get(0, distance);
  // [[maybe_unused]] Vector2 relative_position =
  //     obstacle_relative_position(pose.position, p, radius, d.radius,
  //     distance);
  // a->radius_ = d.radius + fmin(distance - d.radius - _RVOAgent->radius_ -
  // 0.001,
  //                              obstacle_margin(distance, radius, d.radius,
  //                                              safety_margin,
  //                                              d.social_margin));
  _RVOAgent->insertAgentNeighbor(a.get(), rangeSq);
  rvo_neighbors.push_back(std::move(a));
}

void ORCABehavior::set_time_horizon(float value) {
  _RVOAgent->timeHorizon_ = value;
}
float ORCABehavior::get_time_horizon() const { return _RVOAgent->timeHorizon_; }
void ORCABehavior::set_time_step(float value) { _RVOAgent->timeStep_ = value; }
float ORCABehavior::get_time_step() const { return _RVOAgent->timeStep_; }


void ORCABehavior::prepare() {
  if (is_using_effective_center()) {
    Wheeled *wk = dynamic_cast<Wheeled *>(kinematics.get());
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

  const float rangeSq = (horizon * 2) * (horizon * 2);
  if (state.changed(GeometricState::LINE_OBSTACLES)) {
    rvo_obstacles.clear();
    _RVOAgent->obstacleNeighbors_.clear();
    for (auto &line : state.get_line_obstacles()) {
      add_line_obstacle(line);
    }

    for (auto &obstacle : rvo_obstacles) {
      _RVOAgent->insertObstacleNeighbor(obstacle.get(), rangeSq);
    }
  }
  if (state.changed(GeometricState::STATIC_OBSTACLES | GeometricState::NEIGHBORS)) {
    _RVOAgent->agentNeighbors_.clear();
    rvo_neighbors.clear();
    for (const auto &n : state.get_neighbors()) {
      add_neighbor(n, rangeSq, true, 2e-3);
    }
    for (const auto &o : state.get_static_obstacles()) {
      add_obstacle(o, rangeSq, true, 2e-3);
    }
  }
  state.reset_changes();
}

Vector2 ORCABehavior::compute_desired_velocity(float dt) {
  prepare();
  set_time_step(dt);
  _RVOAgent->computeNewVelocity();
  return Vector2(_RVOAgent->newVelocity_.x(), _RVOAgent->newVelocity_.y());
}

// TODO(J 2023): review ... should I check feasibility?
Twist2 ORCABehavior::twist_towards_velocity(const Vector2 &absolute_velocity,
                                            bool relative) {
  if (is_using_effective_center()) {
    Radians angle = orientation_of(absolute_velocity) - pose.orientation;
    float speed = absolute_velocity.norm();
    if (speed) {
      Wheeled *wk = dynamic_cast<Wheeled *>(kinematics.get());
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

// const char *ORCABehavior::name = register_type<ORCABehavior>("ORCA");

}  // namespace hl_navigation
