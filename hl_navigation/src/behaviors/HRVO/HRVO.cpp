/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "hl_navigation/behaviors/HRVO.h"

#include "HRVO/Agent.h"
#include "HRVO/HRVO.h"
#include "HRVO/Obstacle.h"

namespace hl_navigation {

HRVOBehavior::HRVOBehavior(std::shared_ptr<Kinematic> kinematic, float radius)
    : Behavior(kinematic, radius),
      _HRVOAgent(std::make_unique<HRVO::Agent>()) {
  _HRVOAgent->radius_ = radius;
  _HRVOAgent->maxNeighbors_ = 1000;
}
HRVOBehavior::~HRVOBehavior() = default;

bool HRVOBehavior::cache_is_valid() const {
  return !changed(TARGET_POSITION | POSITION | VELOCITY | ORIENTATION |
                  NEIGHBORS | STATIC_OBSTACLES | RADIUS | SAFETY_MARGIN |
                  HORIZON | OPTIMAL_SPEED);
}

void HRVOBehavior::prepare() {
  if (cache_is_valid()) return;
  _HRVOAgent->velocity_ = HRVO::Vector2(twist.velocity.x(), twist.velocity.y());
  _HRVOAgent->orientation_ = normalize(pose.orientation);
  _HRVOAgent->position_ = HRVO::Vector2(pose.position.x(), pose.position.y());

  _HRVOAgent->neighborDist_ = 2 * horizon;
  _HRVOAgent->isColliding_ = false;
  rangeSq = (horizon * 2) * (horizon * 2);
  HRVO::Vector2 t =
      (HRVO::Vector2(target_pose.position.x(), target_pose.position.y()) -
       _HRVOAgent->position_);
  _HRVOAgent->prefVelocity_ = t * optimal_speed / abs(t);
  _HRVOAgent->prefSpeed_ = optimal_speed;
  _HRVOAgent->maxSpeed_ = optimal_speed;
  _HRVOAgent->uncertaintyOffset_ = 0;

  _HRVOAgent->neighbors_.clear();
  for (uint i = 0; i < _HRVOAgent->obstacles_.size(); i++) {
    delete _HRVOAgent->obstacles_[i];
  }

  for (uint i = 0; i < _HRVOAgent->agents_.size(); i++) {
    delete _HRVOAgent->agents_[i];
  }
  _HRVOAgent->obstacles_.clear();
  _HRVOAgent->agents_.clear();
  agentIndex = 0;

  for (const auto &n : neighbors) {
    add_neighbor(n);
  }
  for (const auto &o : static_obstacles) {
    add_neighbor(o);
  }

  reset_changes();
}

void HRVOBehavior::add_neighbor(const Disc &d) {
  HRVO::Agent *a = new HRVO::Agent();
  Vector2 p = d.position;
  a->velocity_ = HRVO::Vector2((float)d.velocity.x(), (float)d.velocity.y());
  a->position_ = HRVO::Vector2((float)p.x(), (float)p.y());

  float distance;
  Vector2 relative_position =
      obstacle_relative_position(pose.position, p, radius, d.radius, distance);
  // a->radius_=r+marginForObstacleAtDistance(distance,r,safetyMargin,socialMargin);
  a->radius_ =
      d.radius + fmin(distance - d.radius - _HRVOAgent->radius_ - 0.001,
                      obstacle_margin(distance, radius, d.radius, safety_margin,
                                      d.social_margin));
  // printf("Obstacle radius %.3f\n",a->radius_);

  // a->radius_=r+marginForObstacleAtDistance(p.norm(),r,safetyMargin,socialMargin);
  a->prefVelocity_ = a->velocity_;
  _HRVOAgent->agents_.push_back(a);
  _HRVOAgent->insertAgentNeighbor(agentIndex, rangeSq);
  agentIndex++;
}

Vector2 HRVOBehavior::compute_desired_velocity() {
  prepare();
  _HRVOAgent->computeNewVelocity();
  return {_HRVOAgent->newVelocity_.x(), _HRVOAgent->newVelocity_.y()};
}

const char *HRVOBehavior::name = register_type<HRVOBehavior>("HRVO");

}  // namespace hl_navigation
