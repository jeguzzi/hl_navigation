/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "HRVOAgent.h"
#include "HRVO/Obstacle.h"

void HRVOAgent::prepare() {
  _HRVOAgent->velocity_ =
      HRVO::Vector2((float)velocity.GetX(), (float)velocity.GetY());
  _HRVOAgent->orientation_ = angle.SignedNormalize().GetValue();
  _HRVOAgent->position_ =
      HRVO::Vector2((float)position.GetX(), (float)position.GetY());

  _HRVOAgent->neighborDist_ = 2 * horizon;
  _HRVOAgent->isColliding_ = false;
  rangeSq = (horizon * 2) * (horizon * 2);
  HRVO::Vector2 t = HRVO::Vector2((float)targetPosition.GetX(),
                                  (float)targetPosition.GetY()) -
                    _HRVOAgent->position_;
  _HRVOAgent->prefVelocity_ = t * optimalSpeed / abs(t);
  _HRVOAgent->prefSpeed_ = optimalSpeed;
  _HRVOAgent->maxSpeed_ = optimalSpeed;
  _HRVOAgent->uncertaintyOffset_ = 0;
}

void HRVOAgent::clear() {
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
}

void HRVOAgent::update_desired_velocity() {
  _HRVOAgent->computeNewVelocity();
  desiredVelocity = CVector2(_HRVOAgent->newVelocity_.x(), _HRVOAgent->newVelocity_.y());
}

void HRVOAgent::add_neighbor(const Disc & d) {
  HRVO::Agent *a = new HRVO::Agent();
  CVector2 p = d.position;
  a->velocity_ = HRVO::Vector2((float)d.velocity.GetX(), (float)d.velocity.GetY());
  a->position_ = HRVO::Vector2((float)p.GetX(), (float)p.GetY());

  Real distance;
  CVector2 relativePosition = relativePositionOfObstacleAt(p, d.radius, distance);
  // a->radius_=r+marginForObstacleAtDistance(distance,r,safetyMargin,socialMargin);
  a->radius_ = d.radius + fmin(
      distance - d.radius - _HRVOAgent->radius_ - 0.001,
      marginForObstacleAtDistance(distance, d.radius, safetyMargin, d.social_margin));
  // printf("Obstacle radius %.3f\n",a->radius_);

  // a->radius_=r+marginForObstacleAtDistance(p.Length(),r,safetyMargin,socialMargin);
  a->prefVelocity_ = a->velocity_;
  _HRVOAgent->agents_.push_back(a);
  _HRVOAgent->insertAgentNeighbor(agentIndex, rangeSq);
  agentIndex++;
}

void HRVOAgent::add_static_obstacle(const Disc & d) {
  add_neighbor(d);
}

const char * HRVOAgent::name = register_type<HRVOAgent>("HRVO");
