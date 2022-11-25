/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "HRVOAgent.h"

void HRVOAgent::clearObstacles() {
  _HRVOAgent->velocity_ =
      HRVO::Vector2((float)velocity.GetX(), (float)velocity.GetY());
  _HRVOAgent->orientation_ = angle.SignedNormalize().GetValue();
  _HRVOAgent->position_ =
      HRVO::Vector2((float)position.GetX(), (float)position.GetY());

  _HRVOAgent->neighborDist_ = 2 * horizon;
  _HRVOAgent->isColliding_ = false;
  _HRVOAgent->neighbors_.clear();

  for (uint i = 0; i < _HRVOAgent->obstacles_.size(); i++) {
    delete _HRVOAgent->obstacles_[i];
  }

  for (uint i = 0; i < _HRVOAgent->agents_.size(); i++) {
    delete _HRVOAgent->agents_[i];
  }

  _HRVOAgent->obstacles_.clear();
  _HRVOAgent->agents_.clear();
  rangeSq = (horizon * 2) * (horizon * 2);
  agentIndex = 0;
}

void HRVOAgent::setup() {
  HRVO::Vector2 t = HRVO::Vector2((float)targetPosition.GetX(),
                                  (float)targetPosition.GetY()) -
                    _HRVOAgent->position_;
  _HRVOAgent->prefVelocity_ = t * optimalSpeed / abs(t);
  _HRVOAgent->prefSpeed_ = optimalSpeed;
  _HRVOAgent->maxSpeed_ = optimalSpeed;
  _HRVOAgent->uncertaintyOffset_ = 0;
}

void HRVOAgent::updateDesiredVelocity() {
  setup();
  _HRVOAgent->computeNewVelocity();
  CVector2 newVelocity(_HRVOAgent->newVelocity_.x(),
                       _HRVOAgent->newVelocity_.y());
  desiredAngle = (newVelocity.Angle() - angle).SignedNormalize();
  desiredSpeed = abs(_HRVOAgent->newVelocity_);
}

void HRVOAgent::updateVelocity() { Agent::updateVelocity(); }

HRVOAgent::HRVOAgent() : Agent() {
  _HRVOAgent = new HRVO::Agent();
  _HRVOAgent->radius_ = radius;
  _HRVOAgent->maxNeighbors_ = 1000;
}

HRVOAgent::~HRVOAgent() { delete _HRVOAgent; }

void HRVOAgent::addObstacleAtPoint(CVector2 p, CVector2 v, Real r,
                                   Real socialMargin) {
  HRVO::Agent *a = new HRVO::Agent();
  a->velocity_ = HRVO::Vector2((float)v.GetX(), (float)v.GetY());
  a->position_ = HRVO::Vector2((float)p.GetX(), (float)p.GetY());

  Real distance;
  CVector2 relativePosition = relativePositionOfObstacleAt(p, r, distance);
  // a->radius_=r+marginForObstacleAtDistance(distance,r,safetyMargin,socialMargin);
  a->radius_ = r + fmin(distance - r - _HRVOAgent->radius_ - 0.001,
                        marginForObstacleAtDistance(distance, r, safetyMargin,
                                                    socialMargin));
  // printf("Obstacle radius %.3f\n",a->radius_);

  // a->radius_=r+marginForObstacleAtDistance(p.Length(),r,safetyMargin,socialMargin);
  a->prefVelocity_ = a->velocity_;
  _HRVOAgent->agents_.push_back(a);
  _HRVOAgent->insertAgentNeighbor(agentIndex, rangeSq);
  agentIndex++;
}

void HRVOAgent::addObstacleAtPoint(CVector2 p, Real r, Real socialMargin) {
  addObstacleAtPoint(p, CVector2(0, 0), r, socialMargin);
}
