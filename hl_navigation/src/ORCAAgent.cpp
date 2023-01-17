/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "ORCAAgent.h"

void ORCAAgent::setTimeHorizon(double value) { timeHorizon = value; }

void ORCAAgent::prepare() {
  if (useEffectiveCenter) {
    D = axisLength * 0.5;
    RVO::Vector2 delta = RVO::Vector2((float)Cos(angle), (float)Sin(angle)) * D;
    _RVOAgent->position_ =
        RVO::Vector2((float)position.GetX(), (float)position.GetY()) + delta;
    _RVOAgent->radius_ = radius + D;
    _RVOAgent->velocity_ = RVO::Vector2(
        (float)(velocity.GetX() - Sin(angle) * D * angularSpeed.GetValue()),
        (float)(velocity.GetY() + Cos(angle) * D * angularSpeed.GetValue()));
    _RVOAgent->maxSpeed_ =
        optimalSpeed / sqrt(1 + RVO::sqr(0.5 * axisLength / D));
  } else {
    _RVOAgent->radius_ = radius;
    _RVOAgent->velocity_ =
        RVO::Vector2((float)velocity.GetX(), (float)velocity.GetY());
    _RVOAgent->position_ =
        RVO::Vector2((float)position.GetX(), (float)position.GetY());
    _RVOAgent->maxSpeed_ = optimalSpeed;
  }
  _RVOAgent->timeHorizon_ = timeHorizon;
  _RVOAgent->neighborDist_ = 2 * horizon;

  RVO::Vector2 t = RVO::Vector2(
      targetPosition.GetX(), targetPosition.GetY()) - _RVOAgent->position_;
  _RVOAgent->prefVelocity_ = t * _RVOAgent->maxSpeed_ / abs(t);

  rangeSq = (horizon * 2) * (horizon * 2);
}

void ORCAAgent::clear() {
  for (uint i = 0; i < _RVOAgent->obstacleNeighbors_.size(); i++) {
    delete _RVOAgent->obstacleNeighbors_[i].second;
  }
  for (uint i = 0; i < agentNeighbors.size(); i++) {
    delete agentNeighbors[i];
  }

  agentNeighbors.clear();
  _RVOAgent->agentNeighbors_.clear();
  _RVOAgent->obstacleNeighbors_.clear();
}

void ORCAAgent::update_desired_velocity() {
  _RVOAgent->computeNewVelocity();
  desiredVelocity = CVector2(_RVOAgent->newVelocity_.x(), _RVOAgent->newVelocity_.y());
}

Twist2D ORCAAgent::compute_desired_twist() const {
  if (type == TWO_WHEELED && useEffectiveCenter) {
    CRadians desiredAngle = desiredVelocity.Angle() - angle;
    double desiredSpeed = desiredVelocity.Length();
    if (desiredSpeed) {
      WheelSpeeds speeds = {
        static_cast<float>(
            desiredSpeed * (Cos(desiredAngle) - axisLength * 0.5 / D * Sin(desiredAngle))),
        static_cast<float>(
            desiredSpeed * (Cos(desiredAngle) + axisLength * 0.5 / D * Sin(desiredAngle)))};
      return twist_from_wheel_speeds(speeds);
    }
  }
  return Agent::compute_desired_twist();
}


// TODO(old) add non penetration check!


void ORCAAgent::add_neighbor(const Disc & d) {
  RVO::Agent *a = new RVO::Agent(NULL);
  CVector2 p = d.position;
  a->velocity_ = RVO::Vector2((float)d.velocity.GetX(), (float)d.velocity.GetY());

  a->position_ = RVO::Vector2((float)p.GetX(), (float)p.GetY());
  Real distance;

  CVector2 relativePosition = relativePositionOfObstacleAt(p, d.radius, distance);
  a->radius_ = d.radius + fmin(distance - d.radius - _RVOAgent->radius_ - 0.001,
                        marginForObstacleAtDistance(distance, d.radius, safetyMargin,
                                                    d.social_margin));

  // printf("Obstacle (%.3f,%.3f) (%.3f,%.3f) %.5f -> radius
  // %.3f\n",p.GetX(),p.GetY(),v.GetX(),v.GetY(),r,a->radius_);

  a->prefVelocity_ = a->velocity_;
  agentNeighbors.push_back(a);
  _RVOAgent->insertAgentNeighbor(a, rangeSq);
}

void ORCAAgent::add_static_obstacle(const Disc & d) {
  add_neighbor(d);
}

const char * ORCAAgent::name = register_type<ORCAAgent>("ORCA");
