/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "behaviors/ORCA.h"
#include "RVO/Agent.h"
#include "RVO/Definitions.h"
#include "RVO/Obstacle.h"
#include "RVO/Vector2.h"

// TODO(J:revision2023): TIME_STEP (should pass/set) and timeHorizon (should not duplicate)

namespace hl_navigation {

ORCABehavior::ORCABehavior(agent_type_t type, float radius, float axis_length) :
  Behavior(type, radius, axis_length), useEffectiveCenter(false), timeHorizon(10.0),
  _RVOAgent(std::make_unique<RVO::Agent>(nullptr)) {
  _RVOAgent->maxNeighbors_ = 1000;
  _RVOAgent->timeStep_ = TIME_STEP;
  _RVOAgent->timeHorizon_ = timeHorizon;
}

// TODO(J: revision): why do I need agentNeighbors. It is not enught to use
// _RVOAgent->agentNeighbors_? No, because insertObstacleNeighbor may remove some entry

ORCABehavior::~ORCABehavior() = default;

void ORCABehavior::setTimeHorizon(float value) { timeHorizon = value; }
float ORCABehavior::getTimeHorizon(float value) const { return timeHorizon; }

void ORCABehavior::setTimeStep(float value) { _RVOAgent->timeStep_  = value; }
float ORCABehavior::getTimeStep(float value) const { return _RVOAgent->timeStep_; }

void ORCABehavior::prepare_line_obstacles() {
  for (auto & obstacle : obstacleNeighbors) {
    _RVOAgent->insertObstacleNeighbor(obstacle.get(), rangeSq);
  }
}

void ORCABehavior::set_line_obstacles(const std::vector<LineSegment> & value) {
  Behavior::set_line_obstacles(value);
  obstacleNeighbors.clear();
  for (auto & line : line_obstacles) {
    add_line_obstacle(line);
  }
}

void ORCABehavior::add_line_obstacle(const LineSegment & line) {
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
  obstacleNeighbors.push_back(std::move(a));
  obstacleNeighbors.push_back(std::move(b));
}

// TODO(J): still need the float casting?
void ORCABehavior::prepare() {
  if (useEffectiveCenter) {
    D = axisLength * 0.5;
    RVO::Vector2 delta = RVO::Vector2((float)cos(angle), (float)sin(angle)) * D;
    _RVOAgent->position_ =
        RVO::Vector2((float)position.x(), (float)position.y()) + delta;
    _RVOAgent->radius_ = radius + D;
    _RVOAgent->velocity_ = RVO::Vector2(
        (float)(velocity.x() - sin(angle) * D * angularSpeed),
        (float)(velocity.y() + cos(angle) * D * angularSpeed));
    _RVOAgent->maxSpeed_ =
        optimalSpeed / sqrt(1 + RVO::sqr(0.5 * axisLength / D));
  } else {
    _RVOAgent->radius_ = radius;
    _RVOAgent->velocity_ =
        RVO::Vector2((float)velocity.x(), (float)velocity.y());
    _RVOAgent->position_ =
        RVO::Vector2((float)position.x(), (float)position.y());
    _RVOAgent->maxSpeed_ = optimalSpeed;
  }
  _RVOAgent->timeHorizon_ = timeHorizon;
  // TODO(Jerome): why 2 * ... verify
  _RVOAgent->neighborDist_ = 2 * horizon;

  RVO::Vector2 t = RVO::Vector2(
      targetPosition.x(), targetPosition.y()) - _RVOAgent->position_;
  _RVOAgent->prefVelocity_ = t * _RVOAgent->maxSpeed_ / abs(t);

  rangeSq = (horizon * 2) * (horizon * 2);

  prepare_line_obstacles();
}

void ORCABehavior::clear() {
  _RVOAgent->obstacleNeighbors_.clear();
  _RVOAgent->agentNeighbors_.clear();
  agentNeighbors.clear();
}

void ORCABehavior::update_desired_velocity() {
  _RVOAgent->computeNewVelocity();
  desiredVelocity = Vector2(_RVOAgent->newVelocity_.x(), _RVOAgent->newVelocity_.y());
}

Twist2D ORCABehavior::compute_desired_twist() const {
  if (type == TWO_WHEELED && useEffectiveCenter) {
    Radians desiredAngle = polar_angle(desiredVelocity) - angle;
    float desiredSpeed = desiredVelocity.norm();
    if (desiredSpeed) {
      WheelSpeeds speeds = {
        static_cast<float>(
            desiredSpeed * (cos(desiredAngle) - axisLength * 0.5 / D * sin(desiredAngle))),
        static_cast<float>(
            desiredSpeed * (cos(desiredAngle) + axisLength * 0.5 / D * sin(desiredAngle)))};
      return twist_from_wheel_speeds(speeds);
    }
  }
  return Behavior::compute_desired_twist();
}


// TODO(old) add non penetration check!


void ORCABehavior::add_neighbor(const Disc & d) {
  auto a = std::make_unique<RVO::Agent>(nullptr);
  Vector2 p = d.position;
  a->velocity_ = RVO::Vector2((float)d.velocity.x(), (float)d.velocity.y());

  a->position_ = RVO::Vector2((float)p.x(), (float)p.y());
  float distance;

  Vector2 relativePosition = relativePositionOfObstacleAt(p, d.radius, distance);
  a->radius_ = d.radius + fmin(distance - d.radius - _RVOAgent->radius_ - 0.001,
                        marginForObstacleAtDistance(distance, d.radius, safetyMargin,
                                                    d.social_margin));

  // printf("Obstacle (%.3f,%.3f) (%.3f,%.3f) %.5f -> radius
  // %.3f\n",p.x(),p.y(),v.x(),v.y(),r,a->radius_);

  a->prefVelocity_ = a->velocity_;
  _RVOAgent->insertAgentNeighbor(a.get(), rangeSq);
  agentNeighbors.push_back(std::move(a));
}

void ORCABehavior::add_static_obstacle(const Disc & d) {
  add_neighbor(d);
}

const char * ORCABehavior::name = register_type<ORCABehavior>("ORCA");

}  // namespace hl_navigation
