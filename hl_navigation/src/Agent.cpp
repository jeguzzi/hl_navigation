/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "Agent.h"

template<typename T>
static T clamp(T value, T min, T max) {
  return std::min(std::max(value, min), max);
}

Twist2D Agent::twist_from_wheel_speeds(const WheelSpeeds & speeds) const {
  if (type == TWO_WHEELED && speeds.size() == 2) {
    // {left, right}
    return Twist2D(0.5 * (speeds[0] + speeds[1]), 0.0, (speeds[1] - speeds[0]) / axisLength);
  }
  if (type == FOUR_WHEELED_OMNI && speeds.size() == 4) {
    // {front left, rear left, rear right, rear left}
    return Twist2D(
        0.25 * (speeds[0] + speeds[1] + speeds[2] + speeds[3]),
        0.25 * (-speeds[0] + speeds[1] - speeds[2] + speeds[3]),
        0.25 * (-speeds[0] - speeds[1] + speeds[2] + speeds[3]) / axisLength);
  }
  return Twist2D(0, 0, 0);
}

WheelSpeeds Agent::wheel_speeds_from_twist(const Twist2D & twist) const {
  if (type == TWO_WHEELED) {
     // {left, right}
     const float rotation = clamp(0.5f * twist.angular * axisLength, -maxSpeed, maxSpeed);
     const float linear = clamp(twist.longitudinal, 0.0f, maxSpeed);
     float left = linear - rotation;
     float right = linear + rotation;
     if (abs(left) > maxSpeed) {
       left = clamp(left, -maxSpeed, maxSpeed);
       right = left + 2 * rotation;
     } else if (abs(right) > maxSpeed) {
       right = clamp(right, -maxSpeed, maxSpeed);
       left = right - 2 * rotation;
     }
     return {left, right};
  }
  if (type == FOUR_WHEELED_OMNI) {
     // {front left, rear left, rear right, rear left}
     const float rotation = clamp(twist.angular * axisLength, -maxSpeed, maxSpeed);
     const float longitudinal = clamp(twist.longitudinal, -maxSpeed, maxSpeed);
     const float lateral = clamp(twist.longitudinal, -maxSpeed, maxSpeed);
     float front_left = longitudinal - lateral - rotation;
     float front_right = longitudinal + lateral + rotation;
     float rear_left = longitudinal + lateral - rotation;
     float rear_right = longitudinal - lateral + rotation;
     if (abs(front_left) > maxSpeed) {
       front_left = clamp(front_left, -maxSpeed, maxSpeed);
       front_right = front_left + 2 * lateral + 2 * rotation;
       rear_left = front_left + 2 * lateral;
       rear_right = front_left + 2 * rotation;
     } else if (abs(front_right) > maxSpeed) {
       front_right = clamp(front_right, -maxSpeed, maxSpeed);
       front_left = front_right - 2 * lateral - 2 * rotation;
       rear_left = front_right - 2 * rotation;
       rear_right = front_right - 2 * lateral;
     } else if (abs(rear_left) > maxSpeed) {
       rear_left = clamp(rear_left, -maxSpeed, maxSpeed);
       front_left = rear_left - 2 * lateral;
       front_right = rear_left + 2 * rotation;
       rear_right = rear_left - 2 * rotation + 2 * rotation;
     } else if (abs(rear_right) > maxSpeed) {
       rear_right = clamp(rear_right, -maxSpeed, maxSpeed);
       front_left = rear_right - 2 * rotation;
       front_right = rear_right + 2 * lateral;
       rear_left = rear_right + 2 * lateral - 2 * rotation;
     }
     return {front_left, rear_left, rear_right, front_right};
  }
  return {};
}

CVector2 Agent::get_target_velocity() const {
  CVector2 v = CVector2(target_twist.longitudinal, target_twist.lateral);
  return rotate(v, angle);
}

void Agent::set_wheel_speeds(const WheelSpeeds & speeds) {
  Twist2D twist = twist_from_wheel_speeds(speeds);
  CVector2 v = CVector2(twist.longitudinal, twist.lateral);
  velocity = rotate(v, angle);
  angularSpeed = CRadians(twist.angular);
}

Twist2D Agent::compute_desired_twist() const {
  float delta_angle = 0.0;
  if (heading_behavior == DESIRED_ANGLE) {
    delta_angle = normalize((polar_angle(desiredVelocity)- angle));
  } else if (heading_behavior == TARGET_ANGLE) {
    delta_angle = normalize(targetAngle - angle);
  } else if (heading_behavior == TARGET_POINT) {
    delta_angle = normalize(polar_angle(targetPosition - position) - angle);
  }
  float angular_speed = (1.0 / rotationTau) * delta_angle;
  if (is_omnidirectional()) {
    auto v = rotate(desiredVelocity, -angle);
    return Twist2D(v.x(), v.y(), angular_speed);
  }
  // TODO(Jerome): wrong!!! ... check how it was before
  return Twist2D(desiredVelocity.norm(), 0.0, angular_speed);
}

bool Agent::is_wheeled() const {
  return type == TWO_WHEELED || type == FOUR_WHEELED_OMNI;
}

bool Agent::is_omnidirectional() const {
  return type == HOLONOMIC || type == FOUR_WHEELED_OMNI;
}

double Agent::get_rotation_tau() const { return rotationTau; }
void Agent::set_rotation_tau(double value) { rotationTau = value; }
double Agent::get_safety_margin() const { return safetyMargin; }
void Agent::set_safety_margin(double value) { safetyMargin = std::max(0.0, value); }
double Agent::get_horizon() const { return horizon; }
void Agent::set_horizon(double value) { horizon = std::max(0.0, value); }
double Agent::get_max_speed() const { return maxSpeed; }
void Agent::set_max_speed(double value) {
  maxSpeed = value;
  if (is_wheeled()) {
    maxAngularSpeed = CRadians(maxSpeed / axisLength);
  }
}
CRadians Agent::get_max_angular_speed() const { return maxAngularSpeed; }
void Agent::set_max_angular_speed(double value) {
  if (is_wheeled()) return;
  maxAngularSpeed = CRadians(value);
}
double Agent::get_optimal_speed() const { return optimalSpeed;}
void Agent::set_optimal_speed(double value) {
  optimalSpeed = clamp<double>(value, 0.0, maxSpeed);
}
CRadians Agent::get_optimal_angular_speed() const { return optimalAngularSpeed;}
void Agent::set_optimal_angular_speed(double value) {
  optimalAngularSpeed = CRadians(clamp<double>(value, 0.0, maxAngularSpeed));
}

void Agent::update(float dt) {
  prepare();
  clear();
  for (auto const & d : static_obstacles) {
    add_static_obstacle(d);
  }
  for (auto const & d : neighbors) {
    add_neighbor(d);
  }
  update_desired_velocity();
  set_desired_twist(compute_desired_twist());
  update_target_twist(dt);
}

void Agent::set_desired_twist(const Twist2D & twist) {
  if (is_wheeled()) {
    desired_wheel_speeds = wheel_speeds_from_twist(twist);
    desired_twist = twist_from_wheel_speeds(desired_wheel_speeds);
  } else {
    // TODO(Jerome): Should clamp here too
    desired_twist = twist;
  }
}

void Agent::update_target_twist(float dt) {
  target_wheel_speeds = desired_wheel_speeds;
  target_twist = desired_twist;
}

CVector2 Agent::relativePositionOfObstacleAt(CVector2 &obstaclePosition,
                                             Real obstacleRadius,
                                             Real &distance) {
  CVector2 relativePosition = obstaclePosition - position;
  distance = relativePosition.norm();
  Real minDistance = (radius + obstacleRadius) + 0.002;
  if (distance < minDistance) {
    // too near,  cannot penetrate in an obstacle (footbot or human)
    relativePosition = relativePosition / distance * minDistance;
    obstaclePosition = position + relativePosition;
    distance = minDistance;
  }
  return relativePosition;
}

Real Agent::marginForObstacleAtDistance(Real distance, Real obstacleRadius,
                                        Real safetyMargin, Real socialMargin) {
  //      printf("margin: dist %.2f or %.2f sM %.2f SM
  //      %.2f\n",distance,obstacleRadius,safetyMargin,socialMargin);

  // socialMargin is now a property of the obstacle!!, so could be less than
  // safety margin
  socialMargin = std::max(socialMargin, safetyMargin);

  double farMargin = 1;
  double distanceToBeSeparated = safetyMargin + radius + obstacleRadius;
  double distanceToBeFar = farMargin + radius + obstacleRadius;

  if (distance < distanceToBeSeparated) {
    return safetyMargin;
  } else if (distance > distanceToBeFar) {
    return socialMargin;
  } else {
    return (socialMargin - safetyMargin) /
               (distanceToBeFar - distanceToBeSeparated) *
               (distance - distanceToBeSeparated) +
           safetyMargin;
  }
}

std::map<std::string, Agent::CreateMethod> Agent::_agent_create_functions = {};
