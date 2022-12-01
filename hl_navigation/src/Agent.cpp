/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "Agent.h"

void Agent::stop() {
  desiredSpeed = 0.0;
  desiredAngle = CRadians::ZERO;
}

std::tuple<float, float> Agent::velocity_from_wheel_speed(float left, float right) {
  return std::make_tuple<float, float>((left + right) * 0.5, (right - left) * 0.5 / axisLength);
}

std::tuple<float, float> Agent::wheel_speed_from_velocity(float linear_speed, float angular_speed) {
  return std::make_tuple<float, float>(
      linear_speed - angular_speed * axisLength, linear_speed + angular_speed * axisLength);
}

// Called after updating the left-right wheel speeds

void Agent::setDesiredWheelSpeeds(double left, double right) {
  rightWheelDesiredSpeed = right;
  leftWheelDesiredSpeed = left;
  desiredAngularSpeed = CRadians(
      (rightWheelDesiredSpeed - leftWheelDesiredSpeed) * 0.5 / axisLength);
  desiredLinearSpeed = (rightWheelDesiredSpeed + leftWheelDesiredSpeed) * 0.5;
  desiredVelocity = CVector2(desiredLinearSpeed, desiredAngularSpeed);
}

void Agent::setRotationTau(double value) { rotationTau = value; }
void Agent::setSafetyMargin(double value) { safetyMargin = value; }

void Agent::setHorizon(double value) { horizon = value; }

void Agent::setMaxSpeed(double value) { maxSpeed = value; }

void Agent::setMaxAngularSpeed(double value) {
  maxAngularSpeed = CRadians(value);
  if (type == TWO_WHEELED) {
    maxRotationSpeed = value * 0.5 * axisLength;
  }
}

void Agent::setMaxRotationSpeed(double value) {
  maxRotationSpeed = value;
  if (type == TWO_WHEELED) {
    maxAngularSpeed = CRadians(value * 2.0 / axisLength);
  }
}

void Agent::setOptimalSpeed(double value) {
  if (value < maxSpeed && value >= 0) {
    optimalSpeed = value;
  } else {
    optimalSpeed = maxSpeed;
  }
}

void Agent::setOptimalAngularSpeed(double value) {
  if (value < maxAngularSpeed.GetAbsoluteValue() && value >= 0) {
    optimalAngularSpeed = CRadians(value);
  } else {
    optimalAngularSpeed = maxAngularSpeed;
  }
  if (type == TWO_WHEELED) {
    optimalRotationSpeed = optimalAngularSpeed.GetValue() * 0.5 * axisLength;
  }
}

void Agent::setOptimalRotationSpeed(double value) {
  if (value < maxRotationSpeed && value >= 0) {
    optimalRotationSpeed = value;
  } else {
    optimalRotationSpeed = maxRotationSpeed;
  }
  if (type == TWO_WHEELED) {
    optimalAngularSpeed = CRadians(optimalRotationSpeed * 2.0 / axisLength);
  }
}

void Agent::updateVelocity(float dt) {
  CRadians delta = desiredAngle.SignedNormalize();
  desiredAngularSpeed = (1.0 / rotationTau) * delta;

  if (type == TWO_WHEELED) {
    desiredLinearSpeed = 0;
    Real targetAngularMotorSpeed =
        desiredAngularSpeed.GetValue() * 0.5 * axisLength;

    if (targetAngularMotorSpeed > optimalRotationSpeed) {
      targetAngularMotorSpeed = optimalRotationSpeed;
      desiredAngularSpeed =
          CRadians(targetAngularMotorSpeed * 2.0 / axisLength);
    } else if (targetAngularMotorSpeed < -optimalRotationSpeed) {
      targetAngularMotorSpeed = -optimalRotationSpeed;
      desiredAngularSpeed =
          CRadians(targetAngularMotorSpeed * 2.0 / axisLength);
    } else {
      if (linearSpeedIsContinuos) {
        desiredLinearSpeed = desiredSpeed * (1 - fabs(targetAngularMotorSpeed) /
                                                     maxRotationSpeed);
      } else {
        desiredLinearSpeed = desiredSpeed;
      }
    }

    // #ifdef ANGULAR_SPEED_DOMINATE
    if (fabs(desiredLinearSpeed) + fabs(targetAngularMotorSpeed) > maxSpeed) {
      if (desiredLinearSpeed < 0) {
        desiredLinearSpeed = -maxSpeed + fabs(targetAngularMotorSpeed);
      } else {
        desiredLinearSpeed = maxSpeed - fabs(targetAngularMotorSpeed);
      }
    }

    // #endif

    leftWheelDesiredSpeed = desiredLinearSpeed - targetAngularMotorSpeed;
    rightWheelDesiredSpeed = desiredLinearSpeed + targetAngularMotorSpeed;

    // TODO(Jerome): do not duplicate velocity and wheel speeds (should use computed properties)
    desiredVelocity = CVector2(desiredLinearSpeed, 0.0);

  } else {
    if (desiredAngularSpeed > optimalAngularSpeed) {
      desiredAngularSpeed = optimalAngularSpeed;
    } else if (desiredAngularSpeed < -optimalAngularSpeed) {
      desiredAngularSpeed = -optimalAngularSpeed;
    }
    desiredLinearSpeed = desiredSpeed;

    desiredVelocity = CVector2(desiredLinearSpeed, desiredAngle);
  }

  // printf("%p TV L:%.3f,
  // R:%.3f\n",this,leftWheelDesiredSpeed,rightWheelDesiredSpeed);
}

Agent::Agent() {
  type = TWO_WHEELED;
  maxSpeed = MAX_SPEED;
  leftWheelSpeed = 0.0;
  rightWheelSpeed = 0.0;
  linearSpeedIsContinuos = DEFAULT_LINEAR_SPEED_CONTINUOUS;
  //    maxRotationSpeed=DEFAULT_MAX_ANGULAR_SPEED;
  radius = RADIUS;
  rotationTau = DEFAULT_ROTATION_TAU;
  setOptimalSpeed(DEFAULT_OPTIMAL_SPEED);
  setOptimalRotationSpeed(DEFAULT_OPTIMAL_SPEED);

  //    socialRadius[HUMAN]=DEFAULT_HUMAN_SOCIAL_RADIUS;
  // socialRadius[FOOTBOT]=DEFAULT_FOOTBOT_SOCIAL_RADIUS;
  // socialRadius[OBSTACLE]=DEFAULT_OBSTACLE_SOCIAL_RADIUS;
  safetyMargin = 0.1;
  // socialMargin=safetMargin;
  // ratioOfSocialRadiusForSensing=DEFAULT_SOCIAL_SENSING_RATIO;

  horizon = DEFAULT_HORIZON;
}

CVector2 Agent::relativePositionOfObstacleAt(CVector2 &obstaclePosition,
                                             Real obstacleRadius,
                                             Real &distance) {
  CVector2 relativePosition = obstaclePosition - position;
  distance = relativePosition.Length();
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
