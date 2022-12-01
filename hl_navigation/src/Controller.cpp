/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */
#include "Controller.h"

void Controller::setTargetPoint(float x, float y, float z) {
  targetType = POINT;
  agent->targetPosition = CVector2(x, y);
  targetZ = z;
  state = MOVE;
}

void Controller::setPose(float x, float y, float z_, float theta) {
  z = z_;
  agent->position = CVector2(x, y);
  agent->angle = CRadians(theta);
}


void Controller::setTargetPose(float x, float y, float z, float theta) {
  targetType = POSE;
  agent->targetPosition = CVector2(x, y);
  targetZ = z;
  targetAngle = theta;
  agent->desiredAngle = CRadians(targetAngle) - agent->angle;
  state = MOVE;
}

void Controller::turn() {
  state = TURN;
  agent->desiredSpeed = 0.0;
}

void Controller::brake() {
  if (state != BRAKING) {
    state = BRAKING;
  }
}

void Controller::stop() {
  if (state != IDLE) {
    state = IDLE;
    setMotorVelocity(0, 0, 0);
    aborted();
  }
}

bool Controller::is_at_target_point() {
  targetDistance = (agent->targetPosition - agent->position).Length();
  return targetDistance < minDeltaDistance;
}

bool Controller::is_at_target_angle() {
  if (targetType == POINT)
    return true;
  if (state == MOVE) {
    return (agent->desiredAngle).GetAbsoluteValue() < minDeltaAngle;
  } else {
    double da = (CRadians(targetAngle) - agent->angle).GetAbsoluteValue();
    return da < minDeltaAngle;
  }
}

void Controller::updateTargetState() {
  switch (state) {
  case MOVE:
    if (is_at_target_point()) {
      if (agent->type == HOLONOMIC) {
        if (is_at_target_angle()) {
          arrived();
          brake();
        }
      } else {
        turn();
      }
    } else {
      updated();
    }
    break;
  case TURN:
    if (is_at_target_angle()) {
      arrived();
      brake();
    }
    break;
  default:
    break;
  }
}

void Controller::updateVerticalVelocity() {
  // TODO(old) complete with obstacle avoidance
  double desiredVerticalSpeed = (targetZ - z) / tauZ;
  if (desiredVerticalSpeed > optimalVerticalSpeed)
    desiredVerticalSpeed = optimalVerticalSpeed;
  else if (desiredVerticalSpeed < -optimalVerticalSpeed)
    desiredVerticalSpeed = -optimalVerticalSpeed;
  velocityZ = velocityZ + (desiredVerticalSpeed - velocityZ) / tauZ;
}

void Controller::setMotorVelocity(double newSpeed, double newVelocityZ,
                                 double newAngularSpeed) {
  setMotorVelocity(newSpeed, 0, newVelocityZ, newAngularSpeed);
}

void Controller::update(float dt) {
  if (!localized() && state != IDLE && state != BRAKING) {
    brake();
    return;
  }
  if (state == IDLE)
    return;
  updateTargetState();
  if (state == BRAKING) {
    if (agent->velocity.Length() > minimalSpeed) {
      agent->desiredSpeed = 0.0;
      agent->desiredVelocity = CVector2(0, 0);
      agent->desiredAngle = CRadians(targetAngle) - agent->angle;
    } else {
      stop();
      return;
    }
  }
  if (state == MOVE) {
    agent->updateDesiredVelocity();
    agent->updateRepulsiveForce();
  }
  if (state == TURN) {
    agent->desiredSpeed = 0.0;
    agent->desiredVelocity = CVector2(0, 0);
    agent->desiredAngle = CRadians(targetAngle) - agent->angle;
  }

  //   if(agent->type!=TWO_WHEELED && agent->insideObstacle)
  //   {
  //           //TODO move away from penetrated obstacles, non so easy to avoid
  //           livelocks. This is far more relevant in 2.5 D because top/bottom
  //           margin of obstacles can be penetrated more easely.
  //           exit_from_obstacle()
  //   }

  agent->updateVelocity(dt);
  updateVerticalVelocity();
  if (agent->type == TWO_WHEELED || agent->type == HEAD) {
    setMotorVelocity(agent->desiredLinearSpeed, velocityZ,
                     (agent->desiredAngularSpeed).GetValue());
  } else if (agent->type == HOLONOMIC) {
    CRadians desiredAngularSpeed;
    if (rotateIfHolo) {
      CRadians delta, tAngle;
      if (targetType == POSE) {
        tAngle = CRadians(targetAngle);
      } else {
        CVector2 agentToTarget = agent->targetPosition - agent->position;
        tAngle = agentToTarget.Angle();
      }
      delta = (tAngle - agent->angle).SignedNormalize();
      desiredAngularSpeed = (1.0 / agent->rotationTau) * delta;
    } else {
      desiredAngularSpeed = CRadians(0);
    }
    setMotorVelocity(agent->desiredVelocity.GetX(),
                     agent->desiredVelocity.GetY(), velocityZ,
                     desiredAngularSpeed.GetValue());
  }
  updated();
}


Controller::Controller() : state(IDLE) { }

Controller::~Controller() { stop(); }
