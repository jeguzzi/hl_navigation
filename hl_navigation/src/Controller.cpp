/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */
#include "Controller.h"

void Controller::set_target_point(float x, float y, float z) {
  target_type = POINT;
  agent->targetPosition = CVector2(x, y);
  targetZ = z;
  state = MOVE;
}

void Controller::set_pose(float x, float y, float z_, float theta) {
  z = z_;
  agent->position = CVector2(x, y);
  agent->angle = CRadians(theta);
}

void Controller::set_target_pose(float x, float y, float z, float theta) {
  target_type = POSE;
  agent->targetPosition = CVector2(x, y);
  agent->targetAngle = CRadians(theta);
  targetZ = z;
  state = MOVE;
}

void Controller::turn() { state = TURN; }

void Controller::brake() {
  if (state != BRAKING) {
    agent->set_desired_twist(Twist2D(0.0, 0.0, 0.0));
    state = BRAKING;
  }
}

void Controller::stop() {
  if (state != IDLE) {
    state = IDLE;
    agent->target_twist = Twist2D(0.0, 0.0, 0.0);
    set_target_twist(agent->target_twist, 0.0);
    aborted();
  }
}

bool Controller::is_at_target_point() {
  targetDistance = (agent->targetPosition - agent->position).Length();
  return targetDistance < distance_tolerance;
}

bool Controller::is_at_target_angle() {
  if (target_type == POINT)
    return true;
  return (agent->targetAngle - agent->angle).GetAbsoluteValue() < angle_tolerance;
}

void Controller::update_target_state() {
  switch (state) {
  case MOVE:
    if (is_at_target_point()) {
      if (is_at_target_angle()) {
          arrived();
          brake();
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

void Controller::update_vertical_velocity() {
  // TODO(old) complete with obstacle avoidance
  double desiredVerticalSpeed = (targetZ - z) / tauZ;
  if (desiredVerticalSpeed > optimalVerticalSpeed)
    desiredVerticalSpeed = optimalVerticalSpeed;
  else if (desiredVerticalSpeed < -optimalVerticalSpeed)
    desiredVerticalSpeed = -optimalVerticalSpeed;
  velocityZ = velocityZ + (desiredVerticalSpeed - velocityZ) / tauZ;
}

void Controller::set_target_twist(const Twist2D & twist, float vertical_speed) { }

void Controller::update(float dt) {
  if (state == IDLE)
    return;
  if (!localized() && state != BRAKING) {
    brake();
  } else {
    update_target_state();
  }
  if (state == BRAKING && agent->velocity.Length() < speed_tolerance) {
    stop();
    return;
  }
  if (state == MOVE) {
    agent->update(dt);
  } else if (state == TURN) {
    float delta = 0.0;
    if (target_type == POSE) {
      delta = (agent->targetAngle - agent->angle).SignedNormalize().GetValue();
    }
    agent->set_desired_twist(Twist2D(0.0, 0.0, delta / agent->get_rotation_tau()));
    agent->update_target_twist(dt);
  } else {
    agent->update_target_twist(dt);
  }

  //   if(agent->type!=TWO_WHEELED && agent->insideObstacle)
  //   {
  //           //TODO move away from penetrated obstacles, non so easy to avoid
  //           livelocks. This is far more relevant in 2.5 D because top/bottom
  //           margin of obstacles can be penetrated more easely.
  //           exit_from_obstacle()
  //   }

  update_vertical_velocity();
  set_target_twist(agent->target_twist, velocityZ);
  updated_control();
}

Controller::Controller() : state(IDLE) {}

Controller::~Controller() { stop(); }
