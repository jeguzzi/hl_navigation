/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */
#include "controller.h"

namespace hl_navigation {

void Controller::set_target_point(float x, float y, float z) {
  target_type = POINT;
  behavior->targetPosition = Vector2(x, y);
  targetZ = z;
  state = MOVE;
}

// TODO(J 2023): Complete 2D and 3D. Ignore 3D if setting target point 2D
void Controller::set_target_point(const Vector2 & point) {
  target_type = POINT;
  behavior->targetPosition = point;
  state = MOVE;
}

void Controller::set_pose(float x, float y, float z_, Radians theta) {
  z = z_;
  behavior->position = Vector2(x, y);
  behavior->angle = theta;
}

void Controller::set_target_pose(float x, float y, float z, Radians theta) {
  target_type = POSE;
  behavior->targetPosition = Vector2(x, y);
  behavior->targetAngle = theta;
  targetZ = z;
  state = MOVE;
}

void Controller::turn() { state = TURN; }

void Controller::brake() {
  if (state != BRAKING) {
    behavior->set_desired_twist(Twist2D(0.0, 0.0, 0.0));
    state = BRAKING;
  }
}

void Controller::stop() {
  if (state != IDLE) {
    state = IDLE;
    behavior->target_twist = Twist2D(0.0, 0.0, 0.0);
    set_target_twist(behavior->target_twist, 0.0);
    aborted();
  }
}

bool Controller::is_at_target_point() {
  targetDistance = (behavior->targetPosition - behavior->position).norm();
  return targetDistance < distance_tolerance;
}

bool Controller::is_at_target_angle() {
  if (target_type == POINT)
    return true;
  return abs(normalize(behavior->targetAngle - behavior->angle)) < angle_tolerance;
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
  float desiredVerticalSpeed = (targetZ - z) / tauZ;
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
  if (state == BRAKING && behavior->velocity.norm() < speed_tolerance) {
    stop();
    return;
  }
  if (state == MOVE) {
    behavior->update(dt);
  } else if (state == TURN) {
    float delta = 0.0;
    if (target_type == POSE) {
      delta = normalize(behavior->targetAngle - behavior->angle);
    }
    behavior->set_desired_twist(Twist2D(0.0, 0.0, delta / behavior->get_rotation_tau()));
    behavior->update_target_twist(dt);
  } else {
    behavior->update_target_twist(dt);
  }

  //   if(behavior->type!=TWO_WHEELED && behavior->insideObstacle)
  //   {
  //           //TODO move away from penetrated obstacles, non so easy to avoid
  //           livelocks. This is far more relevant in 2.5 D because top/bottom
  //           margin of obstacles can be penetrated more easely.
  //           exit_from_obstacle()
  //   }

  update_vertical_velocity();
  set_target_twist(behavior->target_twist, velocityZ);
  updated_control();
}

Controller::Controller() : state(IDLE) {}

Controller::~Controller() { stop(); }

}  // namespace hl_navigation
