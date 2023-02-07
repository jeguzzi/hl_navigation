/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */
#include "controller.h"

namespace hl_navigation {

void Controller::set_target_point(const Vector3 & xyz) {
  target_type = POINT;
  target_has_z = true;
  behavior->targetPosition = xyz.head<2>();
  targetZ = xyz[2];
  state = MOVE;
}

void Controller::set_target_point(const Vector2 & xy) {
  target_type = POINT;
  target_has_z = false;
  behavior->targetPosition = xy;
  state = MOVE;
}

void Controller::set_pose(const Vector2 & xy, Radians theta) {
  has_z = false;
  behavior->position = xy;
  behavior->angle = theta;
}

void Controller::set_pose(const Vector3 & xyz, Radians theta) {
  z = xyz[2];
  has_z = true;
  behavior->position = xyz.head<2>();
  behavior->angle = theta;
}

void Controller::set_target_pose(const Vector3 & xyz, Radians theta) {
  target_type = POSE;
  target_has_z = true;
  behavior->targetPosition = xyz.head<2>();
  behavior->targetAngle = theta;
  targetZ = xyz[2];
  state = MOVE;
}

void Controller::set_target_pose(const Vector2 & xy, Radians theta) {
  target_type = POSE;
  target_has_z = false;
  behavior->targetPosition = xy;
  behavior->targetAngle = theta;
  state = MOVE;
}

void Controller::set_neighbors(const std::vector<Disc> & neighbors) {
  behavior->set_neighbors(neighbors);
}

static bool overlaps(float a1, float a2, float b1, float b2) {
  return b1 <= a2 || a1 <= b2;
}

// TODO(J): filter becomes outdated when z or target z changes!!!

void Controller::set_neighbors(const std::vector<Cylinder> & neighbors) {
  std::vector<Disc> neighbors_2d;
  for (const auto & cylinder : neighbors) {
    if (in_3d() && cylinder.height > 0) {
      float z1, z2;
      if (target_has_z) {
        if (targetZ < z) {
          z1 = targetZ;
          z2 = z;
        } else {
          z2 = targetZ;
          z1 = z;
        }
      } else {
        z1 = z2 = z;
      }
      if (!overlaps(cylinder.position[2], cylinder.position[2] + cylinder.height, z1, z2)) {
          continue;
      }
    }
    neighbors_2d.push_back(cylinder.disc());
  }
  behavior->set_neighbors(neighbors_2d);
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
  if (in_3d() && target_has_z) {
  // TODO(old) complete with obstacle avoidance
    float desiredVerticalSpeed = (targetZ - z) / tauZ;
    if (desiredVerticalSpeed > optimalVerticalSpeed)
      desiredVerticalSpeed = optimalVerticalSpeed;
    else if (desiredVerticalSpeed < -optimalVerticalSpeed)
      desiredVerticalSpeed = -optimalVerticalSpeed;
    velocityZ = velocityZ + (desiredVerticalSpeed - velocityZ) / tauZ;
  } else {
    velocityZ = 0.0;
  }
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

Controller::Controller(bool limit_to_2d) :
  state(IDLE), limit_to_2d(limit_to_2d), has_z(false), target_has_z(false) {}

Controller::~Controller() { stop(); }

}  // namespace hl_navigation
