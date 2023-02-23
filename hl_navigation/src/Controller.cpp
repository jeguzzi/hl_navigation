/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "hl_navigation/controller.h"

namespace hl_navigation {

float FollowTwistAction::tick(Controller* controller, float dt) {
  return 0.0;
}

float FollowAction::tick(Controller* controller, float dt) {
  float distance = controller->time_to_target();
  if (has_target_orientation) {
    distance = std::max(distance, controller->time_to_target_orientation());
  }
  return distance;
}

float MoveAction::tick(Controller* controller, float dt) {
  float distance = 0.0f;
  float duration = 0.0f;
  if (move_state == Behavior::Mode::move) {
    distance = controller->distance_to_target();
    duration = controller->time_to_target(position_tolerance);
    if (position_tolerance < 0 || distance < position_tolerance) {
      move_state = Behavior::Mode::turn;
    } else if (has_target_orientation) {
      duration = std::max(duration, controller->time_to_target_orientation());
    }
  }
  if (move_state == Behavior::Mode::turn) {
    if (!has_target_orientation || orientation_tolerance < 0) {
      move_state = Behavior::Mode::stop;
    } else {
      distance = controller->distance_to_target_orientation();
      duration = controller->time_to_target_orientation(orientation_tolerance);
      if (distance < orientation_tolerance) {
        move_state = Behavior::Mode::stop;
      }
    }
  }
  if (move_state == Behavior::Mode::stop) {
    if (controller->is_still()) {
      state = State::success;
    }
  }
  return distance;
}

}  // namespace hl_navigation
