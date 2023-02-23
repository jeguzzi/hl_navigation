/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_CONTROLLER_H_
#define HL_NAVIGATION_CONTROLLER_H_

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

#include "hl_navigation/behavior.h"

namespace hl_navigation {

class Controller;

/**
 * @brief      Holds the state of a long running task
 * and notifies observers when it terminates.
 */
struct Action {
  /**
   * @brief      The state of an action.
   */
  enum class State { idle, running, failure, success };

  /**
   * The state
   */
  State state;

  Action() : state(State::idle), running_cb(), done_cb() {}

  void update(Controller *controller, float dt) {
    if (state == State::running) {
      const float progress = tick(controller, dt);
      if (done()) {
        if (done_cb) (*done_cb)(state);
      } else {
        if (running_cb) (*running_cb)(progress);
      }
    }
  }

  /**
   * A callback called when the action is running
   *
   * The callback argument is the minimal expected time to terminate the action.
   */
  std::optional<std::function<void(float)>> running_cb;
  /**
   * A callback called when the action terminates
   *
   * The callback argument is the final state of the action.
   */
  std::optional<std::function<void(State state)>> done_cb;

  bool done() const {
    return state == State::failure || state == State::success;
  }

  bool running() const { return state == State::running; }

  void abort() {
    if (state == State::running) {
      state = State::failure;
      if (done_cb) (*done_cb)(state);
    }
  }

  virtual float tick(Controller *controller, float dt) = 0;

  virtual Behavior::Mode mode() { return Behavior::Mode::stop; }
  //  virtual ~Action() = default;
  virtual ~Action() { abort(); }
};

struct FollowTwistAction : public Action {
  using Action::Action;

  float tick(Controller *controller, float dt) override;
  Behavior::Mode mode() override { return Behavior::Mode::follow; };
};

struct FollowAction : Action {
  bool has_target_orientation;

  FollowAction(bool has_target_orientation)
      : Action(), has_target_orientation(has_target_orientation) {}

  float tick(Controller *controller, float dt) override;

  Behavior::Mode mode() override { return Behavior::Mode::follow; };
};

struct MoveAction : Action {
  Behavior::Mode move_state;
  bool has_target_orientation;
  float position_tolerance;
  float orientation_tolerance;

  MoveAction(bool has_target_orientation, float position_tolerance,
             float orientation_tolerance)
      : Action(),
        move_state(Behavior::Mode::move),
        has_target_orientation(has_target_orientation),
        position_tolerance(position_tolerance),
        orientation_tolerance(orientation_tolerance) {}

  float tick(Controller *controller, float dt) override;

  Behavior::Mode mode() override { return move_state; }
};

/**
 * @brief      Exposes a higher level, stateful interface to a
 * behavior, to which it delegates 2D collision avoidance.
 *
 * It provides actions (event-based interfaces) to
 *  - go to a point/pose, stopping once the target has been reached,
 *  - follow a point/pose, which does not terminates once the target has been
 * reached
 *  - follow a velocity/twist
 * It automatically switches between different
 * Behavior::Mode, possibly making the agents turns towards target orientation
 * after a target position has been reached.
 *
 * Typical usage:
 * 1. Pick, configure a Behavior
 * 2. Initialize and configure the controller
 * 3. at regular intervals update the state, using the Behavior API, and call
 * \ref update.
 * 4. Trigger one of the actions.
 * 5. Either manually check the action's \ref get_state or set callbacks to be
 * notified when the action terminates or updates.
 * 6. Actuate the target command by using the return value of \ref update
 *    or by setting a callback \ref set_cmd_cb
 */
class Controller {
 public:
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  behavior                            The navigation behavior
   * @param[in]  compute_relative_twist              The compute relative twist
   * @param[in]  set_twist_as_automatically_actuated  Indicates if set twist as
   * automatically actuated
   */
  Controller(std::shared_ptr<Behavior> behavior = nullptr,
             bool compute_relative_twist = true,
             bool set_twist_as_automatically_actuated = true)
      : behavior(behavior),
        compute_relative_twist(compute_relative_twist),
        set_twist_as_automatically_actuated(
            set_twist_as_automatically_actuated) {}

  virtual ~Controller() = default;

  virtual float distance_to_target() const {
    // Expect[tolerance >= 0];
    if (behavior) {
      return (behavior->get_target_position() - behavior->get_position())
          .norm();
    }
    return 0.0f;
  }

  virtual float distance_to_target_orientation() const {
    // Expect[tolerance >= 0];
    if (behavior) {
      return abs(normalize(behavior->get_target_orientation() -
                           behavior->get_orientation()));
    }
    return 0.0f;
  }

  virtual float time_to_target(float tolerance = 0.0f) const {
    // Expect[tolerance >= 0];
    if (behavior) {
      const float distance = distance_to_target();
      const float speed = behavior->get_optimal_speed();
      if (speed > 0) {
        return (distance - tolerance) / speed;
      } else {
        return std::numeric_limits<float>::infinity();
      }
    }
    return 0.0f;
  }

  virtual float time_to_target_orientation(float tolerance = 0.0f) const {
    // Expect[tolerance >= 0];
    if (behavior) {
      const float distance = distance_to_target_orientation();
      const float speed = behavior->get_optimal_angular_speed();
      if (speed > 0) {
        return (distance - tolerance) / speed;
      } else {
        return std::numeric_limits<float>::infinity();
      }
    }
    return 0.0f;
  }

  virtual bool is_still() const {
    if (behavior) {
      return behavior->get_speed() < speed_tolerance;
    }
    return true;
  }

  /**
   * @brief      Gets the state of the control action.
   *
   * @return     The state.
   */
  Action::State get_state() const {
    if (action) {
      return action->state;
    }
    return Action::State::idle;
  }

  /**
   * @brief      Gets the navigation behavior used by the controller
   *
   * @return     The navigation behavior.
   */
  std::shared_ptr<Behavior> get_behavior() const { return behavior; }
  /**
   * @brief      Sets the navigation behavior used by the controller
   *
   * @param[in]  value  The navigation behavior
   */
  void set_behavior(std::shared_ptr<Behavior> value) { behavior = value; }
  /**
   * @brief      Gets the minimal speed to consider the agent as stopped.
   *
   * @return     The speed tolerance.
   */
  float get_speed_tolerance() const { return speed_tolerance; }
  /**
   * @brief      Sets the minimal speed to consider the agent as stopped.
   *
   * @param[in]  value  The speed tolerance
   */
  void set_speed_tolerance(float value) {
    speed_tolerance = std::max(0.0f, value);
  }
  /**
   * @brief      Starts an action to go to a point
   *
   * The action succeed once the agent arrives at within the tolerance
   * from the target point and come to a stop.
   * The controller aborts the currently running action.
   *
   * @param[in]  point              The target point
   * @param[in]  tolerance  The spatial tolerance
   *
   * @return     The new action.
   */
  std::shared_ptr<Action> go_to_position(const Vector2 &point,
                                         float tolerance) {
    if (action) {
      action->abort();
    }
    if (behavior) {
      behavior->set_target_position(point);
    }
    action = std::make_shared<MoveAction>(false, tolerance, -1.0f);
    action->state = Action::State::running;
    action->update(this, 0.0);
    return action;
  }
  /**
   * @brief      Starts an action to go to a pose
   *
   * The action succeed once the agent arrives at within the tolerances
   * from the target pose and come to a stop.
   * The controller aborts the currently running action.
   *
   * @param[in]  pose              The target pose
   * @param[in]  position_tolerance  The spatial tolerance
   * @param[in]  orientation_tolerance  The spatial tolerance
   *
   * @return     The new action.
   */
  std::shared_ptr<Action> go_to_pose(const Pose2 &pose,
                                     float position_tolerance,
                                     float orientation_tolerance) {
    if (action) {
      action->abort();
    }
    if (behavior) {
      behavior->set_target_position(pose.position);
      behavior->set_target_orientation(pose.orientation);
    }
    action = std::make_shared<MoveAction>(true, position_tolerance,
                                          orientation_tolerance);
    action->state = Action::State::running;
    action->update(this, 0.0);
    return action;
  }
  /**
   * @brief      Starts an action to follow a point
   *
   * The action keeps running even after the agent arrive near to the target
   * point. The controller aborts the currently running action if not already
   * following a point/pose, or updates the target pose.
   *
   * @param[in]  point              The target point
   *
   * @return     The new action.
   */
  std::shared_ptr<Action> follow_point(const Vector2 &point) {
    if (!std::dynamic_pointer_cast<FollowAction>(action)) {
      if (action) {
        action->abort();
      }
      action = std::make_shared<FollowAction>(false);
      action->state = Action::State::running;
      action->update(this, 0.0);
    }
    if (behavior) {
      behavior->set_target_position(point);
    }
    return action;
  }
  /**
   * @brief      Starts an action to follow a pose
   *
   * The action keeps running even after the agent arrive near to the target
   * point. The controller aborts the currently running action if not already
   * following a point/pose, or updates the target pose.
   *
   * @param[in]  pose              The target pose
   *
   * @return     The new action.
   */
  std::shared_ptr<Action> follow_pose(const Pose2 &pose) {
    if (!std::dynamic_pointer_cast<FollowAction>(action)) {
      if (action) {
        action->abort();
      }
      action = std::make_shared<FollowAction>(false);
      action->state = Action::State::running;
      action->update(this, 0.0);
    }
    if (behavior) {
      behavior->set_target_position(pose.position);
      behavior->set_target_orientation(pose.orientation);
    }
    return action;
  }
  /**
   * @brief      Starts an action to follow a velocity
   *
   * The controller aborts the currently running action if not already following
   * a velocity/twist, or updates the target pose.
   *
   * @param[in]  velocity              The target velocity
   *
   * @return     The new action.
   */
  std::shared_ptr<Action> follow_velocity(const Vector2 &velocity) {
    if (!std::dynamic_pointer_cast<FollowAction>(action)) {
      if (action) {
        action->abort();
      }
      action = std::make_shared<FollowTwistAction>();
      action->state = Action::State::running;
      action->update(this, 0.0);
    }
    if (behavior) {
      behavior->set_target_velocity(velocity);
    }
    return action;
  }
  /**
   * @brief      Starts an action to follow a twist
   *
   * The controller aborts the currently running action if not already following
   * a velocity/twist, or updates the target pose.
   *
   * @param[in]  twist              The target twist
   *
   * @return     The new action.
   */
  std::shared_ptr<Action> follow_twist(const Twist2 &twist) {
    if (!std::dynamic_pointer_cast<FollowAction>(action)) {
      if (action) {
        action->abort();
      }
      action = std::make_shared<FollowTwistAction>();
      action->state = Action::State::running;
      action->update(this, 0.0);
    }
    if (behavior) {
      behavior->set_target_velocity(twist.velocity);
      // TODO(Jerome): complete
    }
    return action;
  }
  /**
   * @brief      Returns if the control action is idle.
   *
   * @return     True only if the control action is not running.
   */
  bool idle() const { return action == nullptr || !action->running(); }

  /**
   * @brief      Abort running action.
   */
  void stop() {
    if (action) {
      action->abort();
      action = nullptr;
    }
  }

  /**
   * @brief      Updates the control for time step.
   *
   * Internally calls \ref Behavior::cmd_twist for collision avoidance.
   *
   * @param[in]  time_step  The time step
   *
   * @return     The command twist to actuate the current action
   */
  Twist2 update(float time_step) {
    if (action) {
      action->update(this, time_step);
      if (action && action->done()) {
        action = nullptr;
      }
    }
    if (action && behavior) {
      const Behavior::Mode mode =
          action->running() ? action->mode() : Behavior::Mode::stop;
      const Twist2 cmd =
          behavior->cmd_twist(time_step, compute_relative_twist, mode,
                              set_twist_as_automatically_actuated);
      if (cmd_cb) {
        (*cmd_cb)(cmd);
      }
      return cmd;
    }
    return {};
  }

  /**
   * @brief      Sets the callback called each time a command is computed for an
   * active action.
   *
   * @param[in]  value  The callback that takes the command twist as argument.
   */
  void set_cmd_cb(std::function<void(const Twist2 &)> value) { cmd_cb = value; }

 protected:
  std::shared_ptr<Action> action;
  std::shared_ptr<Behavior> behavior;
  /**
   * Speed tolerance to transition to idle
   */
  float speed_tolerance;
  bool compute_relative_twist;
  /**
   * Specify if the command returned by \ref update can be considered as
   * accepted/actuated, see the corresponding argument of \ref
   * Behavior::target_twist (default: True)
   */
  bool set_twist_as_automatically_actuated;

 private:
  std::optional<std::function<void(const Twist2 &)>> cmd_cb;
};

}  // namespace hl_navigation

#endif /* HL_NAVIGATION_CONTROLLER_H_ */
