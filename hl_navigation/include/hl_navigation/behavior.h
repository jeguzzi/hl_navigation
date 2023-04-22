/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_BEHAVIOR_H_
#define HL_NAVIGATION_BEHAVIOR_H_

#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hl_navigation/common.h"
#include "hl_navigation/kinematics.h"
#include "hl_navigation/property.h"
#include "hl_navigation/register.h"
#include "hl_navigation/social_margin.h"
#include "hl_navigation/state.h"
#include "hl_navigation_export.h"

namespace hl_navigation {

/**
 * @brief      This class describes a generic behavior to reach a target
 * position avoiding collision. Users
 * should not instantiate this class (it's behavior just keeps the agent in
 * place) but one of it's concrete sub-classes.
 *
 * The following lists the typical usage of a behavior.
 *
 * *At initialization*
 *
 * 1. select the concrete behavior, the agent's size (agents are shaped like
 *    discs), and \ref Kinematics;
 *
 * 2. configure the generic parameters: \ref set_optimal_speed, \ref
 * set_horizon, \ref set_safety_margin \ref set_rotation_tau, and \ref
 * set_heading_behavior;
 *
 * 3. configure the specific parameters of the concrete behavior.
 *
 * *At regular time intervals*
 *
 * 1. update the agent's state with \ref set_pose and \ref set_twist (or other
 *    convenience methods)
 *
 * 2. update the state of the local environment for the concrete sub-class
 *
 * 3. update the target with \ref set_target_pose (or other
 *    convenience methods)
 *
 * 4. ask for a control commands by calling \ref cmd_twist
 *
 * 5. actuate the control commands [not part of this API]
 */
class HL_NAVIGATION_EXPORT Behavior : virtual public HasProperties,
                                      virtual public HasRegister<Behavior>,
                                      protected TrackChanges {
 public:
  using HasRegister<Behavior>::C;
  /**
   * @brief      Modes for \ref Behavior::cmd_twist
   */
  enum class Mode {
    move,   /**< move towards the target position */
    follow, /**< follow the target velocity */
    turn,   /**< turn-in-place towards the target orientation */
    stop    /**< stop */
  };

  /**
   * @brief      Different behavior variants for the angular motion
   */
  enum class Heading {
    idle,                 /**< never turn */
    target_point,         /**< turn towards the target position */
    target_angle,         /**< turn towards the target orientation */
    target_angular_speed, /**< follow a target angular speed */
    velocity /**< turn towards the velocity orientation. This is the only
                behavior available to constrained kinematics.*/
  };

  /**
   * @brief      Get a named \ref Heading mode
   *
   * @param      value  The name of the \ref Heading mode
   *
   * @return     The corresponding \ref Heading
   */
  static Heading heading_from_string(const std::string &value) {
    if (value == "target_point") return Heading::target_point;
    if (value == "target_angle") return Heading::target_angle;
    if (value == "target_angular_speed") return Heading::target_angular_speed;
    if (value == "velocity") return Heading::velocity;
    return Heading::idle;
  }

  /**
   * @brief      Get the name of an \ref Heading mode
   *
   * @param[in]  value  The \ref Heading mode
   *
   * @return     The name of the \ref Heading mode.
   */
  static std::string heading_to_string(const Heading &value) {
    if (value == Heading::target_point) return "target_point";
    if (value == Heading::target_angle) return "target_angle";
    if (value == Heading::target_angular_speed) return "target_angular_speed";
    if (value == Heading::velocity) return "velocity";
    return "idle";
  }

  /**
   * Default rotation tau
   */
  static constexpr float default_rotation_tau = 0.5f;
  /**
   * Default horizon
   */
  static constexpr float default_horizon = 5.0f;
  /**
   * Default safety margin
   */
  static constexpr float default_safety_margin = 0.0f;

  /**
   * @brief      Constructs a new instance.
   *
   * @param  kinematics  The kinematics of the agent.
   * @param  radius      The radius of the agent.
   */
  Behavior(std::shared_ptr<Kinematics> kinematics = nullptr,
           float radius = 0.0f)
      : TrackChanges(),
        social_margin(),
        kinematics(kinematics),
        radius(radius),
        pose(),
        twist(),
        horizon(default_horizon),
        safety_margin(default_safety_margin),
        optimal_speed(kinematics ? kinematics->get_max_speed() : 0.0f),
        optimal_angular_speed(kinematics ? kinematics->get_max_angular_speed()
                                         : 0.0f),
        rotation_tau(default_rotation_tau),
        heading_behavior(Heading::idle),
        target_pose(),
        target_twist() {}

  virtual ~Behavior() = default;

  //------------ AGENT PARAMETERS

  /**
   * @brief      Gets the kinematics.
   *
   * @return     The agent's kinematics.
   */
  std::shared_ptr<Kinematics> get_kinematics() const { return kinematics; }
  /**
   * @brief      Sets the kinematics.
   *
   * @param[in]  value The desired kinematics.
   */
  void set_kinematics(std::shared_ptr<Kinematics> value) {
    if (value) {
      if (!kinematics) {
        if (optimal_speed == 0.0f) {
          optimal_speed = value->get_max_speed();
        }
        if (optimal_angular_speed == 0.0f) {
          optimal_angular_speed = value->get_max_angular_speed();
        }
      }
      kinematics = value;
    }
  }

  /**
   * @brief      Gets the radius of the agent.
   *
   * @return     The agent's radius.
   */
  float get_radius() const { return radius; }
  /**
   * @brief      Sets the radius of the agent.
   *
   * @param      value  A positive value.
   */
  void set_radius(float value) {
    radius = std::max(0.0f, value);
    change(RADIUS);
  }
  /**
   * @brief      Gets the maximal speed.
   *
   * @return     The maximal speed returned from \ref Kinematics::get_max_speed
   */
  float get_max_speed() const {
    return kinematics ? kinematics->get_max_speed() : 0.0f;
  }
  /**
   * @brief      Sets the maximal speed.
   *
   * @param[in]  value  The value to pass to \ref Kinematics::set_max_speed
   */
  void set_max_speed(float value) {
    if (kinematics) kinematics->set_max_speed(value);
  }

  /**
   * @brief      Gets the maximal angular speed speed.
   *
   * @return     The maximal angular speed from \ref
   * Kinematics::get_max_angular_speed
   */
  Radians get_max_angular_speed() const {
    return kinematics ? kinematics->get_max_angular_speed() : 0.0f;
  }
  /**
   * @brief      Sets the maximal angular speed speed.
   *
   * @param[in]  value  The value to pass to \ref
   * Kinematics::set_max_angular_speed.
   */
  void set_max_angular_speed(Radians value) {
    if (kinematics) kinematics->set_max_angular_speed(value);
  }

  // ---------------------- BEHAVIOR PARAMETERS

  /**
   * @brief      Gets the desired optimal speed.
   *
   * Unless configured with \ref set_optimal_speed,
   * it is set to \ref get_max_speed.
   *
   * @return     The desired optimal speed.
   */
  float get_optimal_speed() const { return optimal_speed; }
  /**
   * @brief      Sets the desired optimal speed.
   *
   * @param[in]  value A positive linear speed.
   */
  void set_optimal_speed(float value) {
    if (kinematics) {
      optimal_speed = std::clamp(value, 0.0f, get_max_speed());
    } else {
      optimal_speed = std::max(value, 0.0f);
    }
    change(OPTIMAL_SPEED);
  }
  /**
   * @brief      Gets the desired optimal angular speed.
   *
   * Unless configured with \ref set_optimal_angular_speed,
   * it is set to \ref get_max_angular_speed.
   *
   * @return     The desired optimal angular speed.
   */
  Radians get_optimal_angular_speed() const { return optimal_angular_speed; }
  /**
   * @brief      Sets the optimal angular speed.
   *
   * @param[in]  value  A positive angular speed in radians/time unit.
   */
  void set_optimal_angular_speed(Radians value) {
    if (kinematics) {
      optimal_angular_speed = std::clamp(value, 0.0f, get_max_angular_speed());
    } else {
      optimal_angular_speed = std::max(value, 0.0f);
    }
  }
  /**
   * @brief      Gets the relaxation time to rotate towards a desired
   * orientation.
   *
   * The default behaviors applies a P control to rotations, e.g.,
   * \f$\omega = \frac{\delta \theta}{\tau_\textrm{rot}}\f$
   *
   * @return     The rotation relaxation time.
   */
  float get_rotation_tau() const { return rotation_tau; }
  /**
   * @brief      Sets the relaxation time to rotate towards a desired
   * orientation. See \ref get_rotation_tau.
   *
   * @param[in]  value  The rotation relaxation time. Set it to a zero or
   * negative to rotate as fast as possible.
   */
  void set_rotation_tau(float value) { rotation_tau = value; }

  /**
   * @brief      Gets the minimal safety margin to keep away from obstacles
   *
   * @return     The safety margin.
   */
  float get_safety_margin() const { return safety_margin; }
  /**
   * @brief      Sets the safety margin to keep away from obstacles.
   *
   * @param[in]  value  A positive value.
   */
  void set_safety_margin(float value) {
    safety_margin = std::max(0.0f, value);
    change(SAFETY_MARGIN);
  }
  /**
   * @brief      Gets the horizon: the size of the portion of environment
   *             around the agent considered when computing possible collisions.
   *             Larger values generally lead to a more computationally
   * expensive \ref cmd_twist but fewer deadlocks and less jamming.
   *
   * @return     The horizon.
   */
  float get_horizon() const { return horizon; }
  /**
   * @brief      Sets the horizon, see \ref get_horizon.
   *
   * @param[in]  value  A positive value.
   */
  void set_horizon(float value) {
    horizon = std::max(0.0f, value);
    change(HORIZON);
  }

  //----------- AGENT STATE

  /**
   * @brief      Gets the current pose in the world-fixed frame.
   *
   * @return     The current pose.
   */
  Pose2 get_pose() const { return pose; }
  /**
   * @brief      Sets the current pose in the world-fixed frame.
   *
   * @param[in]  value The desired value
   */
  void set_pose(const Pose2 &value) {
    pose = value;
    change(POSITION | ORIENTATION);
  }
  /**
   * @brief      Convenience method to get the current position in the
   * world-fixed frame. See \ref get_pose.
   *
   * @return     The position.
   */
  Vector2 get_position() const { return pose.position; }
  /**
   * @brief      Convenience method to set the current position in the
   * world-fixed frame. See \ref set_pose.
   *
   * @param[in]  value The desired value
   */
  void set_position(const Vector2 &value) {
    pose.position = value;
    change(POSITION);
  }
  /**
   * @brief      Convenience method to get the current orientation. See \ref
   * get_pose.
   *
   * @return     The current orientation.
   */
  Radians get_orientation() const { return pose.orientation; }
  /**
   * @brief      Convenience method to set the current orientation. See \ref
   * set_pose.
   *
   * @param[in]  value  The desired value
   */
  void set_orientation(Radians value) {
    pose.orientation = value;
    change(ORIENTATION);
  }
  /**
   * @brief      Gets the current twist.
   *
   * @param[in]  relative  The frame of reference: true for the
   * agent's own frame and false for the world-fixed frame.
   *
   * @return     The current twist.
   */
  Twist2 get_twist(bool relative = false) const {
    return to_frame(twist, relative);
  }
  /**
   * @brief      Sets the current twist.
   *
   * @param[in]  value The desired value
   */
  void set_twist(const Twist2 &value) {
    twist = value;
    change(VELOCITY | ANGULAR_SPEED);
  }
  /**
   * @brief      Convenience method to get the current velocity. See \ref
   * get_twist
   *
   * @param[in]  relative  The frame of reference: true for the
   * agent's own frame and false for the world-fixed frame.
   *
   * @return     The velocity.
   */
  Vector2 get_velocity(bool relative = false) const {
    return to_frame(twist, relative).velocity;
  }
  /**
   * @brief      Convenience method to set the current velocity. See \ref
   * set_twist
   *
   * @param[in]  value     The velocity
   * @param[in]  relative  The frame of reference: true for the
   * agent's own frame and false for the world-fixed frame.
   */
  void set_velocity(const Vector2 &value, bool relative = false) {
    twist.velocity = value;
    twist.relative = relative;
    change(VELOCITY);
  }
  /**
   * @brief      Convenience method to get the current speed. See \ref get_twist
   *
   * @return     The current speed.
   */
  float get_speed() const { return twist.velocity.norm(); }
  /**
   * @brief      Convenience method to get the current the angular speed.
   *
   * @return     The current angular speed.
   */
  Radians get_angular_speed() const { return twist.angular_speed; }
  void set_angular_speed(Radians value) {
    twist.angular_speed = value;
    change(ANGULAR_SPEED);
  }
  /**
   * @brief      Convenience method to get the current wheel speeds. See \ref
   * get_twist
   *
   * @return     The wheel speeds.
   */
  WheelSpeeds get_wheel_speeds() const {
    return wheel_speeds_from_twist(twist);
  }
  /**
   * @brief      Convenience method to set the current wheel speeds. See \ref
   * set_twist
   *
   * @param[in]  value  The wheel speeds
   */
  void set_wheel_speeds(const WheelSpeeds &value) {
    if (kinematics && kinematics->is_wheeled()) {
      Wheeled *wk = dynamic_cast<Wheeled *>(kinematics.get());
      set_twist(wk->twist(value));
    }
  }
  /**
   * @brief      Gets the last actuated twist.
   *
   * @param[in]  relative  The frame of reference: true for the
   * agent's own frame and false for the world-fixed frame.
   *
   * @return     The actuated twist.
   */
  Twist2 get_actuated_twist(bool relative = false) const {
    return to_frame(actuated_twist, relative);
  }
  /**
   * @brief      Sets the last actuated twist.
   *
   * @param[in]  value The desired value
   */
  void set_actuated_twist(const Twist2 &value) { actuated_twist = value; }
  /**
   * @brief      Convenience method to get the last actuated wheel speeds from
   * \ref actuated_twist.
   *
   * If the agent is not wheeled (\ref Kinematics::is_wheeled), an empty vector
   * is returned.
   *
   * @return     The actuated wheel speeds.
   */
  WheelSpeeds get_actuated_wheel_speeds() const {
    return wheel_speeds_from_twist(actuated_twist);
  }
  /**
   * @brief      Actuate a twist command, integrating using \ref
   * Pose2::integrate
   *
   * @param[in]  twist_cmd  The twist
   * @param[in]  time_step  The time step
   */
  void actuate(const Twist2 &twist_cmd, float time_step) {
    actuated_twist = twist_cmd;
    if (twist_cmd.relative) {
      twist = to_absolute(twist_cmd);
    } else {
      twist = actuated_twist;
    }
    pose = pose.integrate(twist, time_step);
    change(POSITION | ORIENTATION | VELOCITY | ANGULAR_SPEED);
  }
  /**
   * @brief      Convenience method to actuate the stored actuated twist
   * command,
   *
   * @param[in]  time_step  The time step
   */
  void actuate(float time_step) { actuate(actuated_twist, time_step); }

  /**
   * @brief      Clone the state of this behavior from another behavior
   *
   * @param[in]  other  The other behavior
   */
  void set_state_from(const Behavior &other) {
    set_kinematics(other.get_kinematics());
    set_radius(other.get_radius());
    set_optimal_speed(other.get_optimal_speed());
    set_optimal_angular_speed(other.get_optimal_angular_speed());
    set_rotation_tau(other.get_rotation_tau());
    set_safety_margin(other.get_safety_margin());
    set_horizon(other.get_horizon());
    set_heading_behavior(other.get_heading_behavior());
    set_target_pose(other.get_target_pose());
    set_pose(other.get_pose());
    set_twist(other.get_twist());
    set_actuated_twist(other.get_actuated_twist());
  }

  //----------- CONTROL

  /**
   * @brief      Gets the heading behavior:
   *
   * @return     The heading behavior.
   */
  Heading get_heading_behavior() const {
    if (kinematics && kinematics->dof() == 3) {
      return heading_behavior;
    } else {
      return Heading::velocity;
    }
  }
  /**
   * @brief      Sets the heading behavior, see \ref get_heading_behavior.
   *             When the kinematics has only 2 degrees of freedom (see \ref
   * Kinematics::dof()), the only available behavior is \ref Heading::velocity.
   *
   * @param[in]  value The desired value
   */
  void set_heading_behavior(Heading value) { heading_behavior = value; }
  /**
   * @brief      Gets the target pose in the world-fixed frame.
   *
   * @return     The target pose.
   */
  Pose2 get_target_pose() const { return target_pose; }
  /**
   * @brief      Sets the target pose in the world-fixed frame.
   *
   * @param[in]  value The desired value
   */
  void set_target_pose(const Pose2 &value) {
    target_pose = value;
    change(TARGET_POSITION | TARGET_ORIENTATION);
  }
  /**
   * @brief      Gets the target position in the world-fixed frame.
   *
   * @return     The target position.
   */
  Vector2 get_target_position() const { return target_pose.position; }
  /**
   * @brief      Sets the target position in the world-fixed frame.
   *
   * @param[in]  value The desired value
   */
  void set_target_position(const Vector2 &value) {
    target_pose.position = value;
    change(TARGET_POSITION);
  }
  /**
   * @brief      Gets the target orientation.
   *
   * @return     The target orientation.
   */
  Radians get_target_orientation() const { return target_pose.orientation; }
  /**
   * @brief      Sets the target orientation.
   *
   * @param[in]  value The desired value
   */
  void set_target_orientation(Radians value) {
    target_pose.orientation = value;
    change(TARGET_ORIENTATION);
  }
  /**
   * @brief      Gets the target velocity in the world-fixed frame.
   *
   * @return     The target velocity.
   */
  Vector2 get_target_velocity() const { return target_twist.velocity; }
  /**
   * @brief      Sets the target position in the world-fixed frame.
   *
   * @param[in]  value The desired value
   */
  void set_target_velocity(const Vector2 &value) {
    target_twist.velocity = value;
    change(TARGET_VELOCITY);
  }
  /**
   * @brief      Gets the target angular speed.
   *
   * @return     The target angular speed.
   */
  Radians get_target_angular_speed() const {
    return target_twist.angular_speed;
  }
  /**
   * @brief      Sets the target angular speed.
   *
   * @param[in]  value The desired value
   */
  void set_target_angular_speed(Radians value) {
    target_twist.angular_speed = value;
    change(TARGET_ANGULAR_SPEED);
  }
  /**
   * @brief      Query the behavior to get a twist control command
   *
   * Before calling this method, update the state using methods such as
   * \ref set_pose, \ref set_target_position.
   *
   * Behaviors may use caching to speed up the next queries if the state does
   * not change.
   *
   * @param[in]  time_step        The control time step. Not all behavior use it
   * but some may use it, for example, to limit accelerations.
   *
   * @param[in]  mode             Should the agent move towards the \ref
   * Behavior::get_target_position (\ref Behavior::Mode::move), follow \ref
   * Behavior::get_target_velocity (\ref Behavior::Mode::follow), turn-in-place
   * towards \ref Behavior::get_target_orientation (\ref Behavior::Mode::turn),
   * or stop moving
   * (\ref Behavior::Mode::stop).
   *
   * @param[in]  relative         The desired frame of reference for the twist:
   *                              true for the agent's own frame and
   * to false for the world-fixed frame. Leave undefined to use the default
   * frame depending on the kinematics (see \ref default_cmd_frame)
   *
   * @param[in]  set_as_actuated  If set, it assumes that the control command
   * will be actuated, therefore setting \ref Behavior::set_actuated_twist to
   * the returned value. If not set, the user should set \ref
   * Behavior::set_actuated_twist before querying for a new control commands:
   * some behavior use the old actuated control command to compute smoother
   * controls.
   *
   * @return     The control command as a twist in the specified frame.
   */

  Twist2 cmd_twist(float time_step, Mode mode = Mode::move,
                   std::optional<bool> relative = std::nullopt,
                   bool set_as_actuated = true) {
    return cmd_twist(time_step, mode, relative.value_or(default_cmd_frame()),
                     set_as_actuated);
  }

  /**
   * @brief      The most natural frame for the current kinematic:
   * relative in case the agent is wheeled, else absolute
   *
   * @return     True if the frame is relative to the agent.
   */
  bool default_cmd_frame() {
    if (kinematics && kinematics->is_wheeled()) {
      return true;
    }
    return false;
  }

  /**
   * @brief      Gets the last computed desired velocity.
   *
   * @return     The desired velocity (only valid if computed) in relative
   * frame.
   */
  Vector2 get_desired_velocity() const { return desired_velocity; }

  //----------- TRANSFORMATIONS

  /**
   * @brief      Transform a twist to absolute frame
   *
   * @param[in]  value  The original twist
   *
   * @return     The same twist in absolute frame
   *             (unchanged if already in absolute frame)
   */
  Twist2 to_absolute(const Twist2 &value) const {
    if (value.relative) {
      auto a_value = value.rotate(pose.orientation);
      a_value.relative = false;
      return a_value;
    }
    return twist;
  }

  /**
   * @brief      Transform a twist to relative frame
   *
   * @param[in]  value  The original twist
   *
   * @return     The same twist in relative frame
   *             (unchanged if already in relative frame)
   */
  Twist2 to_relative(const Twist2 &value) const {
    if (!value.relative) {
      auto r_value = value.rotate(-pose.orientation);
      r_value.relative = true;
      return r_value;
    }
    return value;
  }

  /**
   * @brief      Transform a vector (e.g., a velocity)
   *             from absolute to relative frame
   *
   * @param[in]  value  The absolute vector
   *
   * @return     The relative vector
   */
  Vector2 to_relative(const Vector2 &value) const {
    return rotate(value, -pose.orientation);
  }

  /**
   * @brief      Convert a twist between reference frames.
   *
   * @param[in]  value     The original frame.
   * @param[in]  relative  The desired frame of reference:
   *                       true for the agent's own frame and false
   * for the world-fixed frame.
   *
   * @return     The frame in the desired frame of reference.
   */
  Twist2 to_frame(const Twist2 &value, bool relative) const {
    return relative ? to_relative(value) : to_absolute(value);
  }

  /**
   * @brief      Convenience method to transform from twist to wheel speeds.
   *
   * If the agent is not wheeled (\ref Kinematics::is_wheeled), an empty vector
   * is returned.
   *
   * @param[in]  value  The twist
   *
   * @return     The corresponding wheel speeds.
   */
  WheelSpeeds wheel_speeds_from_twist(const Twist2 &value) const {
    if (kinematics && kinematics->is_wheeled()) {
      Wheeled *wk = dynamic_cast<Wheeled *>(kinematics.get());
      return wk->wheel_speeds(value.relative ? value : to_relative(value));
    }
    return {};
  }
  /**
   * @brief       Convenience method to transform from wheel speeds to twist.
   *
   * If the agent is not wheeled (\ref Kinematics::is_wheeled), an zero twist is
   * returned.
   *
   * @param[in]  value  The wheel speeds
   *
   * @return     The corresponding twist.
   */
  Twist2 twist_from_wheel_speeds(const WheelSpeeds &value) const {
    if (kinematics && kinematics->is_wheeled()) {
      Wheeled *wk = dynamic_cast<Wheeled *>(kinematics.get());
      return wk->twist(value);
    }
    return {};
  }

  /**
   * The behavior social margin modulation
   */
  SocialMargin social_margin;

  /**
   * @brief      Gets the environment state.
   *
   * @return     The environment state.
   */
  virtual EnvironmentState *get_environment_state() { return nullptr; }

 protected:
  enum {
    POSITION = 1 << 0,
    ORIENTATION = 1 << 1,
    VELOCITY = 1 << 2,
    ANGULAR_SPEED = 1 << 3,
    TARGET_POSITION = 1 << 4,
    TARGET_ORIENTATION = 1 << 5,
    TARGET_VELOCITY = 1 << 6,
    TARGET_ANGULAR_SPEED = 1 << 7,
    HORIZON = 1 << 8,
    OPTIMAL_SPEED = 1 << 9,
    SAFETY_MARGIN = 1 << 10,
    RADIUS = 1 << 11
  };

  std::shared_ptr<Kinematics> kinematics;
  float radius;
  Pose2 pose;
  Twist2 twist;
  Twist2 actuated_twist;
  float horizon;
  float safety_margin;
  float optimal_speed;
  Radians optimal_angular_speed;
  float rotation_tau;
  Heading heading_behavior;
  Pose2 target_pose;
  Twist2 target_twist;
  Vector2 desired_velocity;

  virtual Vector2 compute_desired_velocity([[maybe_unused]] float time_step) {
    return Vector2::Zero();
  }
  virtual Twist2 twist_towards_velocity(const Vector2 &absolute_velocity,
                                        bool relative);
  //   * Sub-classes may not override this method but the specialized methods
  // * \ref cmd_twist_towards_target, \ref cmd_twist_towards_target_orientation,
  // * and \ref cmd_cmd_twist_towards_stopping.
  virtual Twist2 cmd_twist(float time_step, Mode mode, bool relative,
                           bool set_as_actuated);
  virtual Twist2 cmd_twist_towards_target(float dt, bool relative);
  virtual Twist2 cmd_twist_towards_target_orientation(float dt, bool relative);
  virtual Twist2 cmd_twist_towards_stopping(float dt, bool relative);
};

}  // namespace hl_navigation

#endif  // HL_NAVIGATION_BEHAVIOR_H_
