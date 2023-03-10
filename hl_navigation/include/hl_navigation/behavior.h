/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_BEHAVIOR_H_
#define HL_NAVIGATION_BEHAVIOR_H_

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hl_navigation/common.h"
#include "hl_navigation/kinematic.h"
#include "hl_navigation/social_margin.h"

namespace hl_navigation {

/**
 * @brief      This class describes a generic behavior to reach a target
 * position avoiding collision. Users
 * should not instantiate this class (as it's behavior just keeps the agent in
 * place) but one of it's concrete sub-classes.
 *
 * The typical life-cycle of a behavior is:
 *
 * A. Initialization:
 *  1. select the concrete behavior, the agent's size (agents are shaped like
 * disc with radius \ref radius), and the agents \ref Kinematic (see \ref
 * behavior_with_name)
 *  2. configure the generic parameters: desired optimal speed, horizon, safety
 * margin, and rotation time
 *  3. configure the specific parameters of the concrete behavior
 *
 * B. At regular time intervals:
 *  1. update the agent's state with \ref set_pose and \ref set_twist (or other
 * convenience methods)
 *  2. update the local environmental state exposed by the concrete subclass
 *  3. update the target with \ref set_target_position and/or \ref
 * set_target_orientation
 *  4. ask for a control commands with \ref cmd_twist
 *  5, actuate the control commands [not part of this API]
 */
class Behavior : protected RegisterChanges {
  using BehaviorFactory = std::function<std::shared_ptr<Behavior>(
      std::shared_ptr<Kinematic>, float)>;

 public:
  /**
   * @brief      Modes for \ref cmd_twist
   */
  enum class Mode {
    move,   /**< move towards the the target position */
    follow, /**< follow target velocity */
    turn,   /**< turn-in-place towards the the target orientation */
    stop    /**< stop */
  };

  /**
   * @brief      Different behavior for the angular motion
   */
  enum class Heading {
    idle,                 /**< do not turn */
    target_point,         /**< turn towards the target position */
    target_angle,         /**< turn towards the target orientation */
    target_angular_speed, /**< follow angular speed */
    /**
     * heads towards the velocity (only behavior available to constrained
     * kinematics)
     */
    velocity
  };

  SocialMargin social_margin;

  /**
   * Default rotation tau
   */
  static constexpr float default_rotation_tau = 0.5f;
  /**
   * Default horizon
   */
  static constexpr float default_horizon = 5.0f;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  kinematic  The kinematic of the agent.
   * @param[in]  radius     The radius of the agent.
   */
  Behavior(std::shared_ptr<Kinematic> kinematic, float radius)
      : RegisterChanges(),
        social_margin(),
        kinematic(kinematic),
        radius(radius),
        pose(),
        twist(),
        horizon(default_horizon),
        safety_margin(0.0f),
        optimal_speed(kinematic->get_max_speed()),
        optimal_angular_speed(kinematic->get_max_angular_speed()),
        rotation_tau(default_rotation_tau),
        heading_behavior(Heading::idle),
        target_pose(),
        target_twist() {}

  virtual ~Behavior() = default;

  /**
   * @brief     Instantiate a behavior of type with a given name.
   *
   * @param[in]  name       The name of the behavior.
   * @param[in]  kinematic  The kinematic of the agent.
   * @param[in]  radius     The radius of the agent.
   *
   * @return     A behavior.
   */
  static std::shared_ptr<Behavior> behavior_with_name(
      const std::string &name, std::shared_ptr<Kinematic> kinematic,
      float radius) {
    if (factory.count(name)) {
      return factory[name](kinematic, radius);
    }
    return nullptr;
  }

  /**
   * @brief      A map to instantiate behaviors by name.
   *
   * @return     A map {name: (Kinematic, radius) -> Behavior}
   */
  static const std::map<std::string, BehaviorFactory> &all_behaviors() {
    return factory;
  }

  /**
   * @brief      Returns the names of all registered behaviors
   *
   * @return     The available behaviors.
   */
  static const std::vector<std::string> behavior_names() {
    std::vector<std::string> keys;
    std::transform(
        factory.begin(), factory.end(), back_inserter(keys),
        [](std::pair<std::string, BehaviorFactory> p) { return p.first; });
    return keys;
  }

  //------------ AGENT PARAMETERS

  /**
   * @brief      Gets the kinematics.
   *
   * @return     The agent's kinematics.
   */
  std::shared_ptr<Kinematic> get_kinematic() const { return kinematic; }
  /**
   * @brief      Sets the kinematics.
   *
   * @param[in]  value
   */
  void set_kinematic(std::shared_ptr<Kinematic> value) {
    if (value) {
      kinematic = value;
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
   * @return     The maximal speed from \ref Kinematic
   */
  float get_max_speed() const { return kinematic->get_max_speed(); }
  /**
   * @brief      Sets the maximal speed.
   *
   * @param[in]  value  The value to pass to \ref Kinematic.
   */
  void set_max_speed(float value) { kinematic->set_max_speed(value); }

  /**
   * @brief      Gets the maximal angular speed speed.
   *
   * @return     The maximal angular speed from \ref Kinematic
   */
  Radians get_max_angular_speed() const {
    return kinematic->get_max_angular_speed();
  }
  /**
   * @brief      Sets the maximal angular speed speed.
   *
   * @param[in]  value  The value to pass to \ref Kinematic.
   */
  void set_max_angular_speed(Radians value) {
    kinematic->set_max_angular_speed(value);
  }

  // ---------------------- BEHAVIOR PARAMETERS

  /**
   * @brief      Gets the desired optimal speed.
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
    optimal_speed = std::clamp(value, 0.0f, get_max_speed());
    change(OPTIMAL_SPEED);
  }
  /**
   * @brief      Gets the desired optimal angular speed.
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
    optimal_angular_speed = std::clamp(value, 0.0f, get_max_angular_speed());
  }
  /**
   * @brief      Gets the relaxation time to rotate towards a desired
   * orientation.
   *
   * Behaviors generally applies a P control to rotations, e.g.,
   * ``\f$\omega = \delta \theta / \tau_\textrm{rot}\f$``
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
   * @brief      Gets the horizon, as the size of the portion of environment
   *             around the agent considered to compute collisions.
   *             Larger values generally lead to a more expensive query \ref
   * cmd_twist, and fewer deadlocks or less jamming.
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
   * @brief      Gets the current pose in the world fixed frame.
   *
   * @return     The current pose.
   */
  Pose2 get_pose() const { return pose; }
  /**
   * @brief      Sets the current pose in the world fixed frame.
   *
   * @param[in]  value
   */
  void set_pose(const Pose2 &value) {
    pose = value;
    change(POSITION | ORIENTATION);
  }
  /**
   * @brief      Convenience method to get the current position in the world
   * fixed frame. See \ref get_pose.
   *
   * @return     The position.
   */
  Vector2 get_position() const { return pose.position; }
  /**
   * @brief      Convenience method to set the current position in the world
   * fixed frame. See \ref set_pose.
   *
   * @param[in]  value
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
   * @param[in]  value  The value
   */
  void set_orientation(Radians value) {
    pose.orientation = value;
    change(ORIENTATION);
  }
  /**
   * @brief      Gets the current twist.
   *
   * @param[in]  relative  The frame of reference: set it to true for the
   * agent's own frame and to false for the world fixed frame.
   *
   * @return     The current twist.
   */
  Twist2 get_twist(bool relative = false) const {
    return to_frame(twist, relative);
  }
  /**
   * @brief      Sets the current twist.
   *
   * @param[in]  value
   */
  void set_twist(const Twist2 &value) {
    twist = value;
    change(VELOCITY | ANGULAR_SPEED);
  }
  /**
   * @brief      Convenience method to get the current velocity. See \ref
   * get_twist
   *
   * @param[in]  relative  The frame of reference: set it to true for the
   * agent's own frame and to false for the world fixed frame.
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
   * @param[in]  relative  The frame of reference: set it to true for the
   * agent's own frame and to false for the world fixed frame.
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
    if (kinematic->is_wheeled()) {
      Wheeled *wk = dynamic_cast<Wheeled *>(kinematic.get());
      set_twist(wk->twist(value));
    }
  }
  /**
   * @brief      Gets the last actuated twist.
   *
   * @param[in]  relative  The frame of reference: set it to true for the
   * agent's own frame and to false for the world fixed frame.
   *
   * @return     The actuated twist.
   */
  Twist2 get_actuated_twist(bool relative = false) const {
    return to_frame(actuated_twist, relative);
  }
  /**
   * @brief      Sets the last actuated twist.
   *
   * @param[in]  value
   */
  void set_actuated_twist(const Twist2 &value) { actuated_twist = value; }
  /**
   * @brief      Convenience method to get the last actuated wheel speeds from
   * \ref actuated_twist.
   *
   * If the agent is not wheeled (Kinematic::is_wheeled()), an empty vector is
   * returned.
   *
   * @return     The actuated wheel speeds.
   */
  WheelSpeeds get_actuated_wheel_speeds() const {
    return wheel_speeds_from_twist(actuated_twist);
  }
  /**
   * @brief      Actuate a twist command, i.e., integrate pose = pose +
   * time_step * twist_cmd.
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
    change(POSITION|ORIENTATION|VELOCITY|ANGULAR_SPEED);
  }
  /**
   * @brief      Convenience method to actuate the last recorded actuated twist
   * command,
   *
   * @param[in]  time_step  The time step
   */
  void actuate(float time_step) { actuate(actuated_twist, time_step); }

  //----------- CONTROL

  /**
   * @brief      Gets the heading behavior:
   *
   * @return     The heading behavior.
   */
  Heading get_heading_behavior() const {
    if (kinematic->dof() == 3) {
      return heading_behavior;
    } else {
      return Heading::velocity;
    }
  }
  /**
   * @brief      Sets the heading behavior, see \ref get_heading_behavior.
   *             When the kinematic has only 2 degrees of freedom (see \ref
   * Kinematic::dof()), the only available behavior is Heading::velocity.
   *
   * @param[in]  value
   */
  void set_heading_behavior(Heading value) { heading_behavior = value; }
  /**
   * @brief      Gets the target pose in the world fixed frame.
   *
   * @return     The target pose.
   */
  Pose2 get_target_pose() const { return target_pose; }
  /**
   * @brief      Sets the target pose in the world fixed frame.
   *
   * @param[in]  value
   */
  void set_target_pose(const Pose2 &value) {
    target_pose = value;
    change(TARGET_POSITION | TARGET_ORIENTATION);
  }
  /**
   * @brief      Gets the target position in the world fixed frame.
   *
   * @return     The target position.
   */
  Vector2 get_target_position() const { return target_pose.position; }
  /**
   * @brief      Sets the target position in the world fixed frame.
   *
   * @param[in]  value
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
   * @param[in]  value
   */
  void set_target_orientation(Radians value) {
    target_pose.orientation = value;
    change(TARGET_ORIENTATION);
  }
  /**
   * @brief      Gets the target velocity in the world fixed frame.
   *
   * @return     The target velocity.
   */
  Vector2 get_target_velocity() const { return target_twist.velocity; }
  /**
   * @brief      Sets the target position in the world fixed frame.
   *
   * @param[in]  value
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
   * @param[in]  value
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
   * Sub-classes may not override this method but the specialized methods
   * \ref cmd_twist_towards_target, \ref cmd_twist_towards_target_orientation,
   * and \ref cmd_twist_towards_target_orientation.
   *
   * Behaviors use caching to speed up the next queries if the state does not
   * change.
   *
   * @param[in]  time_step        The control time step. Not all behavior use it
   * but some may use it, for example, to limit accelerations.
   * @param[in]  relative         The desired frame of reference for the twist:
   *                              set it to true for the agent's own frame and
   * to false for the world fixed frame.
   *
   * @param[in]  mode             Should the agent move towards the \ref
   * Behavior::get_target_position (\ref Behavior::Mode::move), follow \ref
   * Behavior::get_target_velocity, turn-in-place towards
   * \ref Behavior::get_target_orientation, or stop moving.
   *
   * @param[in]  set_as_actuated  If set, it assumes that the control command
   * will be actuated, therefore setting Behavior::actuated_twist to the
   * returned value. If not set, the user should call
   * Behavior::set_actuated_twist before querying for a new control commands:
   *                              some behavior use the old actuated control
   * command to compute smoother controls.
   *
   * @return     The control command as a twist in the specified frame.
   */
  virtual Twist2 cmd_twist(float time_step, bool relative,
                           Mode mode = Mode::move, bool set_as_actuated = true);

  Twist2 cmd_twist(float time_step, Mode mode = Mode::move,
                   bool set_as_actuated = true) {
    return cmd_twist(time_step, default_cmd_frame(), mode, set_as_actuated);
  }

  //----------- TRANSFORMATIONS

  Twist2 to_absolute(const Twist2 &value) const {
    if (value.relative) {
      auto a_value = value.rotate(pose.orientation);
      a_value.relative = false;
      return a_value;
    }
    return twist;
  }

  Twist2 to_relative(const Twist2 &value) const {
    if (!value.relative) {
      auto r_value = value.rotate(-pose.orientation);
      r_value.relative = true;
      return r_value;
    }
    return value;
  }

  Vector2 to_relative(const Vector2 &absolute_velocity) const {
    return rotate(absolute_velocity, -pose.orientation);
  }

  /**
   * @brief      Convert a twist between reference frames.
   *
   * @param[in]  value     The original frame.
   * @param[in]  relative  The desired frame of reference:
   *                       set it to true for the agent's own frame and to false
   * for the world fixed frame.
   *
   * @return     The frame in the desired frame of reference.
   */
  Twist2 to_frame(const Twist2 &value, bool relative) const {
    return relative ? to_relative(value) : to_absolute(value);
  }

  /**
   * @brief      Convenience method to transform from twist to wheel speeds.
   *
   * If the agent is not wheeled (Kinematic::is_wheeled()), an empty vector is
   * returned.
   *
   * @param[in]  value  The twist
   *
   * @return     The corresponding wheel speeds.
   */
  WheelSpeeds wheel_speeds_from_twist(const Twist2 &value) const {
    if (kinematic->is_wheeled()) {
      Wheeled *wk = dynamic_cast<Wheeled *>(kinematic.get());
      return wk->wheel_speeds(value.relative ? value : to_relative(value));
    }
    return {};
  }
  /**
   * @brief       Convenience method to transform from wheel speeds to twist.
   *
   * If the agent is not wheeled (Kinematic::is_wheeled()), an zero twist is
   * returned.
   *
   * @param[in]  value  The wheel speeds
   *
   * @return     The corresponding twist.
   */
  Twist2 twist_from_wheel_speeds(const WheelSpeeds &value) const {
    if (kinematic->is_wheeled()) {
      Wheeled *wk = dynamic_cast<Wheeled *>(kinematic.get());
      return wk->twist(value);
    }
    return {};
  }

  /**
   * @brief      Gets the desired velocity.
   *
   * @return     The desired velocity (only valid if computed) in relative
   * frame.
   */
  Vector2 get_desired_velocity() const { return desired_velocity; }

  /**
   * @brief      Gets the social margin.
   *
   * @return     The social margin.
   */
  // const SocialMargin &get_social_margin() const { return social_margin; }

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

  std::shared_ptr<Kinematic> kinematic;
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

  static std::map<std::string, BehaviorFactory> factory;

  virtual Vector2 compute_desired_velocity([[maybe_unused]] float time_step) { return Vector2::Zero(); }
  virtual Twist2 twist_towards_velocity(const Vector2 &absolute_velocity,
                                        bool relative);
  virtual Twist2 cmd_twist_towards_target(float dt, bool relative);
  virtual Twist2 cmd_twist_towards_target_orientation(float dt, bool relative);
  virtual Twist2 cmd_twist_towards_stopping(float dt, bool relative);

  template <typename T>
  static const char *register_type(const char *name) {
    factory[name] = [](std::shared_ptr<Kinematic> kinematic, float radius) {
      return std::make_shared<T>(kinematic, radius);
    };
    return name;
  }

  // ask for a relative twist in case the agent is wheeled.
  bool default_cmd_frame() {
    if (kinematic->is_wheeled()) {
      return true;
    }
    return false;
  }
};

}  // namespace hl_navigation

#endif  // HL_NAVIGATION_BEHAVIOR_H_
