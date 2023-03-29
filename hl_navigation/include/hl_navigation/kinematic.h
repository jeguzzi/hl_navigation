#ifndef HL_NAVIGATION_KINEMATIC_H_
#define HL_NAVIGATION_KINEMATIC_H_

#include <assert.h>

#include <algorithm>
#include <vector>

#include "hl_navigation/common.h"
#include "hl_navigation/property.h"
#include "hl_navigation/register.h"

namespace hl_navigation {

/**
 * @brief      Abstract Kinematic type.
 *
 * Kinematic is used to
 * - validated twist in the agent's frame as feasible
 * - convert between wheel speeds and body twist
 * - store maximal linear and angular speed
 * - store the number of degrees of freedom
 */
class Kinematic : virtual public HasProperties,
                  virtual public HasRegister<Kinematic> {
 public:

  using HasRegister<Kinematic>::C;
  
  Kinematic(float max_speed, float max_angular_speed=0.0)
      : max_speed(max_speed), max_angular_speed(max_angular_speed) {}

  virtual ~Kinematic() = default;

  /**
   * @brief      Compute the nearest feasible twist to a desired twist.
   *
   * @param[in]  twist  The desired twist
   *
   * @return     The same desired twist if feasible else a near value that is
   * feasible. How this is defined depends on the concrete Kinematic type.
   */
  virtual Twist2 feasible(const Twist2& twist) const = 0;

  /**
   * @brief      Returns if the kinematic has wheels.
   *
   * @return     True if wheeled, False otherwise.
   */
  virtual bool is_wheeled() const = 0;
  /**
   * @brief      Returns the degrees of freedom (between 0 and 3 for planer
   * rigid body kinematics)
   *
   * @return     The number of degrees of freedom.
   */
  virtual unsigned dof() const = 0;
  /**
   * @brief      Gets the maximal speed.
   *
   * @return     The maximal speed.
   */
  float get_max_speed() const { return max_speed; }
  /**
   * @brief      Sets the maximum speed.
   *
   * @param[in]  value  A positive value.
   */
  void set_max_speed(float value) { max_speed = std::max(0.0f, value); }
  /**
   * @brief      Gets the maximal angular speed.
   *
   * @return     The maximal angular speed.
   */
  virtual float get_max_angular_speed() const { return max_angular_speed; }
  /**
   * @brief      Sets the maximum angular speed.
   *
   * @param[in]  value  A positive value.
   */
  void set_max_angular_speed(float value) {
    max_angular_speed = std::max(0.0f, value);
  }

  // virtual const std::string& get_name() const = 0;

  // using KinematicFactory = std::function<std::shared_ptr<Kinematic>(float)>;

  // static std::shared_ptr<Kinematic> kinematic_with_name(
  //     const std::string& name, float max_speed = 0.0f) {
  //   if (factory.count(name)) {
  //     return factory[name](max_speed);
  //   }
  //   return nullptr;
  // }

  // static const std::map<std::string, KinematicFactory>& all_kinematics() {
  //   return factory;
  // }

  // static const std::map<std::string, Properties>& all_properties() {
  //   return factory_properties;
  // }

  // static inline std::map<std::string, KinematicFactory> factory = {};
  // static inline std::map<std::string, Properties> factory_properties = {};

  // template <typename T>
  // static std::string register_type(const std::string& name) {
  //   factory[name] = [](float max_speed = 0.0f) {
  //     return std::make_shared<T>(max_speed);
  //   };
  //   factory_properties[name] = T::properties;
  //   return name;
  // }

  // static inline std::map<std::string, Property> properties = Properties{};

  // std::string description(bool extensive = false) const {
  //   std::ostringstream os;
  //   os << get_type();
  //   if (extensive) {
  //     os << ":" << std::endl;
  //     os << "  max_speed = " << get_max_speed() << std::endl;
  //     os << "  max_angular_speed = " << get_max_angular_speed() << std::endl;
  //     for (const auto& [k, v] : get_properties()) {
  //       os << k << " = " << get(k) << "[" << v.type_name << "]"
  //          << std::endl;
  //     }
  //   }
  //   return os.str();
  // }

 private:
  /**
   * The maximal speed
   */
  float max_speed;
  /**
   * The maximal angular speed
   */
  float max_angular_speed;
};

/**
 * @brief      Unconstrained kinematic (e.g., quadcopters)
 */
class Holonomic : public Kinematic {
 public:
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  max_speed          The maximal speed
   * @param[in]  max_angular_speed  The maximal angular speed
   */
  Holonomic(float max_speed = 0.0f, float max_angular_speed = 0.0f)
      : Kinematic(max_speed, max_angular_speed) {}

  Twist2 feasible(const Twist2& twist) const override {
    return {clamp_norm(twist.velocity, get_max_speed()),
            std::clamp(twist.angular_speed, -get_max_angular_speed(),
                       get_max_angular_speed()),
            twist.relative};
  }
  /**
   * @brief      Returns if the kinematic has wheels.
   *
   * @return     False
   */
  bool is_wheeled() const override { return false; }
  /**
   * @brief      Returns the degrees of freedom
   *
   * @return     3
   */
  unsigned dof() const override { return 3; }

  std::string get_type() const override { return type; }

 private:
  inline static std::string type = register_type<Holonomic>("Holonomic");
};

/**
 * @brief      Kinematic for non wheeled agents that head towards they move
 * (e.g., people)
 */
class Forward : public Kinematic {
 public:
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  max_speed          The maximal speed
   * @param[in]  max_angular_speed  The maximal angular speed
   */
  Forward(float max_speed = 0.0f, float max_angular_speed = 0.0f)
      : Kinematic(max_speed, max_angular_speed) {}

  Twist2 feasible(const Twist2& twist) const override {
    assert(twist.relative);
    return {{std::clamp(twist.velocity[0], 0.0f, get_max_speed()), 0},
            std::clamp(twist.angular_speed, -get_max_angular_speed(),
                       get_max_angular_speed()),
            true};
  }
  /**
   * @brief      Returns if the kinematic has wheels.
   *
   * @return     False
   */
  bool is_wheeled() const override { return false; }
  /**
   * @brief      Returns the degrees of freedom
   *
   * @return     2
   */
  unsigned dof() const override { return 2; }

  std::string get_type() const override { return type; }

 private:
  inline static std::string type = register_type<Forward>("Forward");
};

/**
 * @brief      Abstract wheeled kinematic
 */
class Wheeled : public Kinematic {
 public:
  Wheeled(float max_speed, float max_angular_speed, float axis)
      : Kinematic(max_speed, max_angular_speed), axis(axis) {}

  virtual ~Wheeled() = default;

  /**
   * @brief      Returns if the kinematic has wheels.
   *
   * @return     True
   */
  bool is_wheeled() const override { return true; }

  // /**
  //  * @brief      Ignored as the angular speed depends on the maximal wheel
  //  speed
  //  * @param[in]  value  A positive value.
  //  */
  // virtual void set_max_angular_speed([[maybe_unused]] float value) override
  // {}

  /**
   * @brief      Convert wheel speeds to a twist
   *
   * @param[in]  value  The wheel speeds
   *
   * @return     The corresponding twist
   */
  virtual Twist2 twist(const WheelSpeeds& value) const = 0;
  /**
   * @brief      Convert a twist to wheel speeds
   *
   * @param[in]  value  The twist
   *
   * @return     The corresponding wheel speeds.
   */
  virtual WheelSpeeds wheel_speeds(const Twist2& value) const = 0;

  /**
   * @brief      Implementation of \ref Kinematic::feasible where the twist is
   *             first transformed to wheel speeds and the back to a twist.
   *
   * @param[in]  value  The desired value
   *
   * @return     The same desired twist if feasible else a near value that is
   * feasible. How this is defined depends on the concrete Kinematic type.
   */
  Twist2 feasible(const Twist2& value) const override {
    return twist(wheel_speeds(value));
  }

  float get_axis() const { return axis; }
  void set_axis(float value) {
    if (value > 0) axis = value;
  }

  virtual const Properties& get_properties() const override {
    return properties;
  };

  static inline std::map<std::string, Property> properties = Properties{
      {"wheel_axis", make_property<float, Wheeled>(
                         &Wheeled::get_axis, &Wheeled::set_axis, 0.0f, "Wheel Axis")},
  };

 protected:
  float axis;
};

/**
 * @brief      Differential two-wheeled robot (e.g., a wheelchair)
 */
class TwoWheeled : public Wheeled {
 public:
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  max_speed          The maximal wheel speed
   * @param[in]  axis               The wheel axis (i.e., the distance between
   * the wheels)
   */
  TwoWheeled(float max_speed = 0.0f, float axis = 0.0f)
      : Wheeled(max_speed, (axis > 0) ? 2 * max_speed / axis : 0.0f, axis) {}

  /**
   * @brief      Returns the degrees of freedom
   *
   * @return     2
   */
  unsigned dof() const override { return 2; }

  // void set_max_speed(float value) override {
  //   Kinematic::set_max_speed(value);
  //   Kinematic::set_max_angular_speed(2 * get_max_speed() / get_axis());
  // }

  /**
   * @brief      TODO(Jerome) Gets the maximum angular speed.
   *
   * @param[in]  value  The value
   *
   * @return     The maximum angular speed.
   */
  float get_max_angular_speed() const override {
    if (get_axis() > 0) {
      return 2 * get_max_speed() / get_axis();
    }
    return 0.0f;
  }

  /**
   * @brief      See \ref Wheeled::twist.
   *
   * @param[in]  speeds  The wheel speeds in the order {left, right}
   *
   * @return     The corresponding twist
   */
  Twist2 twist(const WheelSpeeds& speeds) const override {
    if (speeds.size() == 2) {
      // {left, right}
      return {{0.5f * (speeds[0] + speeds[1]), 0.0f},
              (speeds[1] - speeds[0]) / axis,
              true};
    }
    return {};
  }

  /**
   * @brief      See \ref Wheeled::wheel_speeds.
   *
   * @param[in]  twist  The twist
   *
   * @return     The corresponding wheel speeds in the order {left, right}
   */
  WheelSpeeds wheel_speeds(const Twist2& twist) const override {
    assert(twist.relative);
    // {left, right}
    float max_speed = get_max_speed();
    const float rotation =
        std::clamp(0.5f * twist.angular_speed * axis, -max_speed, max_speed);
    const float linear = std::clamp(twist.velocity[0], 0.0f, max_speed);
    float left = linear - rotation;
    float right = linear + rotation;
    if (abs(left) > max_speed) {
      left = std::clamp(left, -max_speed, max_speed);
      right = left + 2 * rotation;
    } else if (abs(right) > max_speed) {
      right = std::clamp(right, -max_speed, max_speed);
      left = right - 2 * rotation;
    }
    return {left, right};
  }

  std::string get_type() const override { return type; }

 private:
  inline static std::string type = register_type<TwoWheeled>("TwoWheeled");
};

// TODO(Jerome): make it general

/**
 * @brief      Differential four-wheeled robot (e.g., a Robomaster)
 *
 * \warning We assume that the distance between front and back wheel centers is
 * the same as the lateral distance.
 */
class FourWheeled : public Wheeled {
 public:
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  max_speed          The maximal wheel speed
   * @param[in]  axis               The wheel axis (i.e., the distance between
   * the wheels)
   */
  FourWheeled(float max_speed = 0.0f, float axis = 0.0f)
      : Wheeled(max_speed, axis > 0 ? max_speed / axis : 0.0f, axis) {}

  /**
   * @brief      Returns the degrees of freedom
   *
   * @return     3
   */
  unsigned dof() const override { return 3; }

  /**
   * @brief      TODO(Jerome) Gets the maximum angular speed.
   *
   * @param[in]  value  The value
   *
   * @return     The maximum angular speed.
   */
  float get_max_angular_speed() const override {
    if (get_axis() > 0) {
      return get_max_speed() / get_axis();
    }
    return 0.0f;
  }

  /**
   * @brief      See \ref Wheeled::twist.
   *
   * @param[in]  speeds  The wheel speeds in the order {front left, rear left,
   * rear right, rear left}
   *
   * @return     The corresponding twist
   */
  Twist2 twist(const WheelSpeeds& speeds) const override {
    if (speeds.size() == 4) {
      // {front left, rear left, rear right, rear left}
      return {{0.25f * (speeds[0] + speeds[1] + speeds[2] + speeds[3]),
               0.25f * (-speeds[0] + speeds[1] - speeds[2] + speeds[3])},
              0.25f * (-speeds[0] - speeds[1] + speeds[2] + speeds[3]) / axis,
              true};
    }
    return {};
  }

  /**
   * @brief      See \ref Wheeled::wheel_speeds.
   *
   * @param[in]  twist  The twist
   *
   * @return     The corresponding wheel speeds in the order {front left, rear
   * left, rear right, rear left}
   */
  WheelSpeeds wheel_speeds(const Twist2& twist) const override {
    assert(twist.relative);
    // {front left, rear left, rear right, rear left}
    float max_speed = get_max_speed();
    const float rotation =
        std::clamp(twist.angular_speed * axis, -max_speed, max_speed);
    const float longitudinal =
        std::clamp(twist.velocity[0], -max_speed, max_speed);
    const float lateral = std::clamp(twist.velocity[1], -max_speed, max_speed);
    float front_left = longitudinal - lateral - rotation;
    float front_right = longitudinal + lateral + rotation;
    float rear_left = longitudinal + lateral - rotation;
    float rear_right = longitudinal - lateral + rotation;
    if (abs(front_left) > max_speed) {
      front_left = std::clamp(front_left, -max_speed, max_speed);
      front_right = front_left + 2 * lateral + 2 * rotation;
      rear_left = front_left + 2 * lateral;
      rear_right = front_left + 2 * rotation;
    } else if (abs(front_right) > max_speed) {
      front_right = std::clamp(front_right, -max_speed, max_speed);
      front_left = front_right - 2 * lateral - 2 * rotation;
      rear_left = front_right - 2 * rotation;
      rear_right = front_right - 2 * lateral;
    } else if (abs(rear_left) > max_speed) {
      rear_left = std::clamp(rear_left, -max_speed, max_speed);
      front_left = rear_left - 2 * lateral;
      front_right = rear_left + 2 * rotation;
      rear_right = rear_left - 2 * rotation + 2 * rotation;
    } else if (abs(rear_right) > max_speed) {
      rear_right = std::clamp(rear_right, -max_speed, max_speed);
      front_left = rear_right - 2 * rotation;
      front_right = rear_right + 2 * lateral;
      rear_left = rear_right + 2 * lateral - 2 * rotation;
    }
    return {front_left, rear_left, rear_right, front_right};
  }

  std::string get_type() const override { return type; }

 private:
  inline static std::string name = register_type<FourWheeled>("FourWheeled");
};

}  // namespace hl_navigation

#endif /* end of include guard: HL_NAVIGATION_KINEMATIC_H_ */
