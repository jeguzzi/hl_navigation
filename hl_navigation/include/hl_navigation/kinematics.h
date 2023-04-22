#ifndef HL_NAVIGATION_KINEMATICS_H_
#define HL_NAVIGATION_KINEMATICS_H_

#include <assert.h>

#include <algorithm>
#include <vector>

#include "hl_navigation/common.h"
#include "hl_navigation/property.h"
#include "hl_navigation/register.h"
#include "hl_navigation_export.h"

namespace hl_navigation {

/**
 * @brief      Abstract Kinematics type.
 *
 * A kinematics is used to
 * 
 * - validated twist in the agent's frame as feasible
 * 
 * - convert between wheel speeds and body twist
 * 
 * - store maximal linear and angular speed
 * 
 * - store the number of degrees of freedom
 */
class HL_NAVIGATION_EXPORT Kinematics : virtual public HasProperties,
                                        virtual public HasRegister<Kinematics> {
 public:
  using HasRegister<Kinematics>::C;

  Kinematics(float max_speed, float max_angular_speed = 0.0)
      : max_speed(max_speed), max_angular_speed(max_angular_speed) {}

  virtual ~Kinematics() = default;

  /**
   * @brief      Computes the nearest feasible twist to a desired twist.
   *
   * @param[in]  twist  The desired twist
   *
   * @return     The same desired twist if feasible else the nearest feasible value. 
   * How this is defined depends on the concrete sub-class.
   */
  virtual Twist2 feasible(const Twist2& twist) const = 0;

  /**
   * @brief      Returns whether the kinematics has wheels.
   *
   * @return     True if wheeled, False otherwise.
   */
  virtual bool is_wheeled() const = 0;
  /**
   * @brief      Returns the degrees of freedom (between 0 and 3 for planar
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
 * @brief      Unconstrained kinematics (e.g., quad-copters)
 */
class HL_NAVIGATION_EXPORT Holonomic : public Kinematics {
 public:
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  max_speed          The maximal speed
   * @param[in]  max_angular_speed  The maximal angular speed
   */
  Holonomic(float max_speed = 0.0f, float max_angular_speed = 0.0f)
      : Kinematics(max_speed, max_angular_speed) {}

  /**
   * @private
   */
  Twist2 feasible(const Twist2& twist) const override;
  /**
   * @brief      Returns whether the kinematics has wheels.
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

  /**
   * @private
   */
  std::string get_type() const override { return type; }

 private:
  inline static std::string type = register_type<Holonomic>("Holonomic");
};

/**
 * @brief      Kinematics for non-wheeled agents that head towards where they move
 * (e.g., people)
 */
class HL_NAVIGATION_EXPORT Forward : public Kinematics {
 public:
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  max_speed          The maximal speed
   * @param[in]  max_angular_speed  The maximal angular speed
   */
  Forward(float max_speed = 0.0f, float max_angular_speed = 0.0f)
      : Kinematics(max_speed, max_angular_speed) {}

  /**
   * @private
   */
  Twist2 feasible(const Twist2& twist) const override;
  /**
   * @brief      Returns whether the kinematics has wheels.
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

  /**
   * @private
   */
  std::string get_type() const override { return type; }

 private:
  inline static std::string type = register_type<Forward>("Forward");
};

/**
 * @brief      Abstract wheeled kinematics
 * 
 * *Properties*: wheel_axis (float) 
 */
class HL_NAVIGATION_EXPORT Wheeled : public Kinematics {
 public:
  Wheeled(float max_speed, float max_angular_speed, float axis)
      : Kinematics(max_speed, max_angular_speed), axis(axis) {}

  virtual ~Wheeled() = default;

  /**
   * @brief      Returns whether the kinematics has wheels.
   *
   * @return     True
   */
  bool is_wheeled() const override { return true; }

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
   * @private
   */
  Twist2 feasible(const Twist2& value) const override;

  /**
   * @brief      Gets the wheel axis.
   *
   * @return     The axis.
   */
  float get_axis() const { return axis; }
  /**
   * @brief      Sets the wheel axis.
   *
   * @param[in]  value  A positive value
   */
  void set_axis(float value) {
    if (value > 0) axis = value;
  }

  /**
   * @private
   */
  virtual const Properties& get_properties() const override {
    return properties;
  };

  /**
   * @private
   */
  static inline std::map<std::string, Property> properties = Properties{
      {"wheel_axis",
       make_property<float, Wheeled>(&Wheeled::get_axis, &Wheeled::set_axis,
                                     0.0f, "Wheel Axis")},
  };

 protected:
  float axis;
};

/**
 * @brief      Differential two-wheeled robot (e.g., a wheelchair)
 */
class HL_NAVIGATION_EXPORT TwoWheeled : public Wheeled {
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

  /**
   * @private
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
  Twist2 twist(const WheelSpeeds& speeds) const override;

  /**
   * @brief      See \ref Wheeled::wheel_speeds.
   *
   * @param[in]  twist  The twist
   *
   * @return     The corresponding wheel speeds in the order {left, right}
   */
  WheelSpeeds wheel_speeds(const Twist2& twist) const override;

  /**
   * @private
   */
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
class HL_NAVIGATION_EXPORT FourWheeled : public Wheeled {
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
   * @private
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
  Twist2 twist(const WheelSpeeds& speeds) const override;

  /**
   * @brief      See \ref Wheeled::wheel_speeds.
   *
   * @param[in]  twist  The twist
   *
   * @return     The corresponding wheel speeds in the order {front left, rear
   * left, rear right, rear left}
   */
  WheelSpeeds wheel_speeds(const Twist2& twist) const override;

  /**
   * @private
   */
  std::string get_type() const override { return type; }

 private:
  inline static std::string name = register_type<FourWheeled>("FourWheeled");
};

}  // namespace hl_navigation

#endif /* end of include guard: HL_NAVIGATION_KINEMATICS_H_ */
