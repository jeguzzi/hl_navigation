/**
 * @author Jérôme Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_COMMON_H_
#define HL_NAVIGATION_COMMON_H_

#include <math.h>
#include <stdio.h>
#include <time.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

typedef unsigned int uint;

namespace hl_navigation {

/**
 * A two-dimensional vector, see <a
 * href="https://eigen.tuxfamily.org/dox/group__matrixtypedefs.html">Eigen</a>
 */
using Vector2 = Eigen::Vector2f;
/**
 * Angle in radians.
 */
using Radians = float;
/**
 * A vector that holds wheel speeds. The order depends on the kinematic.
 */
using WheelSpeeds = std::vector<float>;

/**
 * @brief      The orientation of a two dimensional vector
 *
 * @param[in]  vector
 *
 * @return     The polar angle
 */
inline Radians polar_angle(const Vector2& vector) {
  return std::atan2(vector.y(), vector.x());
}

/**
 * @brief      Normalize an angle to a value in [-pi, pi]
 *
 * Useful/needed when compute angular differences
 *
 * @param[in]  value
 *
 * @return     The same angle expressed with a value in [-pi, pi]
 */
inline Radians normalize(Radians value) {
  value = std::fmod(value, 2 * M_PI);
  if (value < -M_PI) {
    value += 2 * M_PI;
  } else if (value > M_PI) {
    value -= 2 * M_PI;
  }
  return value;
}

/**
 * @brief      Unit vector towards an angle
 *
 * @param[in]  angle
 *
 * @return     Vector of norm one oriented towards the angle
 */
inline Vector2 unit(float angle) { return {cosf(angle), sinf(angle)}; }

/**
 * @brief      Rotate a two-dimensional vector
 *
 * @param[in]  vector
 * @param[in]  angle  The rotation angle in radians
 *
 * @return     The rotated vector
 */
inline Vector2 rotate(const Vector2 vector, float angle) {
  Eigen::Rotation2D<float> rot(angle);
  return rot * vector;
}

/**
 * @brief      Clamp the norm of a vector
 *
 * @param[in]  vector
 * @param[in]  max_length  The maximum length
 *
 * @return     A vector with the same direction clamped to max_length
 */
inline Vector2 clamp_norm(const Vector2& vector, float max_length) {
  float n = vector.norm();
  if (n > 0 && n > max_length) {
    return vector / n * max_length;
  }
  return vector;
}

/**
 * @brief      Two-dimensional twist composed of planar velocity and angular
 * speed.
 *
 * Twist coordinates may be in a fixed frame or in the agent's own frame, as
 * specified by \ref relative.
 */
struct Twist2 {
  /**
   * Velocity
   */
  Vector2 velocity;
  /**
   * Angular speed
   */
  Radians angular_speed;
  /**
   * If true, the twist is relative to the agent's own frame of reference.
   */
  bool relative;

  Twist2(const Vector2& velocity, Radians angular_speed = 0.0,
         bool relative = false)
      : velocity(velocity), angular_speed(angular_speed), relative(relative) {}
  Twist2() : Twist2({0.0f, 0.0f}) {}
  /**
   * @brief      Rotate the twist by an angle.
   *
   * @param[in]  angle  The rotation angle in radians.
   *
   * @return     The rotated twist.
   */
  Twist2 rotate(Radians angle) const {
    return {::hl_navigation::rotate(velocity, angle), angular_speed};
  }
};

/**
 * @brief      Two-dimensional pose composed of planar position and
 * orientatation.
 *
 * Poses are assumed to be a world fixed frame.
 */
struct Pose2 {
  /**
   * Position in world frame
   */
  Vector2 position;
  /**
   * Orientation in world frame
   */
  Radians orientation;

  Pose2(const Vector2& position, Radians orientation = 0.0)
      : position(position), orientation(orientation) {}
  Pose2() : Pose2({0.0f, 0.0f}) {}
  /**
   * @brief      Rotate the pose by an angle.
   *
   * @param[in]  angle  The rotation angle in radians.
   *
   * @return     The rotated pose.
   */
  Pose2 rotate(Radians angle) const {
    return {::hl_navigation::rotate(position, angle), orientation + angle};
  }

  /**
   * @brief      Integrate a pose
   *
   * @param[in]  twist  The twist (in agent or world frame)
   * @param[in]  dt     The time step
   *
   * @return     pose + dt * twist  (in world frame)
   */
  Pose2 integrate(const Twist2& twist, float dt) {
    return {position + dt * (twist.relative ? ::hl_navigation::rotate(
                                                  twist.velocity, orientation)
                                            : twist.velocity),
            orientation + dt * twist.angular_speed};
  }
};

class RegisterChanges {
 public:
  RegisterChanges() : changes{0xFFFFFFFF} {}
  bool changed(unsigned mask = 0xFFFFFFFF) const { return changes & mask; }
  void reset_changes() { changes = 0; }
  void change(unsigned mask) { changes |= mask; }

 private:
  unsigned changes;
};

}  // namespace hl_navigation

#endif  // HL_NAVIGATION_COMMON_H_
