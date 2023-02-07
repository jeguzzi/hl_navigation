/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "behavior.h"

using Vector3 = Eigen::Vector3f;

namespace hl_navigation {

struct Cylinder {
  Vector3 position;
  Vector3 velocity;
  float radius;
  float social_margin;
  float height;

  Cylinder(const Vector3 p, float r, float height = -1.0, float s = 0.0,
           Vector3 v = Vector3::Zero()) :
    position(p), velocity(v), radius(r), social_margin(s), height(height) { }

  Disc disc() const {
    return {position.head<2>(), radius, social_margin, velocity.head<2>()};
  }
};


class Controller {
 public:
  typedef enum { MOVE, TURN, IDLE, BRAKING } State;

private:

  enum { POSE, POINT } target_type;
  bool target_has_z;
  bool is_at_target_point();
  bool is_at_target_angle();
  void update_target_state();
  void turn();
  void brake();
  void update_vertical_velocity();
  bool limit_to_2d;

  // Navigation parameters

protected:
  float z;
  bool has_z;
  float targetZ;
  float targetDistance;
  float velocityZ;
  float tauZ;
  float optimalVerticalSpeed;
  bool rotateIfHolo;
  virtual void arrived() {};
  virtual void aborted() {};
  virtual void updated_control() {};
  virtual void updated() {};
  virtual void set_target_twist(const Twist2D & twist, float vertical_speed);
  virtual bool localized() {return true;}
public:
  Radians angle_tolerance;
  float distance_tolerance;
  float speed_tolerance;
  void stop();
  void set_pose(const Vector2 & xy, Radians theta);
  void set_pose(const Vector3 & xyz, Radians theta);
  void set_target_point(const Vector2 & xy);
  void set_target_point(const Vector3 & xyz);
  void set_target_pose(const Vector2 & xy, Radians theta);
  void set_target_pose(const Vector3 & xyz, Radians theta);
  void set_neighbors(const std::vector<Disc> & neighbors);
  void set_neighbors(const std::vector<Cylinder> & neighbors);
  bool in_3d() const {
    return !limit_to_2d && has_z;
  }

  State state;
  Behavior *behavior;
  void update(float dt);
  Controller(bool limit_to_2d = false);
  ~Controller();
};

}

#endif /* _CONTROLLER_H_ */
