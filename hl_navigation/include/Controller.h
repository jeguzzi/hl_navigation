/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "Agent.h"

class Controller {
 public:
  typedef enum { MOVE, TURN, IDLE, BRAKING } State;

private:

  enum { POSE, POINT } target_type;

  bool is_at_target_point();
  bool is_at_target_angle();
  void update_target_state();
  void turn();
  void brake();
  void update_vertical_velocity();

  // Navigation parameters

protected:
  double z;
  double targetZ;
  double targetDistance;
  double velocityZ;
  double tauZ;
  double optimalVerticalSpeed;

  bool rotateIfHolo;
  virtual void arrived() {};
  virtual void aborted() {};
  virtual void updated_control() {};
  virtual void updated() {};
  virtual void set_target_twist(const Twist2D & twist, float vertical_speed);
  virtual bool localized() {return true;}
public:
  double angle_tolerance;
  double distance_tolerance;
  double speed_tolerance;
  void stop();
  void set_pose(float x, float y, float z, float theta);
  void set_target_point(float x, float y, float z);
  void set_target_pose(float x, float y, float z, float theta);
  void set_target_point(const CVector2 & point);

  State state;
  Agent *agent;
  void update(float dt);
  Controller();
  ~Controller();
};

#endif /* _CONTROLLER_H_ */
