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

  enum { POSE, POINT } targetType;

  bool is_at_target_point();
  bool is_at_target_angle();
  void updateTargetState();

  void turn();
  void brake();

  void setMotorVelocity(double speed, double verticalVelocity,
                        double angularSpeed);
  void updateVerticalVelocity();

  // Navigation parameters

protected:
  double z;
  double targetZ;
  double targetAngle;
  double targetDistance;
  double velocityZ;
  double tauZ;
  double optimalVerticalSpeed;

  bool rotateIfHolo;
  virtual void arrived() {};
  virtual void aborted() {};
  virtual void updated_control() {};
  virtual void updated() {};
  virtual void setMotorVelocity(double newVelocityX, double newVelocityY,
                                double verticalVelocity, double newAngularSpeed) {};
  virtual bool localized() {return true;}
public:
  double minDeltaAngle;
  double minDeltaDistance;
  double minimalSpeed;
  void stop();
  void setPose(float x, float y, float z, float theta);
  void setTargetPoint(float x, float y, float z);
  void setTargetPose(float x, float y, float z, float theta);
  State state;
  Agent *agent;
  void update(float dt);
  Controller();
  ~Controller();
};

#endif /* _CONTROLLER_H_ */
