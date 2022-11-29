/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef _AGENT_H_
#define _AGENT_H_

#include "common.h"
#include <algorithm>
#include <list>
#include <map>
#include <algorithm>

using namespace argos;

// mobility

#define DEFAULT_LINEAR_SPEED_CONTINUOUS false
#define DEFAULT_MAX_ANGULAR_SPEED 0.2 // 20 //cm/s
#define MIN_ANGULAR_SPEED 0.02
#define MAX_SPEED 1.0 // 0.2
#define DEFAULT_ROTATION_TAU 0.5
#define DEFAULT_HORIZON 3
// configuration

#define DEFAULT_OPTIMAL_SPEED 0.6
//#define DEFAULT_SOCIAL_SENSING_RATIO 1
//#define DEFAULT_FOOTBOT_SOCIAL_RADIUS 20 //40
//#define DEFAULT_HUMAN_SOCIAL_RADIUS 40
//#define DEFAULT_OBSTACLE_SOCIAL_RADIUS 20
#define RADIUS 0.3 // Footbot's radius
//#define HUMAN_RADIUS 0.05//0.25//m

#define NO_COLLISION -1
// Agent Types

// enum {FOOTBOT=0,OBSTACLE=2,HUMAN=1};
//#define NUMBER_OF_AGENT_TYPES 3

#define ANGULAR_SPEED_DOMINATE

typedef enum { HOLONOMIC, TWO_WHEELED, HEAD } agentType;



class Agent {

using CreateMethod = std::function<std::unique_ptr<Agent>()>;

public:
  void stop();
  // Type

  agentType type;

  // Mobility

  Real optimalSpeed;

  CVector2 desiredVelocity;

  Real desiredSpeed;
  CRadians desiredAngle;
  CRadians desiredAngularSpeed;
  Real maxSpeed;
  CVector2 position;
  CVector2 velocity;
  CRadians angularSpeed;
  CRadians optimalAngularSpeed;
  double optimalRotationSpeed;
  CRadians maxAngularSpeed;
  CRadians angle;
  Real rotationTau;

  ////Two wheeled

  Real axisLength;
  Real maxRotationSpeed;
  bool linearSpeedIsContinuos;

  Real leftWheelSpeed;
  Real rightWheelSpeed;
  Real leftWheelDesiredSpeed;
  Real rightWheelDesiredSpeed;
  Real desiredLinearSpeed;

  // Margins

  // Real socialMargin;
  // Real socialRadius[NUMBER_OF_AGENT_TYPES];
  Real safetyMargin;
  Real radius;

  // Navigation
  CVector2 targetPosition;
  Real horizon;

  // Repulsive Force

  CVector2 repulsiveForce;
  bool insideObstacle;

  void setHorizon(double value);
  virtual void setTimeHorizon(double value) = 0;
  virtual void setAperture(double value) = 0;
  virtual void setResolution(unsigned int value) = 0;
  virtual void setTau(double value) = 0;
  virtual void setEta(double value) = 0;

  virtual void updateRepulsiveForce() = 0;
  virtual void updateDesiredVelocity() = 0;
  virtual void updateVelocity(float);
  void updatePolarVelocity();
  virtual void clearObstacles() = 0;

  virtual void addObstacleAtPoint(CVector2 p, CVector2 v, Real r,
                                  Real socialMargin) = 0;
  virtual void addObstacleAtPoint(CVector2 p, Real r, Real socialMargin) = 0;

  Agent();
  ~Agent(){};

  void setMaxAngularSpeed(double value);
  void setMaxRotationSpeed(double value);
  void setOptimalSpeed(double value);
  void setOptimalAngularSpeed(double value);
  void setOptimalRotationSpeed(double value);
  void setMaxSpeed(double value);
  void setRotationTau(double value);
  void setSafetyMargin(double value);


protected:
  Real marginForObstacleAtDistance(Real distance, Real obstacleRadius,
                                   Real safetyMargin, Real socialMargin);
  CVector2 relativePositionOfObstacleAt(CVector2 &position, Real obstacleRadius,
                                        Real &distance);
  void setDesiredWheelSpeeds(double left, double right);

  static std::map<std::string, CreateMethod> _agent_create_functions;
  template<typename T>
  static const char * register_type(const char * name) {
    _agent_create_functions[name] = [](){ return std::make_unique<T>();};
    return name;
  }

private:
  virtual void setup() = 0;

public:
  static std::unique_ptr<Agent> agent_with_name(const std::string & name) {
    if (_agent_create_functions.count(name)) {
      return _agent_create_functions[name]();
    }
    return nullptr;
  }

  static const std::vector<std::string> behavior_names() {
    std::vector<std::string> keys;
    std::transform(_agent_create_functions.begin(), _agent_create_functions.end(),
                   back_inserter(keys),
                   [](std::pair<std::string, CreateMethod> p) { return p.first;});
    return keys;
  }

};

#endif
