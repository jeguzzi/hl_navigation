/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef _AGENT_H_
#define _AGENT_H_

#include "common.h"
#include <algorithm>
#include <list>
#include <map>
#include <tuple>
#include <vector>

// using namespace argos;

#define DEFAULT_ROTATION_TAU 0.5

struct Twist2D {
  float longitudinal;
  float lateral;
  float angular;

  Twist2D(float vx, float vy, float wz) :
    longitudinal(vx), lateral(vy), angular(wz) {}
};

struct Disc {
  CVector2 position;
  CVector2 velocity;
  float radius;
  float social_margin;

  Disc(const CVector2 p, float r,float s = 0.0, CVector2 v = CVector2(0, 0)) :
    position(p), velocity(v), radius(r), social_margin(s) { }
};

struct LineSegment {
  CVector2 p1;
  CVector2 p2;
  CVector2 e1;
  CVector2 e2;
  float length;

  LineSegment(const CVector2 p1, const CVector2 p2) :
    p1(p1), p2(p2), e1((p2-p1).normalized()), e2(-e1[1], e1[0]), length((p2-p1).norm()) { }

  LineSegment(const LineSegment & s) : LineSegment(s.p1, s.p2) { }
};


using WheelSpeeds = std::vector<float>;

typedef enum { HOLONOMIC, TWO_WHEELED, HEAD, FOUR_WHEELED_OMNI } agent_type_t;

typedef enum {IDLE, TARGET_POINT, TARGET_ANGLE, DESIRED_ANGLE} heading_t;

class Agent {

  using CreateMethod = std::function<std::unique_ptr<Agent>(agent_type_t, float, float)>;

public:

  CVector2 position;
  CRadians angle;
  CVector2 velocity;
  CRadians angularSpeed;
  CVector2 targetPosition;
  CRadians targetAngle;
  // absolute
  CVector2 desiredVelocity;
  // relative
  Twist2D desired_twist;
  Twist2D target_twist;
  WheelSpeeds desired_wheel_speeds;
  WheelSpeeds target_wheel_speeds;

  Agent(agent_type_t type, float radius, float axis_length=0.0) :
    position(0.0, 0.0), angle(0.0), velocity(0.0, 0.0), angularSpeed(0.0),
    desiredVelocity(0.0, 0.0), desired_twist(0.0, 0.0, 0.0), target_twist(0.0, 0.0, 0.0),
    type(type), axisLength(axis_length), radius(radius), maxSpeed(10000.0),
    horizon(0.0), safetyMargin(0.0), rotationTau(DEFAULT_ROTATION_TAU), optimalSpeed(0.0),
    heading_behavior(DESIRED_ANGLE), static_obstacles(), neighbors() {
      if(type == TWO_WHEELED)
        target_wheel_speeds = std::vector<float>(2, 0.0);
      if(type == FOUR_WHEELED_OMNI)
        target_wheel_speeds = std::vector<float>(4, 0.0);
    }

  virtual ~Agent() = default;

  CRadians get_max_angular_speed() const;
  double get_optimal_speed() const;
  CRadians get_optimal_angular_speed() const;
  double get_max_speed() const;
  double get_rotation_tau() const;
  double get_safety_margin() const;
  double get_horizon() const;
  double get_radius() const { return radius; }
  heading_t get_heading_behavior() const { return heading_behavior; }
  void set_max_angular_speed(double value);
  void set_optimal_speed(double value);
  void set_optimal_angular_speed(double value);
  void set_max_speed(double value);
  void set_rotation_tau(double value);
  void set_safety_margin(double value);
  void set_horizon(double value);
  void set_desired_twist(const Twist2D & twist);
  void set_heading_behavior(heading_t value) {
    if (is_omnidirectional()) {
      heading_behavior = value;
    } else {
      heading_behavior = DESIRED_ANGLE;
    }
  }

  void update(float dt);
  virtual void update_target_twist(float dt);
  void set_neighbors(const std::vector<Disc> & value) {
    neighbors = value;
  }
  void set_static_obstacles(const std::vector<Disc> & value) {
    static_obstacles = value;
  }

  virtual void set_line_obstacles(const std::vector<LineSegment> & value) {
    line_obstacles = value;
  }

protected:

  agent_type_t type;
  Real axisLength;
  Real radius;
  CRadians maxAngularSpeed;
  Real maxSpeed;

  Real horizon;
  Real safetyMargin;
  Real rotationTau;
  Real optimalSpeed;

  CRadians optimalAngularSpeed;
  heading_t heading_behavior;
  CVector2 repulsiveForce;
  bool insideObstacle;

  std::vector<Disc> static_obstacles;
  std::vector<Disc> neighbors;
  std::vector<LineSegment> line_obstacles;

  virtual void update_desired_velocity() = 0;
  virtual void update_repulsive_force() { };


  virtual void add_neighbor(const Disc & disc) = 0;
  virtual void add_static_obstacle(const Disc & disc) = 0;
  virtual void clear() = 0;
  virtual void prepare() { }

  virtual Twist2D compute_desired_twist() const;

  Real marginForObstacleAtDistance(Real distance, Real obstacleRadius,
                                   Real safetyMargin, Real socialMargin);
  CVector2 relativePositionOfObstacleAt(CVector2 &position, Real obstacleRadius,
                                        Real &distance);
  void setDesiredWheelSpeeds(double left, double right);

  static std::map<std::string, CreateMethod> _agent_create_functions;
  template <typename T> static const char *register_type(const char *name) {
    _agent_create_functions[name] = [](agent_type_t type, float radius, float axis_length=0.0) {
      return std::make_unique<T>(type, radius, axis_length); };
    return name;
  }

public:
  Twist2D twist_from_wheel_speeds(const WheelSpeeds & speeds) const;
  WheelSpeeds wheel_speeds_from_twist(const Twist2D &) const;
  CVector2 get_target_velocity() const;
  void set_wheel_speeds(const WheelSpeeds & speeds);
  bool is_wheeled() const;
  bool is_omnidirectional() const;
  static std::unique_ptr<Agent> agent_with_name(
      const std::string &name, agent_type_t type, float radius, float axis_length=0.0) {
    if (_agent_create_functions.count(name)) {
      return _agent_create_functions[name](type, radius, axis_length);
    }
    return nullptr;
  }

  static const std::map<std::string, CreateMethod> & all_behaviors() {
    return _agent_create_functions;
  };

  static const std::vector<std::string> behavior_names() {
    std::vector<std::string> keys;
    std::transform(
        _agent_create_functions.begin(), _agent_create_functions.end(),
        back_inserter(keys),
        [](std::pair<std::string, CreateMethod> p) { return p.first; });
    return keys;
  }
};

#endif
