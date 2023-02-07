/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef _AGENT_H_
#define _AGENT_H_

#include "common.h"
#include <algorithm>
#include <map>
#include <memory>
#include <tuple>
#include <vector>


#define DEFAULT_ROTATION_TAU 0.5

namespace hl_navigation {

struct Twist2D {
  float longitudinal;
  float lateral;
  float angular;

  Twist2D(float vx, float vy, float wz) :
    longitudinal(vx), lateral(vy), angular(wz) {}
};

struct Disc {
  Vector2 position;
  Vector2 velocity;
  float radius;
  float social_margin;

  Disc(const Vector2 p, float r,float s = 0.0, Vector2 v = Vector2(0, 0)) :
    position(p), velocity(v), radius(r), social_margin(s) { }
};

struct LineSegment {
  Vector2 p1;
  Vector2 p2;
  Vector2 e1;
  Vector2 e2;
  float length;

  LineSegment(const Vector2 p1, const Vector2 p2) :
    p1(p1), p2(p2), e1((p2-p1).normalized()), e2(-e1[1], e1[0]), length((p2-p1).norm()) { }

  LineSegment(const LineSegment & s) : LineSegment(s.p1, s.p2) { }
};


using WheelSpeeds = std::vector<float>;

typedef enum { HOLONOMIC, TWO_WHEELED, HEAD, FOUR_WHEELED_OMNI } agent_type_t;

typedef enum {IDLE, TARGET_POINT, TARGET_ANGLE, DESIRED_ANGLE} heading_t;

class Behavior {

  using CreateMethod = std::function<std::unique_ptr<Behavior>(agent_type_t, float, float)>;

public:

  Vector2 position;
  Radians angle;
  Vector2 velocity;
  Radians angularSpeed;
  Vector2 targetPosition;
  Radians targetAngle;
  // absolute
  Vector2 desiredVelocity;
  // relative
  Twist2D desired_twist;
  Twist2D target_twist;
  WheelSpeeds desired_wheel_speeds;
  WheelSpeeds target_wheel_speeds;

  Behavior(agent_type_t type, float radius, float axis_length=0.0) :
    position(0.0, 0.0), angle(0.0), velocity(0.0, 0.0), angularSpeed(0.0),
    desiredVelocity(0.0, 0.0), desired_twist(0.0, 0.0, 0.0), target_twist(0.0, 0.0, 0.0),
    type(type), axisLength(axis_length), radius(radius), maxSpeed(10000.0),
    horizon(0.0), safetyMargin(0.0), rotationTau(DEFAULT_ROTATION_TAU), optimalSpeed(0.0),
    heading_behavior(DESIRED_ANGLE), static_obstacles(), neighbors(), line_obstacles() {
      if(type == TWO_WHEELED)
        target_wheel_speeds = std::vector<float>(2, 0.0);
      if(type == FOUR_WHEELED_OMNI)
        target_wheel_speeds = std::vector<float>(4, 0.0);
    }

  virtual ~Behavior() = default;

  Radians get_max_angular_speed() const;
  float get_optimal_speed() const;
  Radians get_optimal_angular_speed() const;
  float get_max_speed() const;
  float get_rotation_tau() const;
  float get_safety_margin() const;
  float get_horizon() const;
  float get_radius() const { return radius; }
  heading_t get_heading_behavior() const { return heading_behavior; }
  void set_max_angular_speed(Radians value);
  void set_optimal_speed(float value);
  void set_optimal_angular_speed(Radians value);
  void set_max_speed(float value);
  void set_rotation_tau(float value);
  void set_safety_margin(float value);
  void set_horizon(float value);
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
  float axisLength;
  float radius;
  Radians maxAngularSpeed;
  float maxSpeed;

  float horizon;
  float safetyMargin;
  float rotationTau;
  float optimalSpeed;

  Radians optimalAngularSpeed;
  heading_t heading_behavior;
  Vector2 repulsiveForce;
  bool insideObstacle;

  std::vector<Disc> static_obstacles;
  std::vector<Disc> neighbors;
  std::vector<LineSegment> line_obstacles;

  virtual void update_desired_velocity() = 0;
  virtual void update_repulsive_force() { };


  virtual void add_neighbor(const Disc & disc) { };
  virtual void add_static_obstacle(const Disc & disc) { };
  virtual void clear() { };
  virtual void prepare() { }

  virtual Twist2D compute_desired_twist() const;

  float marginForObstacleAtDistance(float distance, float obstacleRadius,
                                   float safetyMargin, float socialMargin);
  Vector2 relativePositionOfObstacleAt(Vector2 &position, float obstacleRadius,
                                        float &distance);
  void setDesiredWheelSpeeds(float left, float right);

  static std::map<std::string, CreateMethod> _agent_create_functions;
  template <typename T> static const char *register_type(const char *name) {
    _agent_create_functions[name] = [](agent_type_t type, float radius, float axis_length=0.0) {
      return std::make_unique<T>(type, radius, axis_length); };
    return name;
  }

public:
  Twist2D twist_from_wheel_speeds(const WheelSpeeds & speeds) const;
  WheelSpeeds wheel_speeds_from_twist(const Twist2D &) const;
  Vector2 get_target_velocity() const;
  void set_wheel_speeds(const WheelSpeeds & speeds);
  bool is_wheeled() const;
  bool is_omnidirectional() const;
  static std::unique_ptr<Behavior> behavior_with_name(
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

}

#endif
