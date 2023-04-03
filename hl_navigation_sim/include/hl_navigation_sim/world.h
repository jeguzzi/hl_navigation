/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_SIM_WORLD_H_
#define HL_NAVIGATION_SIM_WORLD_H_

#include <geos/geom/Envelope.h>
#include <geos/index/strtree/TemplateSTRtree.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "hl_navigation/behavior.h"
#include "hl_navigation/common.h"
#include "hl_navigation/controller.h"
#include "hl_navigation/property.h"
#include "hl_navigation/register.h"
#include "hl_navigation/states/geometric.h"
#include "hl_navigation/yaml/yaml.h"

namespace hl_navigation_sim {

using Waypoints = std::vector<hl_navigation::Vector2>;
using BoundingBox = geos::geom::Envelope;
using namespace hl_navigation;

struct Entity {
  static inline unsigned _uid = 0;

  Entity() : uid(_uid++) {}

  unsigned uid;
};

class Agent;
class World;

struct Task : public virtual HasProperties, public virtual HasRegister<Task> {
  explicit Task() {}
  virtual void update([[maybe_unused]] Agent *agent) {}
  virtual bool done() const { return false; }
  virtual ~Task() = default;

  /*
  virtual const std::string &get_type() const = 0;
  using Factory = std::function<std::shared_ptr<Task>()>;
  static std::shared_ptr<Task> task_with_name(const std::string &name) {
    if (factory.count(name)) {
      return factory[name]();
    }
    return nullptr;
  }
  static const std::map<std::string, Factory> &all_kinematics() {
    return factory;
  }
  static const std::map<std::string, Properties> &all_properties() {
    return factory_properties;
  }
  static inline std::map<std::string, Factory> factory = {};
  static inline std::map<std::string, Properties> factory_properties = {};
  template <typename T>
  static std::string register_type(const std::string &name) {
    factory[name] = []() { return std::make_shared<T>(); };
    factory_properties[name] = T::properties;
    return name;
  }
  static inline std::map<std::string, Property> properties = Properties{};
  */

#if 0
  std::string description(bool extensive = false) const {
    std::ostringstream os;
    os << get_type();
    if (extensive) {
      os << ":" << std::endl;
      for (const auto &[k, v] : get_properties()) {
        os << "  " << k << " = " << get(k) << "[" << v.type_name << "]"
           << std::endl;
      }
    }
    return os.str();
  }
#endif
};

struct StateEstimation : public virtual HasProperties,
                         public virtual HasRegister<StateEstimation> {
  explicit StateEstimation(World *world = nullptr) : world(world) {}

  virtual void update([[maybe_unused]] Agent *agent) const {};

  virtual void prepare([[maybe_unused]] Agent *agent) const {};

  virtual ~StateEstimation() = default;

  World *world;

#if 0
  std::string description(bool extensive = false) const {
    std::ostringstream os;
    os << get_type();
    if (extensive) {
      os << ":" << std::endl;
      for (const auto &[k, v] : get_properties()) {
        os << "  " << k << " = " << get(k) << "[" << v.type_name << "]"
           << std::endl;
      }
    }
    return os.str();
  }
#endif

  /*
    virtual const std::string &get_type() const = 0;
    using Factory = std::function<std::shared_ptr<StateEstimation>()>;
    static std::shared_ptr<StateEstimation> state_estimation_with_name(
        const std::string &name) {
      if (factory.count(name)) {
        return factory[name]();
      }
      return nullptr;
    }
    static const std::map<std::string, Factory> &all_kinematics() {
      return factory;
    }
    static const std::map<std::string, Properties> &all_properties() {
      return factory_properties;
    }
    static inline std::map<std::string, Factory> factory = {};
    static inline std::map<std::string, Properties> factory_properties = {};
    template <typename T>
    static std::string register_type(const std::string &name) {
      factory[name] = []() { return std::make_shared<T>(); };
      factory_properties[name] = T::properties;
      return name;
    }

    static inline std::map<std::string, Property> properties = Properties{};
    */
};

class Agent : public Entity {
 public:
  using C = std::shared_ptr<Agent>;
  using B = Behavior;
  using K = Kinematic;
  using T = Task;
  using S = StateEstimation;

  virtual ~Agent() = default;

  Agent(float radius = 0.0f, std::shared_ptr<Behavior> behavior = nullptr,
        std::shared_ptr<Kinematic> kinematic = nullptr,
        std::shared_ptr<Task> task = nullptr,
        std::shared_ptr<StateEstimation> estimation = nullptr,
        float control_period = 0.0f, unsigned id = 0)
      : Entity(),
        id(id),
        radius(radius),
        mass(0.0f),
        control_period(control_period),
        control_deadline(0.0),
        task(task),
        state_estimation(estimation),
        nav_behavior(behavior),
        kinematic(kinematic),
        nav_controller(nav_behavior, false, true), type("") {}

  static std::shared_ptr<Agent> make(
      float radius = 0.0f, std::shared_ptr<Behavior> behavior = nullptr,
      std::shared_ptr<Kinematic> kinematic = nullptr,
      std::shared_ptr<Task> task = nullptr,
      std::shared_ptr<StateEstimation> estimation = nullptr,
      float control_period = 0.0f, unsigned id = 0) {
    return std::make_shared<Agent>(radius, behavior, kinematic, task,
                                   estimation, control_period, id);
  }

  virtual GeometricState *get_geometric_state() const {
    return dynamic_cast<GeometricState *>(nav_behavior.get());
  }

#if 0
  std::string description(bool extensive = false) const {
    std::ostringstream os;
    os << "id = " << id << std::endl;
    os << "radius = " << radius << std::endl;
    os << "control_period = " << control_period << std::endl;
    os << "x " << pose.position.x() << std::endl;
    os << "pose = " << pose << std::endl;
    os << "twist = " << twist << std::endl;
    if (kinematic) {
      os << "kinematic: " << kinematic->description(extensive) << std::endl;
    }
    if (nav_behavior) {
      os << "navigation behavior: " << nav_behavior->description(extensive)
         << std::endl;
    }
    if (state_estimation) {
      os << "state_estimation: " << state_estimation->description(extensive)
         << std::endl;
    }
    if (task) {
      os << "task: " << task->description(extensive) << std::endl;
    }

    return os.str();
  }
#endif
  void update(float dt) {
    control_deadline -= dt;
    if (control_deadline > 0) {
      return;
    }
    control_deadline += control_period;
    // bool v1 = dynamic_cast<GeometricState *>(nav_behavior.get()) != nullptr;
    // std::cout << "Agent::update: behavior is geometric? " << nav_behavior <<
    // " " << v1 << std::endl;

    if (task) task->update(this);
    if (state_estimation) state_estimation->update(this);
    if (nav_behavior) {
      nav_behavior->set_actuated_twist(cmd_twist);
      nav_behavior->set_twist(twist);
      nav_behavior->set_pose(pose);
    }
    cmd_twist = nav_controller.update(std::max(control_period, dt));
    // cmd_twist = nav_behavior->get_actuated_twist(true);
  }

  void update_physics(float dt) {
    twist = cmd_twist;  // + collision_force / mass * dt;
    pose = pose.integrate(twist, dt);
  }

  void set_state_estimation(const std::shared_ptr<StateEstimation> &value) {
    state_estimation = value;
  }

  std::shared_ptr<StateEstimation> get_state_estimation() const {
    return state_estimation;
  }

  void set_behavior(const std::shared_ptr<Behavior> &value) {
    nav_behavior = value;
    nav_controller.set_behavior(value);
    if (nav_behavior) {
      nav_behavior->set_radius(radius);
      if (!nav_behavior->get_kinematic()) {
        nav_behavior->set_kinematic(kinematic);
      }
    }
  }

  std::shared_ptr<Behavior> get_behavior() const { return nav_behavior; }

  void set_kinematic(const std::shared_ptr<Kinematic> &value) {
    kinematic = value;
    if (nav_behavior && !nav_behavior->get_kinematic()) {
      nav_behavior->set_kinematic(kinematic);
    }
  }

  std::shared_ptr<Kinematic> get_kinematic() const { return kinematic; }

  void set_task(const std::shared_ptr<Task> &value) { task = value; }

  std::shared_ptr<Task> get_task() const { return task; }

  Controller *get_controller() { return &nav_controller; }

  unsigned id;
  float radius;
  float mass;
  float control_period;
  float control_deadline;
  Pose2 pose;
  Twist2 twist;
  Twist2 cmd_twist;
  // Eigen::Vector2f collision_force;
  Vector2 collision_correction;
  std::shared_ptr<Task> task;
  std::shared_ptr<StateEstimation> state_estimation;
  std::shared_ptr<Behavior> nav_behavior;
  std::shared_ptr<Kinematic> kinematic;
  Controller nav_controller;
  std::string type;
};

inline std::optional<Vector2> penetration(const LineSegment &line,
                                          const Vector2 &center, float radius) {
  float y = (center - line.p1).dot(line.e2);
  if (abs(y) < radius) {
    float x = (center - line.p1).dot(line.e1);
    if (x < radius + 1e-3 || x > line.length - radius - 1e-3)
      return std::nullopt;
    float p = radius - abs(y);
    if (y < 0) p *= -1;
    return p * line.e2;
  }
  return std::nullopt;
}

struct Wall : Entity {
  Wall(const Vector2 &p1, const Vector2 &p2) : Entity(), line(p1, p2) {}
  Wall() : Entity(), line() {}
  Wall(const LineSegment &ls) : Entity(), line(ls) {}
  operator LineSegment() const { return line; }

  LineSegment line;
};

struct Obstacle : Entity {
  Obstacle(const Vector2 &position, float radius)
      : Entity(), disc(position, radius) {}
  Obstacle() : Entity(), disc() {}
  Obstacle(const Disc &disc) : Entity(), disc(disc) {}
  operator Disc() const { return disc; }

  Disc disc;
};

class World {
 public:
  virtual ~World() = default;

  using A = Agent;
  explicit World()
      : agents(), obstacles(), walls(), agent_index(nullptr), ready(false), time(0.0f) {}

  void update(float time_step) {
    if (!ready) {
      prepare();
      ready = true;
    }
    for (auto &a : agents) {
      a->update(time_step);
    }
    for (auto &a : agents) {
      a->update_physics(time_step);
    }
    update_agents_strtree();
    update_collisions();
    time += time_step;
  }

  void add_agent(const std::shared_ptr<Agent> &agent) {
    // bool v1 = dynamic_cast<GeometricState *>(agent->nav_behavior.get()) !=
    // nullptr; std::cout << "add_agent: behavior is geometric? " <<
    // agent->nav_behavior << " " << v1 << std::endl;
    if (agent) {
      agents.push_back(agent);
      ready = false;
    }
  }
  void add_wall(const LineSegment &wall) {
    walls.push_back(wall);
    ready = false;
  }
  void add_obstacle(const Disc &obstacle) {
    obstacles.push_back(obstacle);
    ready = false;
  }

  void prepare() {
    // TODO(Jerome) Should only execute it once if not already prepared.
    update_static_strtree();
    update_agents_strtree();

    for (auto &a : agents) {
      if (a->state_estimation) {
        a->state_estimation->world = this;
        a->state_estimation->prepare(a.get());
      }
      a->collision_correction = Vector2::Zero();
      if (a->nav_behavior) {
        a->nav_behavior->set_kinematic(a->kinematic);
        a->nav_behavior->set_radius(a->radius);
        a->nav_controller.set_behavior(a->nav_behavior);
      }
    }
  }

  void run(unsigned steps, float time_step) {
    for (size_t i = 0; i < steps; i++) {
      update(time_step);
    }
  }

  std::vector<Agent *> get_neighbors(const BoundingBox &bb) const {
    std::vector<Agent *> rs;
    // std::transform(agents.cbegin(), agents.cend(), std::back_inserter(rs),
    // [](const auto & a) { return &a; });
    agent_index->query(bb, rs);
    return rs;
  }

  std::vector<Disc> get_obstacles() const {
    std::vector<Disc> discs(obstacles.size());
    std::transform(obstacles.cbegin(), obstacles.cend(), discs.begin(),
                   [](const auto &o) { return o.disc; });
    return discs;
  }

  // TODO(J): complete
  std::vector<Disc *> get_obstacles(
      [[maybe_unused]] const BoundingBox &bb) const {
    return {};
  }

  std::vector<LineSegment > get_walls() const {
    std::vector<LineSegment> lines(walls.size());
    std::transform(walls.cbegin(), walls.cend(), lines.begin(),
                   [](const auto &w) { return w.line; });
    return lines;
  }

  // TODO(J): complete
  std::vector<LineSegment *> get_walls(
      [[maybe_unused]] const BoundingBox &bb) const {
    return {};
  }
#if 0
  std::string description(bool extensive = false) const {
    std::ostringstream os;
    os << "walls:" << std::endl;
    for (const auto &wall : walls) {
      os << wall << std::endl;
    }
    os << "obstacles:" << std::endl;
    for (const auto &o : obstacles) {
      os << o << std::endl;
    }
    os << "agents:" << std::endl;
    for (const auto &agent : agents) {
      os << agent->description(extensive);
    }
    return os.str();
  }
#endif

  void set_obstacles(const std::vector<Disc> &values) {
    for (const auto &value : values) {
      add_obstacle(value);
    }
  }

  void set_walls(const std::vector<LineSegment> &values) {
    for (const auto &value : values) {
      add_wall(value);
    }
  }

  float get_time() const {
    return time;
  }

  std::vector<std::shared_ptr<Agent>> agents;
  std::vector<Obstacle> obstacles;
  std::vector<Wall> walls;
  std::shared_ptr<geos::index::strtree::TemplateSTRtree<Agent *>> agent_index;
  std::shared_ptr<geos::index::strtree::TemplateSTRtree<Disc *>>
      obstacles_index;
  std::shared_ptr<geos::index::strtree::TemplateSTRtree<LineSegment *>>
      walls_index;
  std::vector<geos::geom::Envelope> agent_envelops;
  std::vector<geos::geom::Envelope> static_envelops;
  bool ready;
  float time;

 protected:
  void resolve_collision(Agent *a1, Agent *a2) {
    auto delta = a1->pose.position - a2->pose.position;
    float p = delta.norm() - a1->radius - a2->radius;
    if (p > 0) {
      return;
    }
    auto u = delta / delta.norm();
    auto correction = (-p * 0.5 + 1e-3) * u;
    a1->collision_correction += correction;
    a2->collision_correction -= correction;
    float d = a1->twist.velocity.dot(-u);
    if (d > 0) {
      a1->twist.velocity += d * u;
    }
    d = a2->twist.velocity.dot(u);
    if (d > 0) {
      a2->twist.velocity -= d * u;
    }
    // auto force = -p * k * delta / delta.norm();
    // std::cout << force << std::endl;
    // a1->collision_force += force;
    // a2->collision_force -= force;
  }

  // TODO(J): avoid repetitions
  void resolve_collision(Agent *agent, Disc *disc) {
    auto delta = agent->pose.position - disc->position;
    float p = delta.norm() - agent->radius - disc->radius;
    if (p > 0) {
      return;
    }
    auto u = delta / delta.norm();
    auto correction = (-p * 1.0 + 1e-3) * u;
    agent->collision_correction += correction;
    float d = agent->twist.velocity.dot(-u);
    if (d > 0) {
      agent->twist.velocity += d * u;
    }
  }

  void resolve_collision(Agent *agent, LineSegment *line) {
    if (auto p = penetration(*line, agent->pose.position, agent->radius)) {
      auto n = p->norm();
      auto u = *p / n;
      agent->collision_correction = (n + 1e-3) * u;
      float d = agent->twist.velocity.dot(u);
      if (d > 0) {
        agent->twist.velocity += d * u;
      }
    }
  }

  void update_collisions() {
    for (size_t i = 0; i < agents.size(); i++) {
      Agent *a1 = agents[i].get();
      agent_index->query(agent_envelops[i], [this, a1](Agent *a2) {
        if (a1 < a2) resolve_collision(a1, a2);
      });
      obstacles_index->query(agent_envelops[i],
                             [this, a1](Disc *o) { resolve_collision(a1, o); });
      walls_index->query(agent_envelops[i], [this, a1](LineSegment *w) {
        resolve_collision(a1, w);
      });
    }
    for (auto &a : agents) {
      a->pose.position += a->collision_correction;
      a->collision_correction = Vector2::Zero();
    }
    // TODO(Jerome): queryPairs not yet released ... but does more or less the
    // above agent_index->queryPairs([this](Agent * a1, Agent * a2) {
    // resolve_collision(a1, a2); });
  }

  void update_agents_strtree() {
    agent_envelops.clear();
    agent_index =
        std::make_shared<geos::index::strtree::TemplateSTRtree<Agent *>>(
            agents.size());
    for (const auto &agent : agents) {
      auto &bb =
          agent_envelops.emplace_back(agent->pose.position[0] - agent->radius,
                                      agent->pose.position[0] + agent->radius,
                                      agent->pose.position[1] - agent->radius,
                                      agent->pose.position[1] + agent->radius);
      agent_index->insert(&bb, (void *)(agent.get()));
    }
  }

  void update_static_strtree() {
    static_envelops.clear();
    obstacles_index =
        std::make_shared<geos::index::strtree::TemplateSTRtree<Disc *>>(
            obstacles.size());
    walls_index =
        std::make_shared<geos::index::strtree::TemplateSTRtree<LineSegment *>>(
            walls.size());
    // TODO(J): should coordinates be ordered?
    for (const auto &wall : walls) {
      const Vector2 &p1 = wall.line.p1;
      const Vector2 &p2 = wall.line.p2;
      auto &bb = static_envelops.emplace_back(p1[0], p2[0], p1[1], p2[1]);
      walls_index->insert(&bb, (void *)(&wall.line));
    }
    for (const auto &obstacle : obstacles) {
      const Vector2 &p = obstacle.disc.position;
      const float r = obstacle.disc.radius;
      auto &bb =
          static_envelops.emplace_back(p[0] - r, p[0] + r, p[1] - r, p[1] + r);
      obstacles_index->insert(&bb, (void *)(&obstacle.disc));
    }
  }
};

// TODO(Jerome) replace Disk::social margin with Disk::type (unsigned)

struct WayPointsTask : Task {
  using Callback = std::function<void(const Vector2 &)>;

  WayPointsTask(Waypoints waypoints_ = {}, bool loop_ = default_loop,
                float tolerance_ = default_tolerance)
      : Task(),
        waypoints(waypoints_),
        waypoint(waypoints.begin()),
        loop(loop_),
        tolerance(tolerance_),
        callbacks() {}

  virtual ~WayPointsTask() = default;

  void update(Agent *agent) override {
    if (agent->nav_controller.idle()) {
      if (waypoint != waypoints.end()) {
        agent->nav_controller.go_to_position(*waypoint, tolerance);
        for (const auto &cb : callbacks) {
          cb(*waypoint);
        }
        ++waypoint;
        if (loop && waypoint == waypoints.end()) {
          waypoint = waypoints.begin();
        }
      }
    }
  }

  bool done() const override { return waypoint == waypoints.end(); }

  void set_waypoints(const Waypoints &value) {
    waypoints = value;
    waypoint = waypoints.begin();
  }
  void set_tolerance(float value) { tolerance = std::max(value, 0.0f); }
  void set_loop(bool value) { loop = value; }
  Waypoints get_waypoints() const { return waypoints; }
  float get_tolerance() const { return tolerance; }
  float get_loop() const { return loop; }

  void add_callback(const Callback &value) { callbacks.push_back(value); }

  void clear_callbacks() { callbacks.clear(); }

  Waypoints waypoints;
  Waypoints::iterator waypoint;
  bool loop;
  float tolerance;
  std::vector<Callback> callbacks;

  virtual const Properties &get_properties() const override {
    return properties;
  };

  inline static const bool default_loop = true;
  inline static const bool default_tolerance = 1.0f;

  static inline std::map<std::string, Property> properties = Properties{
      {"waypoints",
       make_property<Waypoints, WayPointsTask>(&WayPointsTask::get_waypoints,
                                               &WayPointsTask::set_waypoints,
                                               Waypoints{}, "waypoints")},
      {"loop", make_property<bool, WayPointsTask>(&WayPointsTask::get_loop,
                                                  &WayPointsTask::set_loop,
                                                  default_loop, "loop")},
      {"tolerance",
       make_property<float, WayPointsTask>(&WayPointsTask::get_tolerance,
                                           &WayPointsTask::set_tolerance,
                                           default_tolerance, "tolerance")},
  };

  std::string get_type() const override { return type; }
  inline const static std::string type =
      register_type<WayPointsTask>("WayPoints");
};

struct BoundedStateEstimation : public StateEstimation {
  BoundedStateEstimation(World *world_ = nullptr, float field_of_view_ = 0.0f,
                         float range_of_view_ = 0.0f)
      : StateEstimation(world_),
        field_of_view(field_of_view_),
        range_of_view(range_of_view_) {}

  virtual ~BoundedStateEstimation() = default;

  void update(Agent *agent) const override {
    // bool v1 = dynamic_cast<GeometricState *>(agent->nav_behavior.get()) !=
    // nullptr; std::cout << "SE::update: behavior is geometric? " <<
    // agent->nav_behavior << " " << v1 << std::endl; GeometricState *state =
    //     dynamic_cast<GeometricState *>(agent->nav_behavior.get());
    GeometricState *state = agent->get_geometric_state();
    if (state) {
      state->set_neighbors(neighbors(agent));
    } else {
      std::cerr << "Not a geometric state" << std::endl;
      // << typeid(*agent->nav_behavior.get()).name() << std::endl;
    }
  }

  void prepare(Agent *agent) const override {
    // if (GeometricState *state =
    //         dynamic_cast<GeometricState *>(agent->nav_behavior.get())) {
    if (GeometricState *state = agent->get_geometric_state()) {
      state->set_static_obstacles(world->get_obstacles());
      state->set_line_obstacles(world->get_walls());
    }
  }

  virtual std::vector<Neighbor> neighbors(const Agent *agent) const {
    std::vector<Neighbor> ns;

    auto const cs = world->get_neighbors(bounding_box(agent));
    for (const Agent *neighbor : cs) {
      if (neighbor != agent && visible(agent, neighbor)) {
        ns.push_back(perceive_neighbor(agent, neighbor));
      }
    }
    return ns;
  }

  virtual Neighbor perceive_neighbor([[maybe_unused]] const Agent *agent,
                                     const Agent *neighbor) const {
    return Neighbor(neighbor->pose.position, neighbor->radius,
                    neighbor->twist.velocity, neighbor->id);
  }

  virtual bool visible([[maybe_unused]] const Agent *agent,
                       [[maybe_unused]] const Agent *neighbor) const {
    return true;
  }

  BoundingBox bounding_box(const Agent *agent) const {
    return {agent->pose.position[0] - range_of_view,
            agent->pose.position[0] + range_of_view,
            agent->pose.position[1] - range_of_view,
            agent->pose.position[1] + range_of_view};
  }

  void set_range_of_view(float v) { range_of_view = v; }

  float get_range_of_view() const { return range_of_view; }

  void set_field_of_view(float v) { field_of_view = v; }

  float get_field_of_view() const { return field_of_view; }

  virtual const Properties &get_properties() const override {
    return properties;
  };

  static inline std::map<std::string, Property> properties =
      Properties{
          {"field_of_view", make_property<float, BoundedStateEstimation>(
                                &BoundedStateEstimation::get_field_of_view,
                                &BoundedStateEstimation::set_field_of_view,
                                0.0f, "Field of view (< 0 infinite)")},
          {"range_of_view", make_property<float, BoundedStateEstimation>(
                                &BoundedStateEstimation::get_range_of_view,
                                &BoundedStateEstimation::set_range_of_view,
                                0.0f, "Range of view (< 0 =infinite)")},
      } +
      StateEstimation::properties;

  std::string get_type() const override { return type; }
  inline const static std::string type =
      register_type<BoundedStateEstimation>("Bounded");

 private:
  float field_of_view;
  float range_of_view;
};

}  // namespace hl_navigation_sim

#endif /* end of include guard: HL_NAVIGATION_SIM_WORLD_H_ */
