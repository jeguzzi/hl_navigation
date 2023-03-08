/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 *  (Run)

  Experiment
   - World
     - [Wall]
     - [Agent]
       - kinematics
       - Sensor
         - update (<- World)
       - Controller
         - NavigationController Behavior
         - update
       - update_physics
     - update
   - run
   - save
 */

#ifndef HL_NAVIGATION_SIM_SIMULATION_H_
#define HL_NAVIGATION_SIM_SIMULATION_H_

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
#include "hl_navigation/state/geometric.h"
#include "hl_navigation/controller.h"

namespace hl_navigation_sim {

using BoundingBox = geos::geom::Envelope;
using namespace hl_navigation;

class Agent;
class World;

struct Task {
  explicit Task() {}
  virtual void update(Agent *agent) {}
  virtual bool done() const { return false; }
};

class StateEstimation {
 public:
  StateEstimation(const World *world) : world(world) {}

  virtual void update(Agent * agent) const { };

  virtual void prepare(Agent * agent) const { };

 protected:
  const World *world;
};

class Agent {
 public:
  Agent(const char *behavior_name, float mass, float radius,
        std::shared_ptr<Kinematic> kinematic, std::shared_ptr<Task> task,
        std::shared_ptr<StateEstimation> estimation, float control_period)
      : control_period(control_period),
        control_deadline(0.0),
        mass(mass),
        radius(radius),
        task(task),
        state_estimation(estimation),
        nav_behavior(
            Behavior::behavior_with_name(behavior_name, kinematic, radius)),
        nav_controller(nav_behavior, false, true) {}

  void update(float dt) {
    control_deadline -= dt;
    if (control_deadline > 0) {
      return;
    }
    control_deadline += control_period;
    task->update(this);
    state_estimation->update(this);
    nav_behavior->set_actuated_twist(cmd_twist);
    nav_behavior->set_twist(twist);
    nav_behavior->set_pose(pose);
    cmd_twist = nav_controller.update(dt);
    // cmd_twist = nav_behavior->get_actuated_twist(true);
  }

  std::shared_ptr<Behavior> nav_behavior;
  Controller nav_controller;
  float radius;

  void update_physics(float dt) {
    twist = cmd_twist;  // + collision_force / mass * dt;
    pose = pose.integrate(twist, dt);
  }

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

class World {
 public:
  explicit World(float time_step)
      : time_step(time_step),
        agents(),
        obstacles(),
        walls(),
        agent_index(nullptr) {}

  void update() {
    for (auto &a : agents) {
      a.update(time_step);
    }
    for (auto &a : agents) {
      a.update_physics(time_step);
    }
    update_agents_strtree();
    update_collisions();
  }

  void prepare() {
    update_static_strtree();
    update_agents_strtree();

    for (auto &a : agents) {
      a.state_estimation->prepare(&a);
    }
  }

  void run(unsigned steps) {
    prepare();
    for (size_t i = 0; i < steps; i++) {
      update();
    }
  }

  std::vector<Agent *> get_neighbors(const BoundingBox &bb) const {
    std::vector<Agent *> rs;
    // std::transform(agents.cbegin(), agents.cend(), std::back_inserter(rs),
    // [](const auto & a) { return &a; });
    agent_index->query(bb, rs);
    return rs;
  }

  // TODO(J): complete
  std::vector<Disc *> get_obstacles(const BoundingBox &bb) const { return {}; }

  // TODO(J): complete
  std::vector<LineSegment *> get_walls(const BoundingBox &bb) const {
    return {};
  }

  float time_step;
  std::vector<Agent> agents;
  std::vector<Disc> obstacles;
  std::vector<LineSegment> walls;

 protected:
  void resolve_collision(Agent *a1, Agent *a2) {
    auto delta = a1->pose.position - a2->pose.position;
    float penetration = delta.norm() - a1->radius - a2->radius;
    if (penetration > 0) {
      return;
    }
    auto u = delta / delta.norm();
    auto correction = (-penetration * 0.5 + 1e-3) * u;
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
    // auto force = -penetration * k * delta / delta.norm();
    // std::cout << force << std::endl;
    // a1->collision_force += force;
    // a2->collision_force -= force;
  }

  // TODO(J): avoid repetitions
  void resolve_collision(Agent *agent, Disc *disc) {
    auto delta = agent->pose.position - disc->position;
    float penetration = delta.norm() - agent->radius - disc->radius;
    if (penetration > 0) {
      return;
    }
    auto u = delta / delta.norm();
    auto correction = (-penetration * 1.0 + 1e-3) * u;
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
      Agent *a1 = &agents[i];
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
      a.pose.position += a.collision_correction;
      a.collision_correction = Vector2::Zero();
    }
    // TODO(Jerome): queryPairs not yet released ... but does more or less the
    // above agent_index->queryPairs([this](Agent * a1, Agent * a2) {
    // resolve_collision(a1, a2); });
  }

  void update_agents_strtree() {
    agent_envelops.clear();
    agent_index =
        std::make_unique<geos::index::strtree::TemplateSTRtree<Agent *>>(
            agents.size());
    for (const auto &agent : agents) {
      auto &bb =
          agent_envelops.emplace_back(agent.pose.position[0] - agent.radius,
                                      agent.pose.position[0] + agent.radius,
                                      agent.pose.position[1] - agent.radius,
                                      agent.pose.position[1] + agent.radius);
      agent_index->insert(&bb, (void *)(&agent));
    }
  }

  void update_static_strtree() {
    static_envelops.clear();
    obstacles_index =
        std::make_unique<geos::index::strtree::TemplateSTRtree<Disc *>>(
            obstacles.size());
    walls_index =
        std::make_unique<geos::index::strtree::TemplateSTRtree<LineSegment *>>(
            walls.size());
    // TODO(J): should coordinates be ordered?
    for (const auto &wall : walls) {
      auto &bb = static_envelops.emplace_back(wall.p1[0], wall.p2[0],
                                              wall.p1[1], wall.p2[1]);
      walls_index->insert(&bb, (void *)(&wall));
    }
    for (const auto &obstacle : obstacles) {
      auto &bb =
          static_envelops.emplace_back(obstacle.position[0] - obstacle.radius,
                                       obstacle.position[0] + obstacle.radius,
                                       obstacle.position[1] - obstacle.radius,
                                       obstacle.position[1] + obstacle.radius);
      obstacles_index->insert(&bb, (void *)(&obstacle));
    }
  }

  std::unique_ptr<geos::index::strtree::TemplateSTRtree<Agent *>> agent_index;
  std::unique_ptr<geos::index::strtree::TemplateSTRtree<Disc *>>
      obstacles_index;
  std::unique_ptr<geos::index::strtree::TemplateSTRtree<LineSegment *>>
      walls_index;
  std::vector<geos::geom::Envelope> agent_envelops;
  std::vector<geos::geom::Envelope> static_envelops;
};

// TODO(Jerome) replace Disk::social margin with Disk::type (unsigned)

class Experiment {
 public:
  Experiment(float dt = 0.1, int steps = 1000) : dt(dt), steps(steps) {}

  void run(int seed);

 protected:
  virtual void init(World &world, int seed) {}

  float dt;
  int steps;
};

struct WayPointsTask : Task {
  WayPointsTask(std::vector<Eigen::Vector2f> _waypoints, bool loop,
                float tolerance)
      : Task(),
        waypoints(_waypoints),
        waypoint(waypoints.begin()),
        loop(loop),
        tolerance(tolerance) {}

  void update(Agent *agent) override {
    if (agent->nav_controller.idle()) {
      if (waypoint != waypoints.end()) {
        agent->nav_controller.go_to_position(*waypoint, tolerance);
        waypoint++;
        if (loop && waypoint == waypoints.end()) {
          waypoint = waypoints.begin();
        }
      }
    }
  }

  bool done() const override { return waypoint == waypoints.end(); }

  std::vector<Eigen::Vector2f> waypoints;
  std::vector<Eigen::Vector2f>::iterator waypoint;
  bool loop;
  float tolerance;
};

class BoundedStateEstimation : public StateEstimation {
 public:
  BoundedStateEstimation(const World *world, float field_of_view,
                         float range_of_view)
      : StateEstimation(world),
        field_of_view(field_of_view),
        range_of_view(range_of_view) {}

  void update(Agent *agent) const override {
    if(GeometricState * state = dynamic_cast<GeometricState *>(agent->nav_behavior.get())) {
      state->set_neighbors(neighbors(agent));
    }
  }

  void prepare(Agent * agent) const override {
    if(GeometricState * state = dynamic_cast<GeometricState *>(agent->nav_behavior.get())) {
      state->set_static_obstacles(world->obstacles);
      state->set_line_obstacles(world->walls);
    }
  }

  virtual std::vector<Disc> neighbors(const Agent *agent) const {
    std::vector<Disc> ns;
    for (const Agent *neighbor : world->get_neighbors(bounding_box(agent))) {
      if (neighbor != agent && visible(agent, neighbor)) {
        ns.push_back(perceive_neighbor(agent, neighbor));
      }
    }
    return ns;
  }

  virtual Disc perceive_neighbor(const Agent *agent,
                                 const Agent *neighbor) const {
    return Disc(neighbor->pose.position, neighbor->radius, 0.0,
                neighbor->twist.velocity);
  }

  virtual bool visible(const Agent *agent, const Agent *neighbor) const {
    return true;
  }

  BoundingBox bounding_box(const Agent *agent) const {
    return {agent->pose.position[0] - range_of_view,
            agent->pose.position[0] + range_of_view,
            agent->pose.position[1] - range_of_view,
            agent->pose.position[1] + range_of_view};
  }

  float field_of_view;
  float range_of_view;
};

}  // namespace hl_navigation_sim

#endif /* end of include guard: HL_NAVIGATION_SIM_SIMULATION_H_ */
