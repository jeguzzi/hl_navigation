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


#include <chrono>
#include <iostream>
#include <memory>
#include <tuple>
#include <vector>

#include <highfive/H5File.hpp>
#include <geos/geom/Envelope.h>
#include <geos/index/strtree/TemplateSTRtree.h>

#include "hl_navigation/Agent.h"
#include "hl_navigation/Controller.h"


using BoundingBox = geos::geom::Envelope;

class _Agent;
class World;

struct Task {
  explicit Task() { }
  virtual void update(_Agent * agent) { }
  virtual bool done() const { return false; }

};

class StateEstimation {
 public:
  StateEstimation(const World * world): world(world) { }

    virtual std::vector<Disc> neighbors(const _Agent * agent) const {
        return {};
    }

 protected:
  const World * world;
};


class _Agent {
 public:
  _Agent(const char * behavior_name, float mass, float radius,
         std::unique_ptr<Task> && task, std::unique_ptr<StateEstimation> && estimation, float control_period) :
         control_period(control_period), control_deadline(0.0),
         mass(mass), radius(radius),
         task(std::move(task)), state_estimation(std::move(estimation)), nav_controller(),
         nav_behavior(Agent::agent_with_name(behavior_name, HOLONOMIC, radius, 0.0)) {
           nav_controller.agent = nav_behavior.get();
//           printf("Use behavior %s - %s\n", behavior_name, typeid(*nav_behavior.get()).name());
         }

  void update(float dt) {
    control_deadline -= dt;
    if (control_deadline > 0) {
      return;
    }
    control_deadline += control_period;
    task->update(this);
    nav_controller.agent->set_neighbors(state_estimation->neighbors(this));
    nav_controller.agent->velocity = velocity;
    nav_controller.agent->position = position;
    nav_controller.update(dt);
    target_velocity = nav_controller.agent->get_target_velocity();
  }

  Controller nav_controller;
  std::unique_ptr<Agent> nav_behavior;
  float radius;

  void update_physics(float dt) {
    velocity = target_velocity;  // + collision_force / mass * dt;
    position += velocity * dt;
  }

  float mass;
  float control_period;
  float control_deadline;
  Eigen::Vector2f position;
  Eigen::Vector2f velocity;
  Eigen::Vector2f target_velocity;
//  Eigen::Vector2f collision_force;
  Eigen::Vector2f collision_correction;
  std::unique_ptr<Task> task;
  std::unique_ptr<StateEstimation> state_estimation;
};


class World {
 public:
  explicit World(float time_step) : time_step(time_step), agents(), obstacles(), walls(),
                 agent_index(nullptr) {}

  void update() {
    for (auto & a : agents) {
      a.update(time_step);
    }
    for (auto & a : agents) {
      a.update_physics(time_step);
    }
    update_agents_strtree();
    update_collisions();
  }

  void run(unsigned steps) {
    for (size_t i = 0; i < steps; i++) {
      update();
    }
  }

  std::vector<_Agent *> get_neighbors(const BoundingBox & bb) const {
    std::vector<_Agent *> rs;
    // std::transform(agents.cbegin(), agents.cend(), std::back_inserter(rs), [](const auto & a) { return &a; });
    agent_index->query(bb, rs);
    return rs;
  }

  std::vector<Disc *> get_obstacles(const BoundingBox & bb) const {
    return {};
  }

  std::vector<LineSegment *> get_walls(const BoundingBox & bb) const {
    return {};
  }

  float time_step;
  std::vector<_Agent> agents;
  std::vector<Disc> obstacles;
  std::vector<LineSegment> walls;

 protected:

  void resolve_collision(_Agent* a1, _Agent* a2) {
    auto delta = a1->position - a2->position;
    float penetration = delta.norm() - a1->radius - a2->radius;
    if (penetration > 0) {
      return;
    }
    auto u = delta/delta.norm();
    auto correction = (-penetration * 0.5 + 1e-3) * u;
    a1->collision_correction += correction;
    a2->collision_correction -= correction;
    float d = a1->velocity.dot(-u);
    if (d > 0) {
      a1->velocity += d * u;
    }
    d = a2->velocity.dot(u);
    if (d > 0) {
      a2->velocity -= d * u;
    }
    // auto force = -penetration * 100 * delta / delta.norm();
    // std::cout << force << std::endl;
    // a1->collision_force += force;
    // a2->collision_force -= force;
  }


  void update_collisions() {
    for (size_t i = 0; i < agents.size(); i++) {
        _Agent * a1 = &agents[i];
      agent_index->query(agent_envelops[i], [this, a1](_Agent * a2) {
        if (a1 < a2) {
          resolve_collision(a1, a2);
        }
      });
    }
      for (auto & a : agents) {
          a.position += a.collision_correction;
          a.collision_correction = Eigen::Vector2f(0.0, 0.0);
      }
    //  TODO(Jerome): queryPairs not yet released ... but does more or less the above
    //  agent_index->queryPairs([this](_Agent * a1, _Agent * a2) { resolve_collision(a1, a2); });
  }

  void update_agents_strtree() {
    agent_envelops.clear();
    agent_index = std::make_unique<geos::index::strtree::TemplateSTRtree<_Agent *>>(agents.size());
    for (const auto & agent : agents) {
      auto & bb = agent_envelops.emplace_back(
          agent.position[0] - agent.radius, agent.position[0] + agent.radius,
          agent.position[1] - agent.radius, agent.position[1] + agent.radius);
      agent_index->insert(&bb, (void *) &agent);
    }
  }

  std::unique_ptr<geos::index::strtree::TemplateSTRtree<_Agent *>> agent_index;
  std::vector<geos::geom::Envelope> agent_envelops;

};

// TODO(Jerome) replace Disk::social margin with Disk::type (unsigned)


class Experiment {
 public:
  Experiment(float dt = 0.1, int steps = 1000) : dt(dt), steps(steps) {}

  void run(int seed) {
    HighFive::File file("test.h5", HighFive::File::Truncate);
    World world(dt);
    init(world, seed);
    unsigned number = world.agents.size();
    std::vector<size_t> dims{(size_t)steps, number, 3};
    HighFive::DataSet dataset = file.createDataSet<float>(
        "traces", HighFive::DataSpace(dims));
    std::vector<float> data(number * 3, 0.0);
    for (size_t i = 0; i < steps; i++) {
      world.update();
      float * d = (float *)data.data();
      for (const auto & agent : world.agents) {
        *d++ = agent.position[0];
        *d++ = agent.position[1];
        *d++ = 0.0;
      }
      dataset.select({i, 0, 0}, {1, number, 3}).write_raw(data.data());
    }
      printf("Done\n");
  }

 protected:
  virtual void init(World & world, int seed) { }

  float dt;
  int steps;
};


struct WayPointsTask : Task {
  WayPointsTask(std::vector<Eigen::Vector2f> _waypoints, bool loop) :
    Task(),
    waypoints(_waypoints), waypoint(waypoints.begin()), loop(loop) {}

  void update(_Agent * agent) override {
    if (agent->nav_controller.state == Controller::IDLE) {
      if (waypoint != waypoints.end()) {
        agent->nav_controller.set_target_point(*waypoint);
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
};


class BoundedStateEstimation : public StateEstimation{
 public:
    BoundedStateEstimation(const World * world, float field_of_view, float range_of_view):
        StateEstimation(world), field_of_view(field_of_view), range_of_view(range_of_view) { }

  virtual std::vector<Disc> neighbors(const _Agent * agent) const override {
    std::vector<Disc> ns;
    for (const _Agent * neighbor : world->get_neighbors(bounding_box(agent))) {
      if (neighbor != agent && visible(agent, neighbor)) {
        ns.push_back(perceive_neighbor(agent, neighbor));
      }
    }
    return ns;
  }

  virtual Disc perceive_neighbor(const _Agent * agent, const _Agent * neighbor) const {
    return Disc(neighbor->position, neighbor->radius, 0.0, neighbor->velocity);
  }

  virtual bool visible(const _Agent * agent, const _Agent * neighbor) const { return true; }

   BoundingBox bounding_box(const _Agent * agent) const {
        return {agent->position[0] - range_of_view, agent->position[0] + range_of_view, agent->position[1] - range_of_view, agent->position[1] + range_of_view};
   }

  float field_of_view;
  float range_of_view;
};
