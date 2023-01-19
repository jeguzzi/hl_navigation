#include "plugin.h"

#include "simPlusPlus/Plugin.h"
#include "simPlusPlus/Handle.h"
#include "stubs.h"
#include "config.h"

#include "hl_navigation/Controller.h"
#include "hl_navigation/HLAgent.h"

class Plugin : public sim::Plugin {
 public:
    void onStart() {
        if (!registerScriptStuff())
            throw std::runtime_error("script stuff initialization failed");
        setExtVersion("HLNavigation");
        setBuildDate(BUILD_DATE);
    }

    void onSimulationAboutToEnd() {
      controllers.clear();
      agents.clear();
    }

    void Human(Human_in *in, Human_out *out) {
      int handle =  controllers.size();
      out->handle = handle;  // TODO(J): add -1 to mark failures
      auto & controller = controllers.emplace_back();
      auto & agent = agents.emplace_back(HEAD, 0.5);
      controller.agent = &agent;
      agent.set_max_speed(1.0);
      agent.set_optimal_speed(1.0);
      agent.set_horizon(5.0);
      controller.distance_tolerance = 0.5;
      controller.speed_tolerance = 0.1;
  }

  void set_target_position(set_target_position_in *in, set_target_position_out *out) {
    if (in->handle < controllers.size()) {
      controllers[in->handle].set_target_point(in->position[0], in->position[1], in->position[2]);
    }
  }

  void set_pose(set_pose_in *in, set_pose_out *out) {
    if (in->handle < controllers.size()) {
      controllers[in->handle].set_pose(in->pose[0], in->pose[1], in->pose[2], in->pose[3]);
    }
  }

  void set_velocity(set_velocity_in *in, set_velocity_out *out) {
    if (in->handle < controllers.size()) {
      agents[in->handle].velocity = CVector2(in->velocity[0], in->velocity[1]);
    }
  }

  void update(update_in *in, update_out *out) {
    if (in->handle < controllers.size()) {
      auto & agent = agents[in->handle];
      printf("(%.3f, %.3f), (%.3f %.3f), (%.3f, %.3f)\n",
             agent.position.x(), agent.position.y(),
             agent.velocity.x(), agent.velocity.y(),
             agent.targetPosition.x(), agent.targetPosition.y());
      // TODO(J): maybe use agent instead
      controllers[in->handle].update(in->time_step);
      printf("-> (%.3f, %.3f)\n", agent.target_twist.longitudinal, agent.target_twist.angular);
    }
  }

  void get_target_twist(get_target_twist_in *in, get_target_twist_out *out) {
    if (in->handle < controllers.size()) {
      auto & twist = agents[in->handle].target_twist;
      out->twist = {twist.longitudinal, twist.lateral, twist.angular};
    }
  }

  void set_static_obstacles(set_static_obstacles_in *in, set_static_obstacles_out *out) {
    if (in->handle < controllers.size()) {
      auto & agent = agents[in->handle];
      std::vector<Disc> obstacles;
      std::transform(
          in->obstacles.cbegin(), in->obstacles.cend(), std::back_inserter(obstacles),
          [](obstacle_t o) {
             return Disc(CVector2(o.position[0], o.position[1]), o.radius, o.social_margin);
          });
      agent.set_static_obstacles(obstacles);
    }
  }

  void set_neighbors(set_neighbors_in *in, set_neighbors_out *out) {
    if (in->handle < controllers.size()) {
      auto & agent = agents[in->handle];
      std::vector<Disc> obstacles;
      std::transform(
          in->obstacles.cbegin(), in->obstacles.cend(), std::back_inserter(obstacles),
          [](obstacle_t o) {
             return Disc(CVector2(o.position[0], o.position[1]), o.radius, o.social_margin,
                         CVector2(o.velocity[0], o.velocity[1]));
          });
      agent.set_neighbors(obstacles);
    }
  }

 private:
    std::vector<Controller> controllers;
    // TODO(J): generalize to other behaviors
    std::vector<HLAgent> agents;
};

SIM_PLUGIN(PLUGIN_NAME, PLUGIN_VERSION, Plugin)
#include "stubsPlusPlus.cpp"
