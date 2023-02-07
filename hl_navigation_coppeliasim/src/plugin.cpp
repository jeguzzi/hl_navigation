#include "plugin.h"

#include <memory>
#include <vector>

#include "simPlusPlus/Plugin.h"
#include "simPlusPlus/Handle.h"
#include "stubs.h"
#include "config.h"

#include "hl_navigation/behavior.h"
#include "hl_navigation/controller.h"

using namespace hl_navigation;

class Plugin : public sim::Plugin {
 public:
    Plugin() : sim::Plugin(), controllers(), behaviors() {}

    void onStart() {
        if (!registerScriptStuff())
            throw std::runtime_error("script stuff initialization failed");
        setExtVersion("HLNavigation");
        setBuildDate(BUILD_DATE);
    }

    void onSimulationAboutToEnd() {
      controllers.clear();
      behaviors.clear();
    }

    void controller(controller_in *in, controller_out *out) {
      int handle =  controllers.size();
      out->handle = handle;  // TODO(J): add -1 to mark failures
      auto controller = std::make_unique<Controller>();
      auto behavior = Behavior::behavior_with_name(
          in->behavior, agent_type_t(in->kinematics), in->radius, in->axis_length);
      controller->behavior = behavior.get();
      behavior->set_max_speed(1.0);
      behavior->set_safety_margin(0.0);
      behavior->set_optimal_speed(1.0);
      behavior->set_horizon(5.0);
      controller->distance_tolerance = 1.0;
      controller->speed_tolerance = 0.2;
      behaviors.push_back(std::move(behavior));
      controllers.push_back(std::move(controller));
  }

  void set_target_position(set_target_position_in *in, set_target_position_out *out) {
    if (in->handle < controllers.size()) {
      controllers[in->handle]->set_target_point(
          Vector3{in->position[0], in->position[1], in->position[2]});
    }
  }

  void set_pose(set_pose_in *in, set_pose_out *out) {
    if (in->handle < controllers.size()) {
      controllers[in->handle]->set_pose(
        Vector3{in->pose[0], in->pose[1], in->pose[2]}, in->pose[3]);
    }
  }

  void set_velocity(set_velocity_in *in, set_velocity_out *out) {
    if (in->handle < controllers.size()) {
      behaviors[in->handle]->velocity = Vector2(in->velocity[0], in->velocity[1]);
    }
  }

  void update(update_in *in, update_out *out) {
    if (in->handle < controllers.size()) {
      controllers[in->handle]->update(in->time_step);
    }
  }

  void get_target_twist(get_target_twist_in *in, get_target_twist_out *out) {
    if (in->handle < controllers.size()) {
      auto & twist = behaviors[in->handle]->target_twist;
      out->twist = {twist.longitudinal, twist.lateral, twist.angular};
    }
  }

  void set_static_obstacles(set_static_obstacles_in *in, set_static_obstacles_out *out) {
    if (in->handle < controllers.size()) {
      std::vector<Disc> obstacles;
      std::transform(
          in->obstacles.cbegin(), in->obstacles.cend(), std::back_inserter(obstacles),
          [](obstacle_t o) {
             return Disc(Vector2(o.position[0], o.position[1]), o.radius, o.social_margin);
          });
      behaviors[in->handle]->set_static_obstacles(obstacles);
    }
  }

  void set_neighbors(set_neighbors_in *in, set_neighbors_out *out) {
    if (in->handle < controllers.size()) {
      std::vector<Disc> obstacles;
      std::transform(
          in->obstacles.cbegin(), in->obstacles.cend(), std::back_inserter(obstacles),
          [](obstacle_t o) {
             return Disc(Vector2(o.position[0], o.position[1]), o.radius, o.social_margin,
                         Vector2(o.velocity[0], o.velocity[1]));
          });
      behaviors[in->handle]->set_neighbors(obstacles);
    }
  }

  void get_status(get_status_in *in, get_status_out *out) {
    if (in->handle < controllers.size()) {
      out->status = controllers[in->handle]->state;
    }
  }

 private:
    std::vector<std::unique_ptr<Controller>> controllers;
    std::vector<std::unique_ptr<Behavior>> behaviors;
};

SIM_PLUGIN(PLUGIN_NAME, PLUGIN_VERSION, Plugin)
#include "stubsPlusPlus.cpp"
