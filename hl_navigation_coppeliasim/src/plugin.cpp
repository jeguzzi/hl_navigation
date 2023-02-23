#include "plugin.h"

#include <memory>
#include <vector>

#include "config.h"
#include "hl_navigation/behavior.h"
#include "hl_navigation/common.h"
#include "hl_navigation/controller_3d.h"
#include "hl_navigation/kinematic.h"
#include "simPlusPlus/Handle.h"
#include "simPlusPlus/Plugin.h"
#include "stubs.h"

using namespace hl_navigation;

static std::shared_ptr<hl_navigation::Kinematic> make_kinematic(
    const kinematic_t &k) {
  switch (k.type) {
    case sim_hlnavigation_holonomic:
      return std::make_shared<Holonomic>(k.max_speed, k.max_angular_speed);
    case sim_hlnavigation_frontal:
      return std::make_shared<Forward>(k.max_speed, k.max_angular_speed);
    case sim_hlnavigation_two_wheeled:
      return std::make_shared<TwoWheeled>(k.max_speed, k.axis_length);
    case sim_hlnavigation_four_wheeled:
      return std::make_shared<FourWheeled>(k.max_speed, k.axis_length);
    default:
      return nullptr;
  }
}

class Plugin : public sim::Plugin {
 public:
  Plugin() : sim::Plugin(), controllers() {}

  void onStart() {
    if (!registerScriptStuff())
      throw std::runtime_error("script stuff initialization failed");
    setExtVersion("HLNavigation");
    setBuildDate(BUILD_DATE);
  }

  void onSimulationAboutToEnd() { controllers.clear(); }

  Controller3 *controller_at_index(unsigned i) const {
    if (i < controllers.size()) {
      return controllers[i].get();
    }
    return nullptr;
  }

  Behavior *behavior_at_index(unsigned i) const {
    if (i < controllers.size()) {
      return controllers[i]->get_behavior().get();
    }
    return nullptr;
  }

  void Controller(Controller_in *in, Controller_out *out) {
    int handle = controllers.size();
    auto kinematic = make_kinematic(in->kinematic);
    if (!kinematic) {
      out->handle = -1;
      return;
    }
    out->handle = handle;  // TODO(J): add -1 to mark failures
    auto behavior =
        Behavior::behavior_with_name(in->behavior, kinematic, in->radius);
    auto controller = std::make_unique<Controller3>(behavior);
    controllers.push_back(std::move(controller));
  }

  void go_to_position(go_to_position_in *in, go_to_position_out *out) {
    auto controller = controller_at_index(in->handle);
    if (controller) {
      controller->go_to_position(
          Vector3{in->position[0], in->position[1], in->position[2]},
          in->tolerance);
    }
  }

  void go_to_pose(go_to_pose_in *in, go_to_pose_out *out) {
    auto controller = controller_at_index(in->handle);
    if (controller) {
      controller->go_to_pose(
          Pose3{{in->position[0], in->position[1], in->position[2]},
                in->orientation},
          in->position_tolerance, in->orientation_tolerance);
    }
  }

  void follow_point(follow_point_in *in, follow_point_out *out) {
    auto controller = controller_at_index(in->handle);
    if (controller) {
      controller->follow_point(
          Vector3{in->point[0], in->point[1], in->point[2]});
    }
  }

  void follow_pose(follow_pose_in *in, follow_pose_out *out) {
    auto controller = controller_at_index(in->handle);
    if (controller) {
      controller->follow_pose(
          Pose3{{in->position[0], in->position[1], in->position[2]},
                in->orientation});
    }
  }

  void set_pose(set_pose_in *in, set_pose_out *out) {
    auto controller = controller_at_index(in->handle);
    if (controller) {
      controller->set_pose(
          Pose3{{in->position[0], in->position[1], in->position[2]},
                in->orientation});
    }
  }

  void set_twist(set_twist_in *in, set_twist_out *out) {
    auto controller = controller_at_index(in->handle);
    if (controller) {
      controller->set_twist(
          Twist3{{in->velocity[0], in->velocity[1], in->velocity[2]},
                 in->angular_speed});
    }
  }

  void update(update_in *in, update_out *out) {
    auto controller = controller_at_index(in->handle);
    if (controller) {
      auto twist = controller->update(in->time_step);
      out->velocity = {twist.velocity[0], twist.velocity[1], twist.velocity[2]};
      out->angular_speed = twist.angular_speed;
      out->state = static_cast<int>(controller->get_state());
    }
  }

  // TODO(J): extend to 3d
  void set_static_obstacles(set_static_obstacles_in *in,
                            set_static_obstacles_out *out) {
    auto behavior = behavior_at_index(in->handle);
    if (behavior) {
      std::vector<Disc> obstacles;
      std::transform(in->obstacles.cbegin(), in->obstacles.cend(),
                     std::back_inserter(obstacles), [](const obstacle_t &o) {
                       return Disc(Vector2(o.position[0], o.position[1]),
                                   o.radius, o.social_margin);
                     });
      behavior->set_static_obstacles(obstacles);
    }
  }

  void set_neighbors(set_neighbors_in *in, set_neighbors_out *out) {
    auto behavior = behavior_at_index(in->handle);
    if (behavior) {
      std::vector<Disc> obstacles;
      std::transform(in->obstacles.cbegin(), in->obstacles.cend(),
                     std::back_inserter(obstacles), [](obstacle_t o) {
                       return Disc(Vector2(o.position[0], o.position[1]),
                                   o.radius, o.social_margin,
                                   Vector2(o.velocity[0], o.velocity[1]));
                     });
      behavior->set_neighbors(obstacles);
    }
  }

  void set_line_obstacles(set_line_obstacles_in *in,
                          set_line_obstacles_out *out) {
    auto behavior = behavior_at_index(in->handle);
    if (behavior) {
      std::vector<LineSegment> obstacles;
      std::transform(in->obstacles.cbegin(), in->obstacles.cend(),
                     std::back_inserter(obstacles), [](line_t o) {
                       return LineSegment(Vector2(o.p1[0], o.p1[1]),
                                          Vector2(o.p2[0], o.p2[1]));
                     });
      behavior->set_line_obstacles(obstacles);
    }
  }

  void get_state(get_state_in *in, get_state_out *out) {
    auto controller = controller_at_index(in->handle);
    if (controller) {
      out->state = static_cast<int>(controller->get_state());
    }
  }

  void set_rotation_tau(set_rotation_tau_in *in, set_rotation_tau_out *out) {
    auto behavior = behavior_at_index(in->handle);
    if (behavior) {
      behavior->set_rotation_tau(in->value);
    }
  }

  void set_horizon(set_horizon_in *in, set_horizon_out *out) {
    auto behavior = behavior_at_index(in->handle);
    if (behavior) {
      behavior->set_rotation_tau(in->value);
    }
  }

  void set_safety_margin(set_safety_margin_in *in, set_safety_margin_out *out) {
    auto behavior = behavior_at_index(in->handle);
    if (behavior) {
      behavior->set_safety_margin(in->value);
    }
  }

  void set_optimal_speed(set_optimal_speed_in *in, set_optimal_speed_out *out) {
    auto behavior = behavior_at_index(in->handle);
    if (behavior) {
      behavior->set_optimal_speed(in->value);
    }
  }

  void set_heading_behavior(set_heading_behavior_in *in,
                            set_heading_behavior_out *out) {
    auto behavior = behavior_at_index(in->handle);
    if (behavior) {
      behavior->set_heading_behavior(static_cast<Behavior::Heading>(in->value));
    }
  }

  void get_actuated_wheel_speeds(get_actuated_wheel_speeds_in *in,
                                 get_actuated_wheel_speeds_out *out) {
    auto behavior = behavior_at_index(in->handle);
    if (behavior) {
      out->speeds = behavior->get_actuated_wheel_speeds();
    }
  }

 private:
  std::vector<std::unique_ptr<Controller3>> controllers;
};

SIM_PLUGIN(PLUGIN_NAME, PLUGIN_VERSION, Plugin)
#include "stubsPlusPlus.cpp"
