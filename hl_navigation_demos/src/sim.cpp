/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <chrono>
#include <memory>
#include <vector>

#include "hl_navigation/kinematic.h"
#include "hl_navigation_sim/simulation.h"

using hl_navigation::TwoWheeled;
using hl_navigation::Vector2;
using hl_navigation_sim::BoundedStateEstimation;
using hl_navigation_sim::Experiment;
using hl_navigation_sim::WayPointsTask;
using hl_navigation_sim::World;

class ThymioDemo : public Experiment {
 public:
  explicit ThymioDemo(const char* behavior = "HL")
      : Experiment(0.02, 50 * 60), behavior(behavior) {}

 protected:
  void init(World& world, int seed) override {
    const std::vector<Vector2> targets{{1.0f, 0.0f}, {-1.0f, 0.0f}};
    for (size_t i = 0; i < 2; i++) {
      auto task = std::make_shared<WayPointsTask>(targets, true, 0.2);
      auto se = std::make_shared<BoundedStateEstimation>(&world, 1.0, 1.0);
      auto kinematic = std::make_shared<TwoWheeled>(0.166, 0.094);
      auto& agent = world.agents.emplace_back(behavior, 0.1, 0.08, kinematic,
                                              task, se, 0.02);
      agent.nav_behavior->set_optimal_speed(0.12);
      agent.nav_behavior->set_horizon(1.0);
      agent.nav_behavior->set_safety_margin(0.02);
      agent.nav_controller.set_speed_tolerance(0.01);
      agent.pose = {{i ? -0.5f : 0.5f, 0.5f}, 0.0f};
    }
    world.obstacles.emplace_back(Vector2{0.0f, 0.0f}, 0.1f);
  }

 private:
  const char* behavior;
};

static void show_usage(std::string name) {
  std::cout << "Usage: " << name << " <option(s)>" << std::endl
            << "Options:" << std::endl
            << "  --help\t\t\tShow this help message" << std::endl
            << "  --behavior=<BEHAVIOR>\tNavigation behavior" << std::endl;
}

int main(int argc, char* argv[]) {
  char behavior_name[10] = "HL";
  for (int i = 0; i < argc; i++) {
    if (sscanf(argv[i], "--behavior=%10s", behavior_name)) {
      continue;
    } else if (sscanf(argv[i], "--help")) {
      show_usage(argv[0]);
      return 0;
    }
  }
  ThymioDemo demo(behavior_name);
  printf("Start simulating 1 minute at 50 ticks per second\n");
  auto begin = std::chrono::high_resolution_clock::now();
  demo.run(0);
  auto end = std::chrono::high_resolution_clock::now();
  unsigned us =
      std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();
  printf("Done simulating in %.1f ms\n", us * 1e-6);
  return 0;
}

