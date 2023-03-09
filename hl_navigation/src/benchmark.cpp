/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <chrono>
#include <iostream>
#include <memory>
#include <tuple>
#include <vector>

#include "hl_navigation/behavior.h"
#include "hl_navigation/state/geometric.h"

using namespace hl_navigation;

static void show_usage(const std::string &name) {
  std::vector<std::string> keys = Behavior::behavior_names();
  std::ostringstream behaviors;
  // Dump all keys
  std::copy(keys.begin(), keys.end(),
            std::ostream_iterator<std::string>(behaviors, ", "));
  std::cout
      << "Usage: " << name << " <option(s)>" << std::endl
      << "Options:" << std::endl
      << "  --help\t\t\tShow this help message" << std::endl
      << "  --behavior=<NAME>\t\tObstacle avoidance algorithm name: one of "
      << behaviors.str() << std::endl;
}

void run(const char *behavior = "HL", float radius = 4, unsigned number = 5,
         float margin = 1.0, float dt = 0.1) {
  std::vector<std::shared_ptr<Behavior>> agents;
  float x = radius - margin;
  std::vector<std::tuple<Vector2, std::function<Vector2(int)>>> task = {
      {Vector2(x, 0.0),
       [x, number](int i) {
         return Vector2(-x + 2 * x * i / (number - 1), 0.5);
       }},
      {Vector2(-x, 0.0),
       [x, number](int i) {
         return Vector2(-x + 2 * x * i / (number - 1), -0.5);
       }},
      {Vector2(0.0, x),
       [x, number](int i) {
         return Vector2(0.5, -x + 2 * x * i / (number - 1));
       }},
      {Vector2(0.0, -x), [x, number](int i) {
         return Vector2(-0.5, -x + 2 * x * i / (number - 1));
       }}};

  for (auto &[target, position] : task) {
    for (size_t i = 0; i < number; i++) {
      auto agent = Behavior::behavior_with_name(
          behavior, std::make_shared<Holonomic>(1.0, 1.0), 0.1);
      agent->set_horizon(1.0);
      agent->set_position(position(i));
      agent->set_target_position(target);
      agent->social_margin.set_modulation(SocialMargin::LinearModulation(1.0f));
      agent->social_margin.set(0, 0.25f);
      agents.push_back(agent);
    }
  }

  std::vector<Disc> neighbors;
  std::transform(agents.cbegin(), agents.cend(), std::back_inserter(neighbors),
                 [](auto &agent) {
                   return Disc(agent->get_position(), 0.1, 0.0,
                               agent->get_velocity(false));
                 });

  for (size_t i = 0; i < 1000; i++) {
    size_t j = 0;
    for (auto &agent : agents) {
      neighbors[j].position = agent->get_position();
      neighbors[j].velocity = agent->get_velocity(false);
      j++;
    }
    j = 0;
    for (auto &agent : agents) {
      auto g = agent->get_target_position();
      auto p = agent->get_position();
      if ((g[0] == 0 && p[1] / g[1] > 1) || (g[1] == 0 && p[0] / g[0] > 1)) {
        agent->set_target_position(-p);
      }
      if (GeometricState * state = dynamic_cast<GeometricState *>(agent.get())) {
        std::vector<Disc> agent_neighbors;
        std::copy(neighbors.begin(), neighbors.begin() + j,
                  std::back_inserter(agent_neighbors));
        if (j + 1 < neighbors.size()) {
          std::copy(neighbors.begin() + j + 1, neighbors.end(),
                    std::back_inserter(agent_neighbors));
        }
        state->set_neighbors(agent_neighbors);
      }
      const auto twist = agent->cmd_twist(dt);
      agent->actuate(twist, dt);
      j++;
    }
  }
  // for (auto & agent : agents) {
  //   std::cout << agent->position[0] << "," << agent->position[1] << ",";
  // }
}

int main(int argc, char *argv[]) {
  char behavior_name[10] = "HL";
  for (int i = 0; i < argc; i++) {
    if (sscanf(argv[i], "--behavior=%9s", behavior_name)) {
      continue;
    }
    if (strcmp(argv[i], "--help") == 0) {
      show_usage(argv[0]);
      return 0;
    }
  }
  auto begin = std::chrono::high_resolution_clock::now();
  run(behavior_name);
  auto end = std::chrono::high_resolution_clock::now();
  unsigned us =
      std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin)
          .count() /
      1000;
  printf("%s: %u\n", behavior_name, us);
  return 0;
}
