/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "Agent.h"
#include <chrono>
#include <iostream>
#include <memory>
#include <tuple>
#include <vector>

static void show_usage(std::string name) {
  std::vector<std::string> keys = Agent::behavior_names();
  std::ostringstream behaviors;
  // Dump all keys
  std::copy(keys.begin(), keys.end(), std::ostream_iterator<std::string>(behaviors, ", "));
  std::cout << "Usage: " << name << " <option(s)>" << std::endl
            << "Options:" << std::endl
            << "  --help\t\t\tShow this help message" << std::endl
            << "  --behavior=<NAME>\t\tObstacle avoidance algorithm name: one of "
            << behaviors.str()
            << std::endl;
}


void run(const char * behavior = "HL", float radius = 4, unsigned number = 5,
         float margin = 1.0, float dt = 0.1) {
  std::vector<std::unique_ptr<Agent>> agents;
  float x = radius - margin;
  std::vector<std::tuple<CVector2, std::function<CVector2(int)>>> task = {
      {CVector2(x, 0.0),
       [x, number](int i) {return CVector2(-x + 2 * x * i / (number  - 1), 0.5  );}},
      {CVector2(-x, 0.0),
       [x, number](int i) {return CVector2(-x + 2 * x * i / (number  - 1), -0.5 );}},
      {CVector2(0.0, x),
       [x, number](int i) {return CVector2(0.5, -x + 2 * x * i / (number  - 1)  );}},
      {CVector2(0.0, -x),
       [x, number](int i) {return CVector2(-0.5, -x + 2 * x * i / (number  - 1) );}}
  };

  for (auto & [target, position] : task) {
    for (size_t i = 0; i < number; i++) {
      auto agent = Agent::agent_with_name(behavior, HOLONOMIC, 0.1, 0.1);
      agent->set_max_speed(1.0);
      agent->set_max_angular_speed(1.0);
      agent->set_optimal_speed(1.0);
      agent->set_optimal_angular_speed(1.0);
      agent->set_horizon(1.0);
      agent->velocity = CVector2(0.0f, 0.0f);
      agent->position = position(i);
      agent->targetPosition = target;
      agents.push_back(std::move(agent));
    }
  }

  std::vector<Disc> neighbors;
  std::transform(agents.cbegin(), agents.cend(),
                 std::back_inserter(neighbors),
                 [](auto & agent) {
                    return Disc(agent->position, 0.1, 0.0, agent->velocity); });

  for (size_t i = 0; i < 1000; i++) {
    int j = 0;
    for (auto & agent : agents) {
      neighbors[j].position = agent->position;
      neighbors[j].velocity = agent->velocity;
      j++;
    }
    j = 0;
    for (auto & agent : agents) {
      if (agent->targetPosition[0] == 0 && agent->position[1] / agent->targetPosition[1] > 1) {
        agent->targetPosition *= -1;
      } else if (agent->targetPosition[1] == 0 &&
                 agent->position[0] / agent->targetPosition[0] > 1) {
        agent->targetPosition *= -1;
      }
      std::vector<Disc> agent_neighbors;
      std::copy(neighbors.begin(), neighbors.begin() + j, std::back_inserter(agent_neighbors));
      if (j + 1 < neighbors.size()) {
        std::copy(neighbors.begin() + j + 1, neighbors.end(), std::back_inserter(agent_neighbors));
      }
      agent->set_neighbors(agent_neighbors);
      // printf("%d\n", agent_neighbors.size());
      agent->update(dt);
      j++;
    }
    for (auto & agent : agents) {
      agent->velocity = agent->get_target_velocity();
      agent->angle += agent->target_twist.angular * dt;
      agent->position += agent->velocity * dt;
    }
  }
  // for (auto & agent : agents) {
  //   std::cout << agent->position[0] << "," << agent->position[1] << ",";
  // }
}


int main(int argc, char *argv[]) {
  char behavior_name[10] = "HL";
  for (int i = 0; i < argc; i++) {
    if (sscanf(argv[i], "--behavior=%10s", behavior_name)) {
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
  unsigned us = std::chrono::duration_cast<std::chrono::nanoseconds>(end-begin).count()/ 1000;
  printf("%u\n", us);
  return 0;
}
