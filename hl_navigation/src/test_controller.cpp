/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "Agent.h"
#include "Controller.h"
#include <iostream>
#include <iterator>
#include <vector>

// Should also add the wheels

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

class MyController : public Controller {
  void updated() override {
    printf("Updated %d\n", state);
  }

  void arrived() override {
    printf("Arrived\n");
  }

  void aborted() override {
    printf("Aborted\n");
  }
};


int main(int argc, char *argv[]) {
  MyController controller;
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
  auto agent = Agent::agent_with_name(behavior_name, HOLONOMIC, 0.1);
  if (!agent) {
    printf("No behavior with name %s\n", behavior_name);
    exit(1);
  }
  const auto& r = *agent.get();
  printf("Use behavior %s - %s\n", behavior_name, typeid(r).name());
  float dt = 0.1;
  agent->set_max_speed(1.0);
  agent->set_optimal_speed(1.0);
  agent->set_horizon(1.0);
  agent->position = CVector2(0.0f, 0.0f);
  agent->velocity = CVector2(0.0f, 0.0f);
  // Go to 1, 0
  controller.agent = agent.get();
  controller.set_target_point(1.0, 0.0, 0.0);
  controller.distance_tolerance = 0.1;
  controller.speed_tolerance = 0.05;
  float t = 0.0;
  printf("Controller state %d\n", controller.state);
  printf("Start loop @ (%.3f, %.3f)\n", agent->position.GetX(), agent->position.GetY());
  while (controller.state != Controller::IDLE) {
    controller.update(dt);
    agent->velocity = agent->get_target_velocity();
    agent->angle += CRadians(agent->target_twist.angular) * dt;
    agent->position += agent->velocity * dt;
    t += dt;
  }
  printf("\nEnd loop after %.1f s @ (%.3f, %.3f), (%.3f %.3f)\n", t,
         agent->position.GetX(), agent->position.GetY(),
         agent->velocity.GetX(), agent->velocity.GetY());
  return 0;
}
