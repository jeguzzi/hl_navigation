/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "Agent.h"
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
  auto agent = Agent::agent_with_name(behavior_name, TWO_WHEELED, 0.1, 0.1);
  if (!agent) {
    printf("No behavior with name %s\n", behavior_name);
    exit(1);
  }
  const auto& r = *agent.get();
  printf("Use behavior %s - %s\n", behavior_name, typeid(r).name());
  float dt = 0.1;
  agent->set_max_speed(1.0);
  agent->set_max_angular_speed(1.0);
  agent->set_optimal_speed(1.0);
  agent->set_optimal_angular_speed(1.0);
  agent->set_horizon(1.0);

  agent->position = CVector2(0.0f, 0.05f);
  agent->velocity = CVector2(0.0f, 0.0f);
  // Go to 1, 0
  agent->targetPosition = CVector2(3.0f, 0.0f);
  agent->set_static_obstacles({Disc(CVector2(1.5f, 0.0f), 0.5f)});
  // This should be in init
  printf("Start loop @ (%.3f, %.3f)\n", agent->position.GetX(), agent->position.GetY());
  for (size_t i = 0; i < 30; i++) {
    // printf("P (%.3f, %.3f, %.3f)\n",
    //   agent->position.GetX(), agent->position.GetY(), agent->angle.GetValue());
    // TODO(Jerome): rivedere tutte le cache.
    // Per esempio, qui mi obbliga a ricreare gli ostacoli ogni volta

    printf("%.3f, %.3f,", agent->position.GetX(), agent->position.GetY());
    //       agent->velocity.GetX(), agent->velocity.GetY());
    // printf("%zu (%.3f, %.3f), (%.3f, %.3f)\n", i, agent->position.GetX(), agent->position.GetY(),
    //       agent->velocity.GetX(), agent->velocity.GetY());
    // agent->updateDesiredVelocity();
    // printf("V (%.3f, %.3f)\n", agent->desiredVelocity.GetX(), agent->desiredVelocity.GetY());
    agent->update(dt);
    // std::cout << agent->desiredVelocity << std::endl;
    // std::cout << agent->get_target_velocity() << std::endl;
    // printf("V1 (%.3f, %.3f, %.3f. %.3f)\n",
    //   agent->desiredLinearSpeed, agent->desiredVelocity.GetX(), agent->desiredVelocity.GetY(),
    //   agent->desiredAngularSpeed.GetValue());

    // agent->velocity = agent->desiredVelocity;

    agent->velocity = agent->get_target_velocity();
    agent->angle += CRadians(agent->target_twist.angular) * dt;
    agent->position += agent->velocity * dt;
  }
  printf("\nEnd loop @ (%.3f, %.3f), (%.3f %.3f)\n",
         agent->position.GetX(), agent->position.GetY(),
         agent->velocity.GetX(), agent->velocity.GetY());
  return 0;
}
