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
  auto agent = Agent::agent_with_name(behavior_name);
  if (!agent) {
    printf("No behavior with name %s\n", behavior_name);
    exit(1);
  }
  const auto& r = *agent.get();
  printf("Use behavior %s - %s\n", behavior_name, typeid(r).name());
  float dt = 0.1;
  agent->type = TWO_WHEELED;
  agent->radius = 0.1;
  agent->axisLength = 0.1;
  agent->setMaxSpeed(1.0);
  agent->setMaxRotationSpeed(1.0);
  // agent->setMaxAngularSpeed(1.0);
  agent->setOptimalSpeed(1.0);
  agent->setOptimalRotationSpeed(1.0);
  agent->setHorizon(1.0);
  // Set max speed ...
  // Start in 0, 0
  agent->position = CVector2(0.0f, 0.05f);
  agent->velocity = CVector2(0.0f, 0.0f);
  // Go to 1, 0
  agent->targetPosition = CVector2(3.0f, 0.0f);
  // This should be in init
  agent->clearObstacles();
  agent->addObstacleAtPoint(CVector2(0.5f, 0.0f), 0.2f, 0.0);
  printf("Start loop @ (%.3f, %.3f)\n", agent->position.GetX(), agent->position.GetY());
  for (size_t i = 0; i < 30; i++) {
    // printf("P (%.3f, %.3f, %.3f)\n",
    //   agent->position.GetX(), agent->position.GetY(), agent->angle.GetValue());
    // TODO(Jerome): rivedere tutte le cache.
    // Per esempio, qui mi obbliga a ricreare gli ostacoli ogni volta
    agent->clearObstacles();
    agent->addObstacleAtPoint(CVector2(0.5f, 0.0f), CVector2(-0.1f, 0.0f), 0.1f, 0.0);
    printf("%.3f, %.3f,", agent->position.GetX(), agent->position.GetY());
    //       agent->velocity.GetX(), agent->velocity.GetY());
    // printf("%zu (%.3f, %.3f), (%.3f, %.3f)\n", i, agent->position.GetX(), agent->position.GetY(),
    //       agent->velocity.GetX(), agent->velocity.GetY());
    agent->updateDesiredVelocity();
    // printf("V (%.3f, %.3f)\n", agent->desiredVelocity.GetX(), agent->desiredVelocity.GetY());
    agent->updateVelocity(dt);
    // printf("V1 (%.3f, %.3f, %.3f. %.3f)\n",
    //   agent->desiredLinearSpeed, agent->desiredVelocity.GetX(), agent->desiredVelocity.GetY(),
    //   agent->desiredAngularSpeed.GetValue());

    // agent->velocity = agent->desiredVelocity;
    agent->velocity = CVector2(agent->desiredLinearSpeed, agent->angle);
    agent->angle += agent->desiredAngularSpeed * dt;
    agent->position += agent->velocity * dt;
  }
  printf("\nEnd loop @ (%.3f, %.3f), (%.3f %.3f)\n",
         agent->position.GetX(), agent->position.GetY(),
         agent->velocity.GetX(), agent->velocity.GetY());
  return 0;
}
