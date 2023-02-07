/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "behavior.h"
#include <iostream>
#include <iterator>
#include <vector>

// Should also add the wheels

using namespace hl_navigation;

static void show_usage(std::string name) {
  std::vector<std::string> keys = Behavior::behavior_names();
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
  auto behavior = Behavior::behavior_with_name(behavior_name, HOLONOMIC, 0.1, 0.1);
  if (!behavior) {
    printf("No behavior with name %s\n", behavior_name);
    exit(1);
  }
  const auto& r = *behavior.get();
  printf("Use behavior %s - %s\n", behavior_name, typeid(r).name());
  float dt = 0.1;
  behavior->set_max_speed(1.0);
  behavior->set_max_angular_speed(1.0);
  behavior->set_optimal_speed(1.0);
  behavior->set_optimal_angular_speed(1.0);
  behavior->set_horizon(1.0);

  behavior->position = Vector2(0.0f, 0.05f);
  behavior->velocity = Vector2(0.0f, 0.0f);
  // Go to 1, 0
  behavior->targetPosition = Vector2(3.0f, 0.0f);
  behavior->set_static_obstacles({Disc(Vector2(1.5f, 0.0f), 0.5f)});
  // This should be in init
  printf("Start loop @ (%.3f, %.3f)\n", behavior->position.x(), behavior->position.y());
  for (size_t i = 0; i < 30; i++) {
    // printf("P (%.3f, %.3f, %.3f)\n",
    //   behavior->position.GetX(), behavior->position.GetY(), behavior->angle.GetValue());
    // TODO(Jerome): rivedere tutte le cache.
    // Per esempio, qui mi obbliga a ricreare gli ostacoli ogni volta

    printf("%.3f, %.3f,", behavior->position.x(), behavior->position.y());
    //       behavior->velocity.GetX(), behavior->velocity.GetY());
    // printf("%zu (%.3f, %.3f), (%.3f, %.3f)\n", i, behavior->position.GetX(), behavior->position.GetY(),
    //       behavior->velocity.GetX(), behavior->velocity.GetY());
    // behavior->updateDesiredVelocity();
    // printf("V (%.3f, %.3f)\n", behavior->desiredVelocity.GetX(), behavior->desiredVelocity.GetY());
    behavior->update(dt);
    // std::cout << behavior->desiredVelocity << std::endl;
    // std::cout << behavior->get_target_velocity() << std::endl;
    // printf("V1 (%.3f, %.3f, %.3f. %.3f)\n",
    //   behavior->desiredLinearSpeed, behavior->desiredVelocity.GetX(), behavior->desiredVelocity.GetY(),
    //   behavior->desiredAngularSpeed.GetValue());

    // behavior->velocity = behavior->desiredVelocity;

    behavior->velocity = behavior->get_target_velocity();
    behavior->angle += behavior->target_twist.angular * dt;
    behavior->position += behavior->velocity * dt;
  }
  printf("\nEnd loop @ (%.3f, %.3f), (%.3f %.3f)\n",
         behavior->position.x(), behavior->position.y(),
         behavior->velocity.x(), behavior->velocity.y());
  return 0;
}
