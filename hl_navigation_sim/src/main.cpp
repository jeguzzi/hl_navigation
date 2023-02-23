/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */


#include <tuple>
#include <vector>

#include "hl_navigation_sim/simulation.h"

#include "./cross.h"
#include "./collisions.h"


static void show_usage(std::string name) {
  std::cout << "Usage: " << name << " <option(s)>" << std::endl
            << "Options:" << std::endl
            << "  --help\t\t\tShow this help message" << std::endl
            << "  --experiment=<NAME>\t\tType of experiment to run" << std::endl
            << "  --steps=<STEPS>\t\tNumber of simulation steps" << std::endl
            << "  --time_step=<TIME_STEP>\tDuration of a simulation step" << std::endl
            << "  --behavior=<BEHAVIOR>\tNavigation behavior" << std::endl;
}

std::unique_ptr<hl_navigation_sim::Experiment> create_experiment(
    const std::string & name, float time_step, unsigned steps, const char * behavior_name) {
  if (name == "cross") {
    return std::make_unique<hl_navigation_sim::CrossExperiment>(time_step, steps, behavior_name);
  } else if (name == "collision") {
    return std::make_unique<hl_navigation_sim::CollisionsTest>(time_step, steps, behavior_name);
  }
  return nullptr;
}

int main(int argc, char *argv[]) {
  char experiment_name[10] = "cross";
  char behavior_name[10] = "Dummy";
  unsigned steps = 100;
  float time_step = 0.1;
  for (int i = 0; i < argc; i++) {
    if (sscanf(argv[i], "--experiment=%10s", experiment_name)) {
      continue;
    }
    if (sscanf(argv[i], "--behavior=%10s", behavior_name)) {
      continue;
    }
    if (sscanf(argv[i], "--steps=%u", &steps)) {
      continue;
    }
    if (sscanf(argv[i], "--time_step=%f", &time_step)) {
      continue;
    }
    if (strcmp(argv[i], "--help") == 0) {
      show_usage(argv[0]);
      return 0;
    }
  }
  auto experiment = create_experiment(experiment_name, time_step, steps, behavior_name);
  if (!experiment) {
    printf("Failed to find experiment %s\n", experiment_name);
    return 1;
  }
  printf("Start experiment %s - %s\n", experiment_name, behavior_name);
  experiment->run(0);
  printf("End\n");
  return 0;
}
