/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */


#include <tuple>
#include <vector>

#include "simulation.h"


class CrossExperiment : public Experiment {
 public:
  CrossExperiment(float dt = 0.05, int steps = 2000, const char * behavior = "HL",
                  float radius = 4.0, unsigned number = 7, float margin = 1.0) :
                  Experiment(dt, steps),
                  behavior(behavior), radius(radius), number(number), margin(margin) {}

 protected:
  void init(World & world, int seed) override {
    float x = radius - margin;
    std::vector<std::tuple<Vector2, std::function<Vector2(int, int)>>> task = {
        {Vector2(radius, 0.0),
         [x](int i, unsigned number) {return Vector2(-x + 2 * x * i / (number  - 1), 0.49  );}},
        {Vector2(-radius, 0.0),
         [x](int i, unsigned number) {return Vector2(-x + 2 * x * i / (number  - 1), -0.51 );}},
        {Vector2(0.0, radius),
         [x](int i, unsigned number) {return Vector2(0.52, -x + 2 * x * i / (number  - 1)  );}},
        {Vector2(0.0, -radius),
         [x](int i, unsigned number) {return Vector2(-0.53, -x + 2 * x * i / (number  - 1) );}}};


//      std::vector<std::tuple<Vector2, std::function<Vector2(int, int)>>> task = {
//          {Vector2(radius, 0.00), [this](int i, unsigned number) {return Vector2(-radius, 0.6 * i);}},
//          {Vector2(-radius, 0.00), [this](int i, unsigned number) {return Vector2(radius, 0.6 * i + 0.01);}},
//      };



     for (auto & [target, position] : task) {
       for (size_t i = 0; i < number; i++) {
          auto task = std::make_unique<WayPointsTask>(std::vector<Vector2>{target, -target}, true);
          auto se = std::make_unique<BoundedStateEstimation>(&world, 1.0, 1.0);
          auto & agent = world.agents.emplace_back(
            behavior, 0.1, 0.1, std::move(task), std::move(se), dt);
          agent.nav_behavior->set_max_speed(1.0);
          agent.nav_behavior->set_max_angular_speed(1.0);
          agent.nav_behavior->set_optimal_speed(1.0);
          agent.nav_behavior->set_optimal_angular_speed(1.0);
          agent.nav_behavior->set_horizon(1.0);
          agent.nav_controller.distance_tolerance = 1.0;
          agent.nav_controller.angle_tolerance = 4.0;
          agent.nav_controller.speed_tolerance = 0.1;
          agent.position = position(i, number);
        }
      }
    }

 private:
    const char * behavior;
    float radius;
    unsigned number;
    float margin;
};


int main(int argc, char *argv[]) {
  CrossExperiment experiment;
  printf("Start\n");
  experiment.run(0);
  printf("End\n");
  return 0;
}
