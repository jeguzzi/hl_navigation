/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <vector>

#include "hl_navigation/core/behavior.h"
#include "hl_navigation/core/behaviors/dummy.h"
#include "hl_navigation/core/common.h"
#include "hl_navigation/core/controller.h"

using hl_navigation::core::Action;
using hl_navigation::core::DummyBehavior;
using hl_navigation::core::Controller;
using hl_navigation::core::OmnidirectionalKinematics;
using hl_navigation::core::Twist2;
using hl_navigation::core::Vector2;

void move(Controller *controller, Vector2 target) {
  printf("Next target (%.2f, %.2f)\n", target.x(), target.y());
  auto action = controller->go_to_position(target, 0.1f);
  action->done_cb = [controller, target = target](Action::State state) {
    if (state == Action::State::success) {
      move(controller, -target);
    } else {
      printf("Stopped\n");
    }
  };
  action->running_cb = [](float time) {
    printf("In progress ... %.2f s\n", time);
  };
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[]) {
  auto behavior = std::make_shared<DummyBehavior>(std::make_shared<OmnidirectionalKinematics>(1.0, 1.0), 0.1);
  Controller controller(behavior);
  controller.set_speed_tolerance(0.05);
  float dt = 0.03;
  controller.set_cmd_cb(
      [&, behavior, dt](const Twist2 &cmd) { behavior->actuate(cmd, dt); });
  move(&controller, {1.0, 0.0});
  for (int i = 0; i < 1000; ++i) {
    controller.update(dt);
  }
  return 0;
}
