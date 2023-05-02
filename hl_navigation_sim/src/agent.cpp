/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "hl_navigation/sim/agent.h"

namespace hl_navigation::sim {

void Agent::update(float dt, float time, World * world) {
  control_deadline -= dt;
  if (control_deadline > 0) {
    return;
  }
  control_deadline += control_period;

  if (task) task->update(this, world, time);
  if (state_estimation) state_estimation->update(this, world);
  if (behavior) {
    behavior->set_actuated_twist(last_cmd);
    behavior->set_twist(twist);
    behavior->set_pose(pose);
  }
  last_cmd = controller.update(std::max(control_period, dt));
  // last_cmd = behavior->get_actuated_twist(true);
}

void Agent::actuate(float dt) {
  twist = last_cmd;  // + collision_force / mass * dt;
  pose = pose.integrate(twist, dt);
}

void Agent::set_behavior(const std::shared_ptr<Behavior> &value) {
  behavior = value;
  controller.set_behavior(value);
  if (behavior) {
    behavior->set_radius(radius);
    if (!behavior->get_kinematics()) {
      behavior->set_kinematics(kinematics);
    }
  }
}

bool Agent::idle() const {
  return (!task || task->done()) && controller.idle();
}

}  // namespace hl_navigation::sim
