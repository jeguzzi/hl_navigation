/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "hl_navigation_sim/agent.h"

namespace hl_navigation_sim {

void Agent::update(float dt, float time, World * world) {
  control_deadline -= dt;
  if (control_deadline > 0) {
    return;
  }
  control_deadline += control_period;

  if (task) task->update(this, world, time);
  if (state_estimation) state_estimation->update(this, world);
  if (behavior) {
    behavior->set_actuated_twist(cmd_twist);
    behavior->set_twist(twist);
    behavior->set_pose(pose);
  }
  cmd_twist = controller.update(std::max(control_period, dt));
  // cmd_twist = behavior->get_actuated_twist(true);
}

void Agent::actuate(float dt) {
  twist = cmd_twist;  // + collision_force / mass * dt;
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

}  // namespace hl_navigation_sim
