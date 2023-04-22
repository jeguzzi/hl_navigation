/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "hl_navigation_sim/tasks/waypoints.h"

#include "hl_navigation/states/geometric.h"
#include "hl_navigation_sim/agent.h"

namespace hl_navigation_sim {

void WaypointsTask::update(Agent *agent, [[maybe_unused]] World * world, float time) {
  auto c = agent->get_controller();
  if (c->idle()) {
    if (waypoint != waypoints.end()) {
      c->go_to_position(*waypoint, tolerance);
      running = true;
      for (const auto &cb : callbacks) {
        cb({time, 1.0f, waypoint->x(), waypoint->y()});
      }
      ++waypoint;
      if (loop && waypoint == waypoints.end()) {
        waypoint = waypoints.begin();
      }
    } else if (running) {
      for (const auto &cb : callbacks) {
        cb({time, 0.0f, 0.0f, 0.0f});
      }
      running = false;
    }
  }
}

bool WaypointsTask::done() const { return waypoint == waypoints.end(); }

}  // namespace hl_navigation_sim
