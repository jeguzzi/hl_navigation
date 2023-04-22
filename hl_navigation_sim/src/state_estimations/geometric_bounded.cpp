/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "hl_navigation_sim/state_estimations/geometric_bounded.h"

#include "hl_navigation_sim/agent.h"
#include "hl_navigation_sim/world.h"

namespace hl_navigation_sim {

void BoundedStateEstimation::update(Agent *agent, World * world) const {
  // bool v1 = dynamic_cast<GeometricState *>(agent->behavior.get()) !=
  // nullptr; std::cout << "SE::update: behavior is geometric? " <<
  // agent->behavior << " " << v1 << std::endl; GeometricState *state =
  //     dynamic_cast<GeometricState *>(agent->behavior.get());
  GeometricState *state = agent->get_geometric_state();
  if (state) {
    state->set_neighbors(neighbors_of_agent(agent, world));
  } else {
    std::cerr << "Not a geometric state" << std::endl;
    // << typeid(*agent->behavior.get()).name() << std::endl;
  }
}

void BoundedStateEstimation::prepare(Agent *agent, World * world) const {
  // if (GeometricState *state =
  //         dynamic_cast<GeometricState *>(agent->behavior.get())) {
  if (GeometricState *state = agent->get_geometric_state()) {
    state->set_static_obstacles(world->get_discs());
    state->set_line_obstacles(world->get_line_obstacles());
  }
}

std::vector<Neighbor> BoundedStateEstimation::neighbors_of_agent(
    const Agent *agent, const World * world) const {
  return world->get_neighbors(agent, range_of_view);
}

#if 0
std::vector<Neighbor> BoundedStateEstimation::neighbors_of_agent(
    const Agent *agent) const {
  std::vector<Neighbor> ns;

  auto const cs = world->get_agents_in_region(bounding_box(agent));
  for (const Agent *neighbor : cs) {
    if (neighbor != agent && visible(agent, neighbor)) {
      ns.push_back(perceive_neighbor(agent, neighbor));
    }
  }
  return ns;
}

Neighbor BoundedStateEstimation::perceive_neighbor(
    [[maybe_unused]] const Agent *agent, const Agent *neighbor) const {
  return Neighbor(neighbor->pose.position, neighbor->radius,
                  neighbor->twist.velocity, neighbor->id);
}

bool BoundedStateEstimation::visible(
    [[maybe_unused]] const Agent *agent,
    [[maybe_unused]] const Agent *neighbor) const {
  return true;
}

BoundingBox BoundedStateEstimation::bounding_box(const Agent *agent) const {
  return {agent->pose.position[0] - range_of_view,
          agent->pose.position[0] + range_of_view,
          agent->pose.position[1] - range_of_view,
          agent->pose.position[1] + range_of_view};
}

#endif

}  // namespace hl_navigation_sim
