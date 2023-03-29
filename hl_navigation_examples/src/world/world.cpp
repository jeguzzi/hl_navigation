/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "hl_navigation_sim/world.h"

#include <iostream>

#include "hl_navigation/behaviors/dummy.h"
#include "hl_navigation_sim/yaml/world.h"

using hl_navigation::DummyBehavior;
using hl_navigation::Holonomic;
using hl_navigation::Vector2;
namespace sim = hl_navigation_sim;

int main() {
  sim::World world;
  world.walls.emplace_back(Vector2{-1.0f, -1.f}, Vector2{-1.f, 1.f});
  world.obstacles.emplace_back(Vector2{2.f, 0.f}, 0.5f);
  auto a = std::make_shared<sim::Agent>(
      0.1, std::make_shared<DummyBehavior>(), std::make_shared<Holonomic>(1.0),
      std::make_shared<sim::WayPointsTask>(sim::Waypoints{{1, 0}}, false, 0.1),
      nullptr, 0.1);
  world.agents.push_back(std::move(a));
  std::cout << "\nLoaded world\n====================" << std::endl;
  std::cout << YAML::dump<sim::World>(&world) << std::endl;
  world.run(100, 0.1);
  std::cout
      << "\nAfter simulating for 10 s at 0.1 s steps\n===================="
      << std::endl;
  std::cout << YAML::dump<sim::World>(&world) << std::endl;
  return 0;
}
