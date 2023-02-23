#include "hl_navigation_sim/simulation.h"

#include <highfive/H5File.hpp>

using namespace hl_navigation;
using namespace hl_navigation_sim;

void Experiment::run(int seed) {
  HighFive::File file("test.h5", HighFive::File::Truncate);
  World world(dt);
  init(world, seed);
  world.prepare();
  HighFive::DataSet dataset = file.createDataSet<float>(
      "lines",
      HighFive::DataSpace({static_cast<size_t>(world.walls.size()), 2, 2}));
  std::vector<float> walls_data;
  for (const auto &wall : world.walls) {
    walls_data.push_back(wall.p1[0]);
    walls_data.push_back(wall.p1[1]);
    walls_data.push_back(wall.p2[0]);
    walls_data.push_back(wall.p2[1]);
  }
  dataset.write_raw(walls_data.data());
  dataset = file.createDataSet<float>(
      "discs",
      HighFive::DataSpace({static_cast<size_t>(world.obstacles.size()), 3}));
  std::vector<float> obstacles_data;
  for (const auto &obstacle : world.obstacles) {
    obstacles_data.push_back(obstacle.position[0]);
    obstacles_data.push_back(obstacle.position[1]);
    obstacles_data.push_back(obstacle.radius);
  }
  dataset.write_raw(obstacles_data.data());
  unsigned number = world.agents.size();
  std::vector<size_t> dims{static_cast<size_t>(steps), number, 3};
  dataset = file.createDataSet<float>("traces", HighFive::DataSpace(dims));
  std::vector<float> data(number * 3, 0.0);
  for (size_t i = 0; i < steps; i++) {
    world.update();
    float *d = reinterpret_cast<float *>(data.data());
    for (const auto &agent : world.agents) {
      *d++ = agent.pose.position[0];
      *d++ = agent.pose.position[1];
      *d++ = 0.0;
    }
    dataset.select({i, 0, 0}, {1, number, 3}).write_raw(data.data());
  }
}
