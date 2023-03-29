#include "hl_navigation_sim/experiment.h"

#include <filesystem>
#include <highfive/H5File.hpp>

#include "hl_navigation/yaml/yaml.h"
#include "hl_navigation_sim/world.h"
#include "hl_navigation_sim/yaml/experiment.h"
#include "hl_navigation_sim/yaml/world.h"

using namespace hl_navigation;
using namespace hl_navigation_sim;

namespace fs = std::filesystem;

static void store_world(const World &world, HighFive::Group &group) {
  const std::string yaml = YAML::dump<World>(&world);
  HighFive::Attribute a = group.createAttribute<std::string>(
      "world", HighFive::DataSpace::From(yaml));
  a.write(yaml);
}

static void store_experiment(const std::string & yaml, HighFive::File &file) {
  HighFive::Attribute a = file.createAttribute<std::string>("experiment", yaml);
  a.write(yaml);
}

static std::string current_stamp() {
  auto time = std::time(nullptr);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&time),
                      "%F_%T");  // ISO 8601 without timezone information.
  auto s = ss.str();
  std::replace(s.begin(), s.end(), ':', '-');
  return s;
}

void Trace::init(const World &world, HighFive::Group &group, unsigned steps) {
  // TODO(Jerome): set the column names
  unsigned n = 0;
  if (record_pose) n += 3;
  if (record_twist) n += 3;
  if (record_cmd) n += 3;
  if (record_target) n += 3;
  record = n > 0;
  if (!record) return;
  number = world.agents.size();
  const std::vector<size_t> dims{static_cast<size_t>(steps), number, n};
  auto ds = group.createDataSet<float>("traces", HighFive::DataSpace(dims));
  data = std::vector<float>(number * n, 0.0f);
  dataset = std::make_shared<HighFive::DataSet>(std::move(ds));
}

void Trace::update(const World &world, unsigned step) {
  if (!record) return;
  float *d = reinterpret_cast<float *>(data.data());
  for (const auto &agent : world.agents) {
    if (record_pose) {
      const auto pose = agent->pose;
      *d++ = pose.position[0];
      *d++ = pose.position[1];
      *d++ = pose.orientation;
    }
    if (record_twist) {
      const auto twist = agent->twist;
      *d++ = twist.velocity[0];
      *d++ = twist.velocity[1];
      *d++ = twist.angular_speed;
    }
    if (record_cmd) {
      const auto twist = agent->cmd_twist;
      *d++ = twist.velocity[0];
      *d++ = twist.velocity[1];
      *d++ = twist.angular_speed;
    }
    if (record_target && agent->nav_behavior) {
      const auto pose = agent->nav_behavior->get_target_pose();
      *d++ = pose.position[0];
      *d++ = pose.position[1];
      *d++ = pose.orientation;
    }
  }
  dataset->select({step, 0, 0}, {1, number, 3}).write_raw(data.data());
}

void Experiment::init_dataset() {
  file = nullptr;
  if (save_directory.empty()) {
    return;
  }
  fs::current_path(save_directory);
  const std::string dir_name = name + "_" + current_stamp();
  const fs::path dir = save_directory / dir_name;
  if (!create_directory(dir)) {
    std::cerr << "Could not create directory " << dir << std::endl;
    return;
  }
  fs::current_path(dir);
  file = std::make_shared<HighFive::File>("data.h5", HighFive::File::Truncate);
  store_experiment(dump(), *file);
}

std::string Experiment::dump() {
  return YAML::dump<Experiment>(this);
}

void Experiment::init_dataset_run(unsigned index) {
  if (!file) {
    trace.record = false;
    return;
  }
  auto group = file->createGroup("run_" + std::to_string(index));
  store_world(*world, group);
  trace.init(*world, group, steps);
  run_group = std::make_shared<HighFive::Group>(std::move(group));
}

void Experiment::run() {
  for (size_t i = 0; i < runs; i++) {
    run_once(i);
  }
}

void Experiment::run_once(int seed) {
  if (!initialized) {
    init_dataset();
    initialized = true;
  }
  init_run(seed);
  if (!world) {
    std::cerr << "No world" << std::endl;
    return;
  }
  world->prepare();
  init_dataset_run(seed);
  for (step = 0; step < steps; step++) {
    world->update(time_step);
    trace.update(*world, step);
    for (const auto &cb : callbacks) {
      cb();
    }
  }
}
