/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 *  (Run)
 */

#ifndef HL_NAVIGATION_SIM_EXPERIMENT_H_
#define HL_NAVIGATION_SIM_EXPERIMENT_H_

#include <filesystem>
#include <highfive/H5File.hpp>
#include <memory>

#include "hl_navigation/yaml/yaml.h"
#include "hl_navigation/yaml/yaml.h"
#include "hl_navigation_sim/scenario.h"
#include "hl_navigation_sim/world.h"

namespace hl_navigation_sim {

struct Trace {
  bool record_pose;
  bool record_twist;
  bool record_cmd;
  bool record_target;
  unsigned number;
  std::vector<float> data;
  std::shared_ptr<HighFive::DataSet> dataset;
  bool record;

  Trace()
      : record_pose(false),
        record_twist(false),
        record_cmd(false),
        record_target(false),
        number(0),
        data(), 
        dataset(nullptr),
        record(false) {}

  void init(const World & world, HighFive::Group& group, unsigned steps);
  void update(const World& world, unsigned step);
};

struct Experiment {

  virtual ~Experiment() = default;

  using Callback = std::function<void()>;

  explicit Experiment(float time_step = 0.1, int steps = 1000)
      : runs(1),
        step(0), 
        time_step(time_step),
        steps(steps),
        world(),
        scenario(nullptr),
        save_directory(),
        file(),
        run_group(),
        trace(),
        callbacks(),
        initialized(false),
        run_index(0),
        name("experiment") {}

  void run_once(int seed);

  void run();

  void init_run([[maybe_unused]] int seed) {
    world = make_world();
    if (scenario) {
      scenario->init_world(world.get());
    }
  }

  virtual std::shared_ptr<World> make_world() {
    return std::make_shared<World>();
  }

  virtual std::string dump();
  void init_dataset();
  void init_dataset_run(unsigned index);

  void add_callback(const Callback & value) {
    callbacks.push_back(value);
  }

  float get_time() const {
    return time_step * step;
  }

  std::string get_path() const {
    if (file) {
      std::cerr << file->getPath() << " : " << file->getName() << std::endl;
      return file->getPath();
    }
    return "";
  }

  unsigned runs;
  unsigned step;
  float time_step;
  unsigned steps;
  std::shared_ptr<World> world;
  // std::shared_ptr<WorldGenerator> scenario;
  std::shared_ptr<Scenario> scenario;
  std::filesystem::path save_directory;
  std::shared_ptr<HighFive::File> file;
  std::shared_ptr<HighFive::Group> run_group;
  Trace trace;
  std::vector<Callback> callbacks;
  bool initialized;
  unsigned run_index;
  std::string name;
};

}  // namespace hl_navigation_sim

#endif /* end of include guard: HL_NAVIGATION_SIM_EXPERIMENT_H_ */
