/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_SIM_SCENARIO_H
#define HL_NAVIGATION_SIM_SCENARIO_H

#include <functional>
#include <vector>

#include "hl_navigation/yaml/yaml.h"
#include "hl_navigation/property.h"
#include "hl_navigation/register.h"
#include "hl_navigation_sim/world.h"

using hl_navigation::HasProperties;
using hl_navigation::HasRegister;
using hl_navigation::Disc;
using hl_navigation::LineSegment;

namespace hl_navigation_sim {

// TODO(Jerome): forse dovrebbe essere SamplerFromRegister<....>
// cosi' lo yaml mi legge anche le proprieta' ... o forse no. meglio che prima
// pulisco

struct Scenario : virtual public HasProperties,
                  virtual public HasRegister<Scenario> {
  struct Group {
    virtual void add_to_world(World* world) = 0;
    virtual void reset() = 0;
    virtual ~Group() = default;
  };

  using Init = std::function<void(World*)>;
  using Inits = std::vector<Init>;

  explicit Scenario(const Inits& inits = {})
      : groups(), obstacles(), walls(), initializers(inits) {}

  // !!!! This may break the python autoregistration!
  // std::string get_type() const override { return name; }

  virtual void init_world(World* world) {
    for (auto& group : groups) {
      if (group) {
        group->reset();
        group->add_to_world(world);
      }
    }
    world->set_obstacles(obstacles);
    world->set_walls(walls);
    for (const auto& f : initializers) {
      f(world);
    }
  }

  void add_init(const std::function<void(World*)>& f) {
    initializers.push_back(f);
  }

  std::vector<std::unique_ptr<Group>> groups;
  std::vector<Disc> obstacles;
  std::vector<LineSegment> walls;
  std::vector<std::function<void(World*)>> initializers;
};

}  // namespace hl_navigation_sim

#endif  // HL_NAVIGATION_SIM_SCENARIO_H
