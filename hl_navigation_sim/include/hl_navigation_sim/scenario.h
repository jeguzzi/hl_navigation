/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_SIM_SCENARIO_H
#define HL_NAVIGATION_SIM_SCENARIO_H

#include <functional>
#include <vector>

#include "hl_navigation/property.h"
#include "hl_navigation/register.h"
#include "hl_navigation/yaml/yaml.h"
#include "hl_navigation_sim/world.h"

#include "hl_navigation_sim_export.h"

using hl_navigation::Disc;
using hl_navigation::HasProperties;
using hl_navigation::HasRegister;
using hl_navigation::LineSegment;

namespace hl_navigation_sim {

/**
 * @brief      A scenario describes a distribution of \ref World
 * that can be sampled to perform an experiment.
 */
struct HL_NAVIGATION_SIM_EXPORT Scenario : virtual public HasProperties,
                  virtual public HasRegister<Scenario> {
  /**
   * @brief      A group of agents that can be generated and added to the world.
   */
  struct Group {
    /**
     * @brief      Generate and add the agents to the world.
     *
     * @param      world  The world
     */
    virtual void add_to_world(World* world) = 0;
    /**
     * @brief      Resets the agent generator.
     */
    virtual void reset() = 0;
    virtual ~Group() = default;
  };

  /**
   * A world initializer
   */
  using Init = std::function<void(World*)>;
  /**
   * A collection of world initializers
   */
  using Inits = std::vector<Init>;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  inits  The collection of world initializers to use.
   */
  explicit Scenario(const Inits& inits = {})
      : groups(), obstacles(), walls(), initializers(inits) {}

  // !!!! This may break the auto-registration!
  // std::string get_type() const override { return name; }

  /**
   * @brief      Initializes the world.
   *
   * @param      world  The world
   */
  virtual void init_world(World* world);

  /**
   * @brief      Adds a world initializer.
   *
   * @param[in]  f     The initializer
   */
  void add_init(const std::function<void(World*)>& f) {
    initializers.push_back(f);
  }

  /**
   * @brief      Gets the world initializers.
   *
   * @return     The initializers.
   */
  const Inits & get_initializers() const {
    return initializers;
  }

  /**
   * Groups
   */
  std::vector<std::unique_ptr<Group>> groups;
  /**
   * Obstacles
   */
  std::vector<Disc> obstacles;
  /**
   * Walls
   */
  std::vector<LineSegment> walls;

 private:
  Inits initializers;
};

}  // namespace hl_navigation_sim

#endif  // HL_NAVIGATION_SIM_SCENARIO_H
