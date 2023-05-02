/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_SIM_TASK_H_
#define HL_NAVIGATION_SIM_TASK_H_

#include "hl_navigation/core/property.h"
#include "hl_navigation/core/register.h"

#include "hl_navigation_sim_export.h"

using hl_navigation::core::HasProperties;
using hl_navigation::core::HasRegister;

namespace hl_navigation::sim {

class Agent;
class World;

/**
 * @brief      This class describe the high-level task control that provides
 * navigation goals.
 */
struct HL_NAVIGATION_SIM_EXPORT Task : public virtual HasProperties, public virtual HasRegister<Task> {

  using Callback = std::function<void(const std::vector<float> & data)>;

  friend class Agent;

  /**
   * @brief      Constructs a new instance.
   * @private
   */
  explicit Task() : callbacks() {}
  virtual ~Task() = default;

  /**
   * @brief      Adds a callback called to log task events
   *
   * @param[in]  value  The desired callback
   */
  void add_callback(const Callback& value) { 
    callbacks.push_back(value); 
  }

  /**
   * @brief      Remove any callbacks
   */
  void clear_callbacks() { callbacks.clear(); }

  /**
   * @brief      Returns whether the task is done.
   *
   * @return     True if the task has finished.
   */
  virtual bool done() const { return false; }

 protected:
  /**
   * @brief      Tick the task, possibly updating the navigation goal of the agent.
   *
   * @param      agent  The agent ticking the task
   * @param      world  The world the agent is part of
   * @param[in]  time   The simulation time
   */
  virtual void update(Agent *agent, World * world, float time) {}

  std::vector<Callback> callbacks;
};
}  // namespace hl_navigation::sim

#endif /* end of include guard: HL_NAVIGATION_SIM_TASK_H_ */
