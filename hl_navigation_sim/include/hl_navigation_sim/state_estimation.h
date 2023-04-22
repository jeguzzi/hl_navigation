/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_SIM_STATE_ESTIMATION_H_
#define HL_NAVIGATION_SIM_STATE_ESTIMATION_H_

#include "hl_navigation/property.h"
#include "hl_navigation/register.h"
#include "hl_navigation_sim_export.h"

using hl_navigation::HasProperties;
using hl_navigation::HasRegister;

namespace hl_navigation_sim {

class Agent;
class World;

/**
 * @brief      This class describe a generic state estimation that should update
 * the environment state used by the agent \ref hl_navigation::Behavior.
 *
 * As the environment state is specialized by sub-classes of \ref
 * hl_navigation::Behavior like \ref hl_navigation::GeometricState, concrete
 * sub-classes have to target one or more of them.
 *
 * In particular, the agent should use a state estimation compatible with it's
 * state representation.
 *
 * \ref StateEstimation holds a pointer to the \ref World containing the agent,
 * which it queries to get the relevant entities (located nearby the agent).
 */
struct HL_NAVIGATION_SIM_EXPORT StateEstimation
    : public virtual HasProperties,
      public virtual HasRegister<StateEstimation> {
  /**
   * @brief      Constructs a new instance.
   * @private
   */
  StateEstimation() {}
  virtual ~StateEstimation() = default;

  friend class Agent;
  friend class World;

 protected:
  /**
   * @brief      Updates the state of a given agent \ref hl_navigation::Behavior
   * @param      agent  The agent owning the state estimation
   * @param[in]  world    The that the agent is part of
   */
  virtual void update(Agent *agent, World *world) const {};

  /**
   * @brief      Setup the state estimation.
   * Called before starting a simulation.
   * @param      agent  The agent owning the state estimation
   * @param[in]  world    The that the agent is part of
   */
  virtual void prepare(Agent *agent, World *world) const {};
};

}  // namespace hl_navigation_sim

#endif /* end of include guard: HL_NAVIGATION_SIM_STATE_ESTIMATION_H_ */
