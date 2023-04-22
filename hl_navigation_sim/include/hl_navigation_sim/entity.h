/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_SIM_ENTITY_H_
#define HL_NAVIGATION_SIM_ENTITY_H_

#include "hl_navigation_sim_export.h"

namespace hl_navigation_sim {

/**
 * @brief      Super-class that adds a unique ID to world entities.
 *
 * This unique ID should not be fed to navigation behaviors,
 * but only used internally by the simulation,
 * for instance, to identify entities in a UI.
 */
struct HL_NAVIGATION_SIM_EXPORT Entity {
  /**
   * @brief      Constructs a new instance.
   */
  Entity() : uid(_uid++) {}

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  id    The identifier
   */
  explicit Entity(unsigned id) : uid(id) {}

  /**
   * Unique identifier
   */
  unsigned uid;

 private:
  static inline unsigned _uid = 0;
};

}  // namespace hl_navigation_sim

#endif /* end of include guard: HL_NAVIGATION_SIM_ENTITY_H_ */
