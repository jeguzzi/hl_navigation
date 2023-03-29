/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_BEHAVIOR_DUMMY_H_
#define HL_NAVIGATION_BEHAVIOR_DUMMY_H_

#include "hl_navigation/behavior.h"

namespace hl_navigation {

/**
 * @brief      Dummy behavior that ignores obstacles.
 *
 * Mainly useful to test the interaction with other components
 */
class DummyBehavior : public Behavior {
 public:
  using Behavior::Behavior;

  std::string get_type() const override { return type; }

 protected:
  Vector2 compute_desired_velocity([[maybe_unused]] float time_step) override;

 private:
  static inline const std::string type = register_type<DummyBehavior>("Dummy");
};

}  // namespace hl_navigation

#endif  // HL_NAVIGATION_BEHAVIOR_DUMMY_H_
