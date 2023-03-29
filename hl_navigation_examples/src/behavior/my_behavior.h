/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_EXAMPLES_MY_BEHAVIOR_H_
#define HL_NAVIGATION_EXAMPLES_MY_BEHAVIOR_H_

#include "hl_navigation/behavior.h"

namespace hl_navigation {

/**
 * @brief      Idle behavior that always stay still.
 *
 * It showcases how to define and use a new behavior from an external shared
 * library.
 */
class IdleBehavior : public Behavior {
 public:
  using Behavior::Behavior;

  std::string get_type() const override { return type; }

 protected:
  Vector2 compute_desired_velocity([[maybe_unused]] float time_step) override {
    return Vector2::Zero();
  }

 private:
  static inline const std::string type = register_type<IdleBehavior>("Idle");
};

}  // namespace hl_navigation

#endif  // HL_NAVIGATION_EXAMPLES_MY_BEHAVIOR_H_
