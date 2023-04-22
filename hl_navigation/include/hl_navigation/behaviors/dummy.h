/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_BEHAVIOR_DUMMY_H_
#define HL_NAVIGATION_BEHAVIOR_DUMMY_H_

#include "hl_navigation/behavior.h"
#include "hl_navigation_export.h"

namespace hl_navigation {

/**
 * @brief      Dummy behavior that ignores obstacles.
 *
 * Mainly useful to test the interaction with other components
 * 
 * *State*: empty
 */
class HL_NAVIGATION_EXPORT DummyBehavior : public Behavior {
 public:
  /**
   * @brief      Contruct a new instance
   *
   * @param[in]  kinematics  The kinematics
   * @param[in]  radius      The radius
   */
  DummyBehavior(std::shared_ptr<Kinematics> kinematics = nullptr,
                float radius = 0.0f)
      : Behavior(kinematics, radius) {}

  /** 
   * @private
  */
  std::string get_type() const override { return type; }

 protected:
  Vector2 compute_desired_velocity([[maybe_unused]] float time_step) override;

 private:
  static inline const std::string type = register_type<DummyBehavior>("Dummy");
};

}  // namespace hl_navigation

#endif  // HL_NAVIGATION_BEHAVIOR_DUMMY_H_
