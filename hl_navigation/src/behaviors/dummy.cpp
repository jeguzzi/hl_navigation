/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "hl_navigation/behaviors/dummy.h"

namespace hl_navigation {

Vector2 DummyBehavior::compute_desired_velocity([[maybe_unused]] float dt) {
  auto delta = target_pose.position - pose.position;
  return optimal_speed * delta / delta.norm();
}

// std::string DummyBehavior::type = register_type<DummyBehavior>("Dummy");

// const char* DummyBehavior::name = register_type<DummyBehavior>("Dummy");

}  // namespace hl_navigation
