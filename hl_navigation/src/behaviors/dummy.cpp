/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "behaviors/dummy.h"

namespace hl_navigation {

void DummyBehavior::update_desired_velocity() {
  Vector2 delta = targetPosition - position;
  desiredVelocity = optimalSpeed * delta / delta.norm();
}

const char * DummyBehavior::name = register_type<DummyBehavior>("Dummy");

}  // namespace hl_navigation
