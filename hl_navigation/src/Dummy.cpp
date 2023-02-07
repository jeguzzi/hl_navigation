/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "Dummy.h"

void DummyAgent::update_desired_velocity() {
  CVector2 delta = targetPosition - position;
  desiredVelocity = optimalSpeed * delta / delta.norm();
}

const char * DummyAgent::name = register_type<DummyAgent>("Dummy");
