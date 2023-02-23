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

protected:

  Vector2 compute_desired_velocity() override;

private:
  static const char * name;
};

}

#endif  // HL_NAVIGATION_BEHAVIOR_DUMMY_H_
