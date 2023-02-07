/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef _DUMMY_H_
#define _DUMMY_H_

#include "../behavior.h"

namespace hl_navigation {

class DummyBehavior : public Behavior {

public:

  DummyBehavior(agent_type_t type, float radius, float axis_length=0.0) :
    Behavior(type, radius, axis_length) {}
  ~DummyBehavior() {}

protected:
  virtual void update_desired_velocity() override;

private:
  static const char * name;
};

}

#endif
