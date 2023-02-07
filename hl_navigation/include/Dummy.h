/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef _DUMMY_H_
#define _DUMMY_H_

#include "Agent.h"

class DummyAgent : public Agent {

public:

  DummyAgent(agent_type_t type, float radius, float axis_length=0.0) :
    Agent(type, radius, axis_length) {}
  ~DummyAgent() {}

protected:
  virtual void update_desired_velocity() override;

private:
  static const char * name;
};

#endif
