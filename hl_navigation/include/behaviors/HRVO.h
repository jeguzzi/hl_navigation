/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef _HRVO_H_
#define _HRVO_H_

#include "../behavior.h"

namespace HRVO
{
  class Agent;
}

namespace hl_navigation {

// TODO(J:revision2023): verify DIFFERENTIAL_DRIVE

class HRVOBehavior : public Behavior {
public:
  HRVOBehavior(agent_type_t type, float radius, float axis_length=0.0);
  ~HRVOBehavior();

protected:
  virtual void update_desired_velocity() override;
  virtual void add_neighbor(const Disc & disc) override;
  virtual void add_static_obstacle(const Disc & disc) override;
  virtual void clear() override;
  virtual void prepare() override;
private:
  uint agentIndex;
  float rangeSq;
  std::unique_ptr<HRVO::Agent> _HRVOAgent;
  static const char * name;
};

}

#endif
