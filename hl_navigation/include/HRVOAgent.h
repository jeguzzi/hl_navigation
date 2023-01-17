/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef _HRVOAGENT_H_
#define _HRVOAGENT_H_

#include "Agent.h"

namespace HRVO
{
    class Agent;
}

class HRVOAgent : public Agent {
public:
  HRVOAgent(agent_type_t type, float radius, float axis_length=0.0);
  ~HRVOAgent();

protected:
  virtual void update_desired_velocity() override;
  virtual void add_neighbor(const Disc & disc) override;
  virtual void add_static_obstacle(const Disc & disc) override;
  virtual void clear() override;
  virtual void prepare() override;
private:
  uint agentIndex;
  Real rangeSq;
  std::unique_ptr<HRVO::Agent> _HRVOAgent;
  static const char * name;
};

#endif
