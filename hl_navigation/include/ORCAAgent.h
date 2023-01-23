/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef _ORCAAGENT_H_
#define _ORCAAGENT_H_

#include "Agent.h"
#include <memory>

#define TIME_STEP 0.1

namespace RVO
{
    class Agent;
    class Obstacle;
}

class ORCAAgent : public Agent {
public:
  bool useEffectiveCenter;
  ORCAAgent(agent_type_t type, float radius, float axis_length=0.0);
  ~ORCAAgent();
  void setTimeHorizon(double value);
  float getTimeHorizon(double value) const;

  void setTimeStep(double value);
  float getTimeStep(double value) const;

  void set_line_obstacles(const std::vector<LineSegment> & value) override;

protected:
  virtual Twist2D compute_desired_twist() const override;
  virtual void update_desired_velocity() override;
  virtual void add_neighbor(const Disc & disc) override;
  virtual void add_static_obstacle(const Disc & disc) override;
  virtual void clear() override;
  virtual void prepare() override;

private:

  Real timeHorizon;
  // Real timeHorizonStatic;
  std::vector<std::unique_ptr<const RVO::Agent>> agentNeighbors;
  std::vector<std::unique_ptr<const RVO::Obstacle>> obstacleNeighbors;

  void add_line_obstacle(const LineSegment & line);
  void prepare_line_obstacles();

  // See [1] J. Snape, J. van den Berg, S. J. Guy, and D. Manocha, “Smooth and
  // collision-free navigation for multiple robots under differential-drive
  // constraints,” in 2010 IEEE/RSJ International Conference on Intelligent
  // Robots and Systems, 2010, pp. 4584–4589. with D=L/2

  Real D;

  Real rangeSq;
  std::unique_ptr<RVO::Agent> _RVOAgent;

  static const char * name;
};

#endif
