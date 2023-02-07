/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef _ORCA_H_
#define _ORCA_H_

#include "../behavior.h"
#include <memory>

#define TIME_STEP 0.1

namespace RVO
{
    class Agent;
    class Obstacle;
}

namespace hl_navigation {

class ORCABehavior : public Behavior {
public:
  bool useEffectiveCenter;
  ORCABehavior(agent_type_t type, float radius, float axis_length=0.0);
  ~ORCABehavior();
  void setTimeHorizon(float value);
  float getTimeHorizon(float value) const;

  void setTimeStep(float value);
  float getTimeStep(float value) const;

  void set_line_obstacles(const std::vector<LineSegment> & value) override;

protected:
  virtual Twist2D compute_desired_twist() const override;
  virtual void update_desired_velocity() override;
  virtual void add_neighbor(const Disc & disc) override;
  virtual void add_static_obstacle(const Disc & disc) override;
  virtual void clear() override;
  virtual void prepare() override;

private:

  float timeHorizon;
  // float timeHorizonStatic;
  std::vector<std::unique_ptr<const RVO::Agent>> agentNeighbors;
  std::vector<std::unique_ptr<const RVO::Obstacle>> obstacleNeighbors;

  void add_line_obstacle(const LineSegment & line);
  void prepare_line_obstacles();

  // See [1] J. Snape, J. van den Berg, S. J. Guy, and D. Manocha, “Smooth and
  // collision-free navigation for multiple robots under differential-drive
  // constraints,” in 2010 IEEE/RSJ International Conference on Intelligent
  // Robots and Systems, 2010, pp. 4584–4589. with D=L/2

  float D;

  float rangeSq;
  std::unique_ptr<RVO::Agent> _RVOAgent;

  static const char * name;
};

}

#endif
