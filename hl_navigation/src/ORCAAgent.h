/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef _ORCAAGENT_H_
#define _ORCAAGENT_H_

#include "Agent.h"
#include "RVO/Agent.h"
#include "RVO/Definitions.h"
#include "RVO/Obstacle.h"
#include "RVO/Vector2.h"

#define TIME_STEP 0.1

using namespace argos;

class ORCAAgent : public Agent {
public:
  virtual void setTimeHorizon(double value);
  virtual void setAperture(double value){};
  virtual void setResolution(unsigned int value){};
  virtual void setTau(double value){};
  virtual void setEta(double value){};

  virtual void updateDesiredVelocity();
  virtual void updateVelocity(float);
  // virtual void Init(TConfigurationNode& t_tree);
  virtual void clearObstacles();

  virtual void addObstacleAtPoint(CVector2 p, CVector2 v, Real r,
                                  Real socialMargin);
  virtual void addObstacleAtPoint(CVector2 p, Real r, Real socialMargin);
  virtual void updateRepulsiveForce(){};

  ORCAAgent();
  ~ORCAAgent();

  bool useEffectiveCenter;

private:
  Real timeHorizon;

  Real timeHorizonStatic;
  std::vector<const RVO::Agent *> agentNeighbors;

  // See [1] J. Snape, J. van den Berg, S. J. Guy, and D. Manocha, “Smooth and
  // collision-free navigation for multiple robots under differential-drive
  // constraints,” in 2010 IEEE/RSJ International Conference on Intelligent
  // Robots and Systems, 2010, pp. 4584–4589. with D=L/2

  Real D;

  Real rangeSq;
  virtual void setup();
  RVO::Agent *_RVOAgent;

  static const char * name;
};

#endif
