/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef _HLAGENT_H_
#define _HLAGENT_H_

#include "Agent.h"

// trajectory planning

#define UNKNOWN -2
#define MAX_RESOLUTION 301

// configuration

#define DEFAULT_APERTURE CRadians::PI
#define DEFAULT_RESOLUTION 40
#define DEFAULT_TAU 0.1
#define MIN_TAU 0.1

typedef struct {
  Real centerDistance;
  Real radius;
  Real sensingMargin;
  Real agentSensingMargin;

  // Real socialMargin;
  // Real penetration;
  CVector2 dx;
  CVector2 va;
  CVector2 position;
  Real C;
  CRadians gamma, visibleAngle;
} AgentCache;

typedef std::list<AgentCache> obstacleList_t;
typedef obstacleList_t::iterator obstacleIterator_t;
typedef std::list<AgentCache> agentList_t;
typedef agentList_t::iterator agentIterator_t;

class HLAgent : public Agent {

public:
  // Mobility
  // Navigation

  virtual void addObstacleAtPoint(CVector2 point, Real radius,
                                  Real socialMargin);
  virtual void addObstacleAtPoint(CVector2 point, CVector2 velocity,
                                  Real radius, Real socialMargin);

  virtual void clearObstacles();
  virtual void updateDesiredVelocity();
  virtual void updateRepulsiveForce();
  virtual void updateVelocity(float);
  // virtual void Init(TConfigurationNode& t_tree);

  Real *collisionMap();
  unsigned int resolution;

  HLAgent();
  ~HLAgent();

  std::vector<Real> getDistances();

  virtual void setTau(double value);
  virtual void setEta(double value);
  virtual void setAperture(double value);
  virtual void setResolution(unsigned int value);
  virtual void setTimeHorizon(double value){};

  CRadians angleResolution();
  CRadians aperture;

private:
  virtual void setup();

  Real effectiveHorizon;

  Real tau;
  Real eta;

  CVector2 previousDesiredVelocity;
  void prepareAgents();
  Real distanceCache[MAX_RESOLUTION];
  Real staticDistanceCache[MAX_RESOLUTION];
  void initDistanceCache();
  unsigned int indexOfRelativeAngle(CRadians relativeAngle);
  Real computeDistanceToCollisionAtRelativeAngle(CRadians relativeAngle,
                                                 Real *staticCache);
  Real fearedDistanceToCollisionAtRelativeAngle(CRadians angle);
  Real distForAngle(AgentCache *agent, CRadians angle);
  Real staticDistForAngle(AgentCache *agent, CRadians angle);
  Real distanceToCollisionAtRelativeAngle(CRadians angle);

  AgentCache makeObstacleAtPoint(CVector2 p, CVector2 v, Real r,
                                 Real socialMargin);
  AgentCache makeObstacleAtPoint(CVector2 p, Real r, Real socialMargin);

  agentList_t nearAgents;
  obstacleList_t staticObstacles;

  void updateVelocityCartesian();
  void debugAgents();
  void debugStaticObstacles();

  static const char * name;
};

#endif
