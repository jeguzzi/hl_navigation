/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef _HLAGENT_H_
#define _HLAGENT_H_

#include "Agent.h"

// TODO(J): verify if behavior for tau < step is correct (non smooth)

// trajectory planning

#define UNKNOWN_DIST -2
#define MAX_RESOLUTION 301
#define NO_COLLISION -1
// configuration

#define DEFAULT_APERTURE M_PI
#define DEFAULT_RESOLUTION 360
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
  // virtual void Init(TConfigurationNode& t_tree);

  Real *collisionMap();
  unsigned int resolution;

  HLAgent(agent_type_t type, float radius, float axis_length=0.0) :
    Agent(type, radius, axis_length),
    resolution(DEFAULT_RESOLUTION), aperture(DEFAULT_APERTURE), tau(DEFAULT_TAU),
    eta(tau) {}
  ~HLAgent() {}


  std::vector<Real> getDistances();

  void setTau(double value);
  void setEta(double value);
  void setAperture(double value);
  void setResolution(unsigned int value);

  CRadians angleResolution();
  CRadians aperture;

  virtual void update_target_twist(float dt) override;

  Real distance_to_segment(const LineSegment & line, CRadians absolute_angle);

protected:
  virtual void update_desired_velocity() override;
  virtual void update_repulsive_force() override;
  virtual void add_neighbor(const Disc & disc) override;
  virtual void add_static_obstacle(const Disc & disc) override;
  virtual void clear() override;
  virtual void prepare() override;

private:

  Real effectiveHorizon;

  Real tau;
  Real eta;

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

  // void updateVelocityCartesian();
  void debugAgents();
  void debugStaticObstacles();

  static const char * name;
};

#endif
