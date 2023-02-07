/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef _HL_H_
#define _HL_H_

#include <list>

#include "../behavior.h"

// TODO(J): verify if behavior for tau < step is correct (non smooth)

// trajectory planning

#define UNKNOWN_DIST -2
#define MAX_RESOLUTION 361
#define NO_COLLISION -1
// configuration

#define DEFAULT_APERTURE M_PI
#define DEFAULT_RESOLUTION 101
#define DEFAULT_TAU 0.1
#define MIN_TAU 0.1

namespace hl_navigation {

typedef struct {
  float centerDistance;
  float radius;
  float sensingMargin;
  float agentSensingMargin;

  // float socialMargin;
  // float penetration;
  Vector2 dx;
  Vector2 va;
  Vector2 position;
  float C;
  Radians gamma, visibleAngle;
} AgentCache;

typedef std::list<AgentCache> obstacleList_t;
typedef obstacleList_t::iterator obstacleIterator_t;
typedef std::list<AgentCache> agentList_t;
typedef agentList_t::iterator agentIterator_t;

class HLBehavior : public Behavior {

public:
  // Mobility
  // Navigation
  // virtual void Init(TConfigurationNode& t_tree);

  float *collisionMap();
  unsigned int resolution;

  HLBehavior(agent_type_t type, float radius, float axis_length=0.0) :
    Behavior(type, radius, axis_length),
    resolution(DEFAULT_RESOLUTION), aperture(DEFAULT_APERTURE), tau(DEFAULT_TAU),
    eta(tau), nearAgents(), staticObstacles() {}
  ~HLBehavior() {}


  std::vector<float> getDistances();

  void setTau(float value);
  void setEta(float value);
  void setAperture(float value);
  void setResolution(unsigned int value);

  Radians angleResolution();
  Radians aperture;

  virtual void update_target_twist(float dt) override;

  float distance_to_segment(const LineSegment & line, Radians absolute_angle);

protected:
  virtual void update_desired_velocity() override;
  virtual void update_repulsive_force() override;
  virtual void add_neighbor(const Disc & disc) override;
  virtual void add_static_obstacle(const Disc & disc) override;
  virtual void clear() override;
  virtual void prepare() override;

private:

  float effectiveHorizon;

  float tau;
  float eta;

  void prepareAgents();
  float distanceCache[MAX_RESOLUTION];
  float staticDistanceCache[MAX_RESOLUTION];
  void initDistanceCache();
  unsigned int indexOfRelativeAngle(Radians relativeAngle);
  float computeDistanceToCollisionAtRelativeAngle(Radians relativeAngle,
                                                 float *staticCache);
  float fearedDistanceToCollisionAtRelativeAngle(Radians angle);
  float distForAngle(AgentCache *agent, Radians angle);
  float staticDistForAngle(const AgentCache *agent, Radians angle);
  float distanceToCollisionAtRelativeAngle(Radians angle);


  AgentCache makeObstacleAtPoint(Vector2 p, Vector2 v, float r,
                                 float socialMargin);
  AgentCache makeObstacleAtPoint(Vector2 p, float r, float socialMargin);

  agentList_t nearAgents;
  obstacleList_t staticObstacles;

  // void updateVelocityCartesian();
  void debugNeigbors();
  void debugStaticObstacles();

  static const char * name;
};

}

#endif
