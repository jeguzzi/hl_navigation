/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "HLAgent.h"
#include <algorithm>


void HLAgent::setTau(double value) { tau = value; }
void HLAgent::setEta(double value) { eta = value; }
void HLAgent::setAperture(double value) { aperture = CRadians(value); }

void HLAgent::setResolution(unsigned int value) {
  resolution = std::min((int)value, MAX_RESOLUTION);
}

static Real relax(Real x0, Real x1, Real tau, Real dt) {
  if (tau == 0)
    return x1;
  // Real dt=0.1;
  return exp(-dt / tau) * (x0 - x1) + x1;
}

#if 0
static CVector2 relax(CVector2 x0, CVector2 x1, Real tau, Real dt) {
  if (tau == 0)
    return x1;
  // Real dt=0.1;
  return exp(-dt / tau) * (x0 - x1) + x1;
}
#endif

static std::vector<float> relax(const std::vector<float> & v0, const std::vector<float> & v1,
                                Real tau, Real dt) {
  if (tau == 0)
    return v1;
  auto v2 = std::vector<float>(v0.size());
  for (size_t i = 0; i < v0.size(); i++) {
    v2[i] = relax(v0[i], v1[i], tau, dt);
  }
  return v2;
}

static Twist2D relax(const Twist2D & v0, const Twist2D & v1, Real tau, Real dt) {
  if (tau == 0)
    return v1;
  return Twist2D(
    relax(v0.longitudinal, v1.longitudinal, tau, dt),
    relax(v0.lateral, v1.lateral, tau, dt),
    relax(v0.angular, v1.angular, tau, dt));
}


static void setDx(AgentCache &agent, CVector2 p) {
  agent.dx = p;
  agent.centerDistance = p.Length();

  // visibleDistance=centerDistance-agentSensingMargin;
  // penetration=(socialMargin-centerDistance);

  agent.C = p.SquareLength() - agent.sensingMargin * agent.sensingMargin;
  agent.gamma = (-p).Angle();
  agent.visibleAngle = CRadians::PI_OVER_TWO;
  // CRadians alpha=ASin(agent.agentSensingMargin/agent.centerDistance);
  // agent.beta1=agent.gamma-alpha;
  // agent.beta2=agent.gamma+alpha;
}



CRadians HLAgent::angleResolution() { return 2 * aperture / resolution; }

void HLAgent::debugAgents() {
  printf("--------------------- AGENTS -------------------\n");
  for (agentIterator_t it = nearAgents.begin(); it != nearAgents.end(); it++) {
    printf("dx=(%.2f,%.2f), va=(%.2f, %.2f), C=%.2f\r\n", it->dx.GetX(),
           it->dx.GetY(), it->va.GetX(), it->va.GetY(), it->C);
  }
  printf("--------------------- ****** -------------------\n");
}

void HLAgent::initDistanceCache() {
  unsigned int k = 0;
  for (; k < resolution; k++)
    distanceCache[k] = UNKNOWN_DIST;
}

Real HLAgent::fearedDistanceToCollisionAtRelativeAngle(CRadians relativeAngle) {
  if (relativeAngle.SignedNormalize().GetAbsoluteValue() >= aperture.GetValue())
    return UNKNOWN_DIST;
  int k = indexOfRelativeAngle(relativeAngle);
  return staticDistanceCache[k];
}

unsigned int HLAgent::indexOfRelativeAngle(CRadians relativeAngle) {
  int k = floor((relativeAngle + aperture) / (2 * aperture) * resolution);

  k = k % resolution;
  if (k < 0)
    k += resolution;
  return k;
}

Real HLAgent::distanceToCollisionAtRelativeAngle(CRadians relativeAngle) {
  if (relativeAngle.SignedNormalize().GetAbsoluteValue() >= aperture.GetValue())
    return UNKNOWN_DIST;
  int k = indexOfRelativeAngle(relativeAngle);
  if (distanceCache[k] < 0) {
    distanceCache[k] = computeDistanceToCollisionAtRelativeAngle(
        relativeAngle, staticDistanceCache + k);
  }
  return distanceCache[k];
}

Real HLAgent::computeDistanceToCollisionAtRelativeAngle(CRadians relativeAngle,
                                                        Real *staticCache) {
  CRadians vangle = relativeAngle + angle;

  //!!! Change horizon -> effectiveHorizon !!!

  Real minDistance = effectiveHorizon - radius;

  ////
  Real distance;
  *staticCache = 0;

  *staticCache = minDistance;
  double staticDistance;

  for (obstacleIterator_t it = staticObstacles.begin();
       it != staticObstacles.end(); it++) {
    staticDistance = staticDistForAngle(&(*it), vangle);
    if (!(staticDistance < 0))
      *staticCache = fmin(*staticCache, staticDistance);

    distance = staticDistance;

    if (distance < 0)
      continue;
    minDistance = fmin(minDistance, distance);

    if (minDistance == 0)
      return 0;
  }

  for (agentIterator_t it = nearAgents.begin(); it != nearAgents.end(); it++) {
    staticDistance = staticDistForAngle(&(*it), vangle);
    if (!(staticDistance < 0))
      *staticCache = fmin(*staticCache, staticDistance);

    distance = distForAngle(&(*it), vangle);

    if (distance < 0)
      continue;
    minDistance = fmin(minDistance, distance);

    if (minDistance == 0)
      return 0;
  }

  return minDistance;
}

std::vector<Real> HLAgent::getDistances() {
  std::vector<Real> d;
  CRadians a = -aperture;
  CRadians da = 2 * aperture / (Real)resolution;
  while (a < aperture) {
    d.push_back(distanceToCollisionAtRelativeAngle(a));
    a += da;
  }
  return d;
}

static Real penetration(AgentCache *agent) {
  if (agent->C > 0) {
    return 0;
  } else {
    return agent->centerDistance - agent->sensingMargin;
  }
}

void HLAgent::update_repulsive_force() {
  repulsiveForce = CVector2(0, 0);
  insideObstacle = false;

  for (obstacleIterator_t it = staticObstacles.begin();
       it != staticObstacles.end(); it++) {
    AgentCache *a = &(*it);
    double d = penetration(a);
    if (d) {
      repulsiveForce += CVector2(d, a->gamma);
      insideObstacle = true;
    }
  }
  for (agentIterator_t it = nearAgents.begin(); it != nearAgents.end(); it++) {
    AgentCache *a = &(*it);
    double d = penetration(a);
    if (d) {
      repulsiveForce += CVector2(d, a->gamma);
      insideObstacle = true;
    }
  }
}

void HLAgent::update_desired_velocity() {
  // setup();

  // debugAgents();
  // debugStaticObstacles();

  CVector2 agentToTarget = targetPosition - position;
  CRadians a0 = agentToTarget.Angle() - angle;
  CRadians da = angleResolution();
  Real D = agentToTarget.Length();
  effectiveHorizon = horizon;
  // effectiveHorizon=fmin(horizon,D);

  // CVector2 effectiveTarget = agentToTarget / D * effectiveHorizon;


  // printf("HLAgent: updateDesiredVelocity to %.2f, h = %.2f
  // \r\n",a0.GetValue(),horizon);

  // initDistanceCache();

  CRadians searchAngle = CRadians::ZERO;

  Real minPossibleDistanceToTarget;
  D = effectiveHorizon;
  Real minDistanceToTarget = D;
  Real d;
  Real distanceToTarget;
  CRadians nearestAngle = a0;

  int leftOut = 0;
  int rightOut = 0;

  CRadians maxAngle(1.6);
  // CRadians::PI_OVER_TW0;

  while (searchAngle < maxAngle && !(leftOut == 2 && rightOut == 2)) {
    // new in paper
    minPossibleDistanceToTarget = fabs(Sin(searchAngle) * D);
    // minPossibleDistanceToTarget=2*D*Sin(0.5*searchAngle);
    if (minDistanceToTarget < minPossibleDistanceToTarget)
      break;
    d = distanceToCollisionAtRelativeAngle(a0 + searchAngle);

    // printf("%.2f -> %.2f\r\n",searchAngle.GetValue(),d);

    if (d == UNKNOWN_DIST && leftOut == 1)
      leftOut = 2;
    if (d != UNKNOWN_DIST && leftOut == 0)
      leftOut = 1;

    d = fmin(D, d);

    if (d != UNKNOWN_DIST) {
      if (Cos(searchAngle) * D < d) {
        distanceToTarget = fabs(Sin(searchAngle) * D);
      } else {
        distanceToTarget = sqrt(D * D + d * d - 2 * d * D * Cos(searchAngle));
      }
      // printf("%.2f < ? %.2f\r\n",distanceToTarget,minDistanceToTarget);
      // distanceToTarget=sqrt(D*D+d*d-2*d*D*Cos(searchAngle));
      if (distanceToTarget < minDistanceToTarget) {
        minDistanceToTarget = distanceToTarget;
        nearestAngle = a0 + searchAngle;
      }
    }
    if (searchAngle > CRadians::ZERO) {
      d = distanceToCollisionAtRelativeAngle(a0 - searchAngle);

      //  printf("%.2f -> %.2f\r\n",-searchAngle.GetValue(),d);

      if (d == UNKNOWN_DIST && rightOut == 1)
        rightOut = 2;
      if (d != UNKNOWN_DIST && rightOut == 0)
        rightOut = 1;

      d = fmin(D, d);

      if (d != UNKNOWN_DIST) {
        if (Cos(searchAngle) * D < d) {
          distanceToTarget = fabs(Sin(searchAngle) * D);
        } else {
          distanceToTarget = sqrt(D * D + d * d - 2 * d * D * Cos(searchAngle));
        }

        // printf("%.2f < ? %.2f\r\n",distanceToTarget,minDistanceToTarget);

        // distanceToTarget=sqrt(D*D+d*d-2*d*D*Cos(searchAngle));
        if (distanceToTarget < minDistanceToTarget) {
          minDistanceToTarget = distanceToTarget;
          nearestAngle = a0 - searchAngle;
        }
      }
    }
    searchAngle += da;
  }

  Real nearestCollision =
      fearedDistanceToCollisionAtRelativeAngle(nearestAngle);

  // printf("nearearest %.2f\n",nearestCollision);

  double newTargetSpeed;
  if (nearestCollision > 0) {
    newTargetSpeed = fmin(optimalSpeed, nearestCollision / eta);
  } else {
    newTargetSpeed = 0;
  }

  // desiredAngle = nearestAngle;
  // desiredSpeed = newTargetSpeed;
  // TODO(Jerome): verify that all desired velocities are in the fixed frame
  desiredVelocity = CVector2(newTargetSpeed, nearestAngle + angle);

  // DEBUG_CONTROLLER("=> desiredVelocity (%.2f,%.2f) %.2f
  // \n",desiredVelocity.GetX(),desiredVelocity.GetY(),desiredAngle.GetValue());

  //    DEBUG_CONTROLLER("=> desired speed %.2f, desired angle %.2f
  //    \n",desiredSpeed,desiredAngle.GetValue());
}

void HLAgent::clear() {
  nearAgents.clear();
  staticObstacles.clear();
}


void HLAgent::add_static_obstacle(const Disc & d) {
  AgentCache obstacle = makeObstacleAtPoint(d.position, d.radius, d.social_margin);
  staticObstacles.push_back(obstacle);
}

void HLAgent::add_neighbor(const Disc & d) {
  // printf("Add obstacle at (%.2f %.2f) with v (%.2f %.2f) and r %.2f
  // \n",p.GetX(),p.GetY(),v.GetX(),v.GetY(),r);
  AgentCache agent = makeObstacleAtPoint(d.position, d.velocity, d.radius, d.social_margin);
  nearAgents.push_back(agent);
}

/*
void HLAgent::addObstacleWithHuman(Human *human)
{
  AgentCache agent=makeAgentCacheWithHuman(human);
  nearAgents.push_back(agent);
  }*/

# if 0
static bool compare(AgentCache first, AgentCache second) {
  return first.centerDistance < second.centerDistance;
}
#endif

void HLAgent::prepare() {
  initDistanceCache();
  effectiveHorizon = horizon;
}

AgentCache HLAgent::makeObstacleAtPoint(CVector2 p, Real r, Real socialMargin) {
  AgentCache obstacle = AgentCache();
  obstacle.radius = r;
  Real distance;
  CVector2 relativePosition = relativePositionOfObstacleAt(p, r, distance);
  obstacle.agentSensingMargin = r;
  obstacle.sensingMargin = safetyMargin + radius + r;
  obstacle.position = p;
  setDx(obstacle, -relativePosition);
  return obstacle;
}

/*
AgentCache HLAgent::makeObstacleWithHuman(Human* human)
{
  //printf("Add human agent cache ");

   AgentCache agent=AgentCache();
    agent.radius=radius;

    CVector2 relativePosition=human->position-position;



    Real distance=relativePosition.Length();
    Real minDistance=radius+0.01+HUMAN_RADIUS;
    if(distance<minDistance)
      {
        relativePosition=relativePosition/distance*minDistance;
        distance=minDistance;
      }

    double farMargin=2;
    double margin;
    double distanceToBeSeparated=safetyMargin+radius+HUMAN_RADIUS;
    double distanceToBeFar=farMargin+radius+HUMAN_RADIUS;



    if(distance<distanceToBeSeparated)
    {
        margin=safetyMargin;
        //obstacle.visibleAngle=PI/2;
    }
    else if(distance>distanceToBeFar)
    {
      margin=socialMargin;
    }
    else
    {
        margin=(socialMargin-safetyMargin)/(distanceToBeFar -
distanceToBeSeparated ) * (distance - distanceToBeSeparated)+ safetyMargin;
    }



    agent.sensingMargin=HUMAN_RADIUS+radius+margin;

    //printf("at (%.2f,%.2f), margin %.2f, minDist %.2f,
[%.2f,%.2f,%.2f,%.2f,%.2f]\n",relativePosition.GetX(),relativePosition.GetY(),margin,agent.sensingMargin,distance,distanceToBeSeparated,distanceToBeFar,safetyMargin,socialMargin);

    setDx(agent,-relativePosition);
    agent.va=human->velocity;
    agent.position=human->position;
    return agent;
    }*/

AgentCache HLAgent::makeObstacleAtPoint(CVector2 obstaclePosition,
                                        CVector2 obstacleVelocity,
                                        Real obstacleRadius,
                                        Real socialMargin) {
  /// TODO: eliminare radius (non lo uso)

  AgentCache agent = AgentCache();
  agent.radius = radius;
  Real distance;
  CVector2 relativePosition =
      relativePositionOfObstacleAt(obstaclePosition, obstacleRadius, distance);
  agent.sensingMargin = (radius + obstacleRadius) +
                        marginForObstacleAtDistance(distance, obstacleRadius,
                                                    safetyMargin, socialMargin);

  // printf("X %.2f, r %.3f or %.3f, sm %.3f Sm %.3f
  // \n",agent.sensingMargin,radius,obstacleRadius,safetyMargin,socialMargin);

  agent.va = obstacleVelocity;
  setDx(agent, -relativePosition);
  agent.position = obstaclePosition;
  return agent;
}

void HLAgent::debugStaticObstacles() {
  // if(!consoleDebugging) return;
  printf("---------------------------- Static Obstacles "
         "----------------------------\r\n");
  printf("staticObstacles=[");
  for (obstacleIterator_t it = staticObstacles.begin();
       it != staticObstacles.end(); it++) {
    printf("%.3f,%.3f,", it->position.GetX(), it->position.GetY());
  }
  printf("];\n");
  printf("---------------------------- **************** "
         "----------------------------\r\n");
}

Real HLAgent::distForAngle(AgentCache *agent, CRadians alpha) {
  if (agent->C < 0) {
    if ((alpha - agent->gamma).SignedNormalize().GetAbsoluteValue() <
        agent->visibleAngle.GetValue())
      return 0;
    return NO_COLLISION;
  }

  CVector2 dv = CVector2(optimalSpeed, alpha) - agent->va;
  Real A = dv.SquareLength();
  Real B = agent->dx.GetX() * dv.GetX() + agent->dx.GetY() * dv.GetY();

  if (B > 0)
    return NO_COLLISION;
  Real D = B * B - A * agent->C;

  if (D < 0)
    return NO_COLLISION;

  return optimalSpeed * (-B - sqrt(D)) / A;
}

Real HLAgent::staticDistForAngle(AgentCache *agent, CRadians alpha) {
  if (agent->C < 0) {
    if ((alpha - agent->gamma).SignedNormalize().GetAbsoluteValue() <
        agent->visibleAngle.GetAbsoluteValue())
      return 0;
    return NO_COLLISION;
  }
  Real B = agent->dx.GetX() * Cos(alpha) + agent->dx.GetY() * Sin(alpha);
  if (B > 0)
    return NO_COLLISION;
  Real D = B * B - agent->C;
  if (D < 0)
    return NO_COLLISION;
  return -B - sqrt(D);
}

void HLAgent::update_target_twist(float dt) {
  if (is_wheeled()) {
    target_wheel_speeds = relax(target_wheel_speeds, desired_wheel_speeds, tau, dt);
    target_twist = twist_from_wheel_speeds(target_wheel_speeds);
  } else {
    // TODO(Jerome): same than before when I relaxed the absolute velocity, not the relative
    // but different than original paper
    target_twist = relax(target_twist, desired_twist, tau, dt);
  }
}

Real *HLAgent::collisionMap() { return &distanceCache[0]; }


/*
  void HLAgent::updateVelocityCartesian ()
  {
    //DEBUG_CONTROLLER("updateVelocityCartesian()\r\n");

#ifdef RELAXED_VELOCITY
    //Perhaps it makes more sense to do it in polar coordinates, i.e
speed->exp(...), angle->angle(...)

    desiredVelocity=relax(velocity,desiredVelocity,tau);
    //desiredAngle=relax(0,desiredAngle,tau);
#endif



    CRadians delta;

    Real v=fabs(desiredVelocity.Length()/0.05);

    if(v>1) delta=desiredVelocity.Angle();
    else
delta=(CVector2(v,desiredVelocity.Angle())+CVector2(1-v,desiredAngle)).Angle();

    delta=delta.SignedNormalize();
    Real angularSpeed;
    Real linearSpeed=0;

    angularSpeed=1.0/rotationTau*delta.SignedNormalize().GetValue()*0.5*axisLength;
    if(angularSpeed>maxRotationSpeed)
      {
        angularSpeed=maxRotationSpeed;
      }
    else if(angularSpeed<-maxRotationSpeed)
      {
        angularSpeed=-maxRotationSpeed;
      }
    else
      {
        linearSpeed=desiredVelocity.Length()*(1-fabs(angularSpeed)/maxRotationSpeed);
      }
    //DEBUG_CONTROLLER("Angular Speed %.2f, Linear Speed %.2f
[cm/s]\r\n",100*angularSpeed,100*linearSpeed);
    leftWheelDesiredSpeed=linearSpeed-angularSpeed;
    rightWheelDesiredSpeed=linearSpeed+angularSpeed;
  }
*/

const char * HLAgent::name = register_type<HLAgent>("HL");
