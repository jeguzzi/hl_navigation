/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "behaviors/HL.h"
#include <algorithm>

namespace hl_navigation {

void HLBehavior::setTau(float value) { tau = value; }
void HLBehavior::setEta(float value) { eta = value; }
void HLBehavior::setAperture(Radians value) { aperture = value; }

void HLBehavior::setResolution(unsigned int value) {
  resolution = std::min((int)value, MAX_RESOLUTION);
}

static float relax(float x0, float x1, float tau, float dt) {
  if (tau == 0)
    return x1;
  // float dt=0.1;
  return exp(-dt / tau) * (x0 - x1) + x1;
}

#if 0
static Vector2 relax(Vector2 x0, Vector2 x1, float tau, float dt) {
  if (tau == 0)
    return x1;
  // float dt=0.1;
  return exp(-dt / tau) * (x0 - x1) + x1;
}
#endif

static std::vector<float> relax(const std::vector<float> & v0, const std::vector<float> & v1,
                                float tau, float dt) {
  if (tau == 0)
    return v1;
  auto v2 = std::vector<float>(v0.size());
  for (size_t i = 0; i < v0.size(); i++) {
    v2[i] = relax(v0[i], v1[i], tau, dt);
  }
  return v2;
}

static Twist2D relax(const Twist2D & v0, const Twist2D & v1, float tau, float dt) {
  if (tau == 0)
    return v1;
  return Twist2D(
    relax(v0.longitudinal, v1.longitudinal, tau, dt),
    relax(v0.lateral, v1.lateral, tau, dt),
    relax(v0.angular, v1.angular, tau, dt));
}


static void setDx(AgentCache &agent, Vector2 p) {
  agent.dx = p;
  agent.centerDistance = p.norm();

  // visibleDistance=centerDistance-agentSensingMargin;
  // penetration=(socialMargin-centerDistance);

  agent.C = p.squaredNorm() - agent.sensingMargin * agent.sensingMargin;
  agent.gamma = polar_angle(-p);
  agent.visibleAngle = M_PI_2;
  // Radians alpha=ASin(agent.agentSensingMargin/agent.centerDistance);
  // agent.beta1=agent.gamma-alpha;
  // agent.beta2=agent.gamma+alpha;
}



Radians HLBehavior::angleResolution() { return 2 * aperture / resolution; }

void HLBehavior::debugNeigbors() {
  printf("--------------------- AGENTS -------------------\n");
  for (agentIterator_t it = nearAgents.begin(); it != nearAgents.end(); it++) {
    printf("dx=(%.2f,%.2f), va=(%.2f, %.2f), C=%.2f\r\n", it->dx.x(),
           it->dx.y(), it->va.x(), it->va.y(), it->C);
  }
  printf("--------------------- ****** -------------------\n");
}

void HLBehavior::initDistanceCache() {
  unsigned int k = 0;
  for (; k < resolution; k++)
    distanceCache[k] = UNKNOWN_DIST;
}

float HLBehavior::fearedDistanceToCollisionAtRelativeAngle(Radians relativeAngle) {
  if (abs(normalize(relativeAngle)) >= aperture)
    return UNKNOWN_DIST;
  int k = indexOfRelativeAngle(relativeAngle);
  return staticDistanceCache[k];
}

unsigned int HLBehavior::indexOfRelativeAngle(Radians relativeAngle) {
  int k = floor((normalize(relativeAngle) + aperture) / (2 * aperture) * resolution);

  k = k % resolution;
  if (k < 0)
    k += resolution;
  return k;
}

float HLBehavior::distanceToCollisionAtRelativeAngle(Radians relativeAngle) {
  if (abs(normalize(relativeAngle)) >= aperture)
    return UNKNOWN_DIST;
  int k = indexOfRelativeAngle(relativeAngle);
  if (distanceCache[k] < 0) {
    distanceCache[k] = computeDistanceToCollisionAtRelativeAngle(
        relativeAngle, staticDistanceCache + k);
  }
  return distanceCache[k];
}



float HLBehavior::distance_to_segment(const LineSegment & line, Radians absolute_angle) {
  Vector2 delta = position - line.p1;
  float r = radius + safetyMargin;
  float y = delta.dot(line.e2);
  float x = delta.dot(line.e1);
  Vector2 e = unit(absolute_angle);
  float d = line.e2.dot(e);
  if (y * d >= 0) {
    // moving away
    return NO_COLLISION;
  }
  if (abs(y) < r && x > -r && x < line.length + r) {
    // already colliding
    return 0.0;
  }
  float distance = -y / d - r;
  x = line.e1.dot(distance * e + delta);
  if (x < -r || x > line.length + r) {
    // will not collide
    return NO_COLLISION;
  }
  return distance;
}

// TODO(J:revision2023): check why we need effectiveHorizon

float HLBehavior::computeDistanceToCollisionAtRelativeAngle(Radians relativeAngle,
                                                        float *staticCache) {
  Radians vangle = relativeAngle + angle;

  //!!! Change horizon -> effectiveHorizon !!!

  // HACK(J): disabled effectiveHorizon to test function in python
  effectiveHorizon = horizon;
  float minDistance = effectiveHorizon - radius;

  ////
  float distance;
  float staticDistance;
  *staticCache = minDistance;

  for (auto & segment : line_obstacles) {
      staticDistance = distance_to_segment(segment, vangle);
      // printf("staticDistance %.4f\n", staticDistance);
      if (staticDistance >= 0) {
        *staticCache = fmin(*staticCache, staticDistance);
      }
      distance = staticDistance;
      if (distance < 0)
        continue;
      minDistance = fmin(minDistance, distance);
      if (minDistance == 0)
        return 0;
  }

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

std::vector<float> HLBehavior::getDistances() {
  std::vector<float> d;
  Radians a = -aperture;
  Radians da = 2 * aperture / (float)resolution;
  while (a < aperture) {
    d.push_back(distanceToCollisionAtRelativeAngle(a));
    a += da;
  }
  return d;
}

static float penetration(AgentCache *agent) {
  if (agent->C > 0) {
    return 0;
  } else {
    return agent->centerDistance - agent->sensingMargin;
  }
}

void HLBehavior::update_repulsive_force() {
  repulsiveForce = Vector2(0, 0);
  insideObstacle = false;

  for (obstacleIterator_t it = staticObstacles.begin();
       it != staticObstacles.end(); it++) {
    AgentCache *a = &(*it);
    float d = penetration(a);
    if (d) {
      repulsiveForce += d * unit(a->gamma);
      insideObstacle = true;
    }
  }
  for (agentIterator_t it = nearAgents.begin(); it != nearAgents.end(); it++) {
    AgentCache *a = &(*it);
    float d = penetration(a);
    if (d) {
      repulsiveForce += d * unit(a->gamma);
      insideObstacle = true;
    }
  }
}

void HLBehavior::update_desired_velocity() {
  // setup();

  // debugBehaviors();
  // debugStaticObstacles();

  Vector2 agentToTarget = targetPosition - position;
  Radians a0 = polar_angle(agentToTarget) - angle;
  Radians da = angleResolution();
  float D = agentToTarget.norm();
  effectiveHorizon = horizon;
  // effectiveHorizon=fmin(horizon,D);

  // Vector2 effectiveTarget = agentToTarget / D * effectiveHorizon;


  // printf("HLBehavior: updateDesiredVelocity to %.2f, h = %.2f
  // \r\n",a0.GetValue(),horizon);

  // initDistanceCache();

  Radians searchAngle = 0.0;

  float minPossibleDistanceToTarget;
  D = effectiveHorizon;
  float minDistanceToTarget = D;
  float d;
  float distanceToTarget;
  Radians nearestAngle = a0;

  int leftOut = 0;
  int rightOut = 0;

  Radians maxAngle(1.6);
  // Radians::PI_OVER_TW0;

  while (searchAngle < maxAngle && !(leftOut == 2 && rightOut == 2)) {
    // new in paper
    minPossibleDistanceToTarget = fabs(sin(searchAngle) * D);
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
      if (cos(searchAngle) * D < d) {
        distanceToTarget = fabs(sin(searchAngle) * D);
      } else {
        distanceToTarget = sqrt(D * D + d * d - 2 * d * D * cos(searchAngle));
      }
      // printf("%.2f < ? %.2f\r\n",distanceToTarget,minDistanceToTarget);
      // distanceToTarget=sqrt(D*D+d*d-2*d*D*Cos(searchAngle));
      if (distanceToTarget < minDistanceToTarget) {
        minDistanceToTarget = distanceToTarget;
        nearestAngle = a0 + searchAngle;
      }
    }
    if (searchAngle > 0.0) {
      d = distanceToCollisionAtRelativeAngle(a0 - searchAngle);

      //  printf("%.2f -> %.2f\r\n",-searchAngle.GetValue(),d);

      if (d == UNKNOWN_DIST && rightOut == 1)
        rightOut = 2;
      if (d != UNKNOWN_DIST && rightOut == 0)
        rightOut = 1;

      d = fmin(D, d);

      if (d != UNKNOWN_DIST) {
        if (cos(searchAngle) * D < d) {
          distanceToTarget = fabs(sin(searchAngle) * D);
        } else {
          distanceToTarget = sqrt(D * D + d * d - 2 * d * D * cos(searchAngle));
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

  float nearestCollision =
      fearedDistanceToCollisionAtRelativeAngle(nearestAngle);

  // printf("nearearest %.2f\n",nearestCollision);

  float newTargetSpeed;
  if (nearestCollision > 0) {
    newTargetSpeed = fmin(optimalSpeed, nearestCollision / eta);
  } else {
    newTargetSpeed = 0;
  }

  // desiredAngle = nearestAngle;
  // desiredSpeed = newTargetSpeed;
  // TODO(Jerome): verify that all desired velocities are in the fixed frame
  desiredVelocity = newTargetSpeed * unit(nearestAngle + angle);

  // DEBUG_CONTROLLER("=> desiredVelocity (%.2f,%.2f) %.2f
  // \n",desiredVelocity.x(),desiredVelocity.y(),desiredAngle.GetValue());

  //    DEBUG_CONTROLLER("=> desired speed %.2f, desired angle %.2f
  //    \n",desiredSpeed,desiredAngle.GetValue());
}

void HLBehavior::clear() {
  nearAgents.clear();
  staticObstacles.clear();
}


void HLBehavior::add_static_obstacle(const Disc & d) {
  AgentCache obstacle = makeObstacleAtPoint(d.position, d.radius, d.social_margin);
  staticObstacles.push_back(obstacle);
}

void HLBehavior::add_neighbor(const Disc & d) {
  // printf("Add obstacle at (%.2f %.2f) with v (%.2f %.2f) and r %.2f
  // \n",p.x(),p.y(),v.x(),v.y(),r);
  AgentCache agent = makeObstacleAtPoint(d.position, d.velocity, d.radius, d.social_margin);
  nearAgents.push_back(agent);
}

/*
void HLBehavior::addObstacleWithHuman(Human *human)
{
  AgentCache agent=makeAgentCacheWithHuman(human);
  nearAgents.push_back(agent);
  }*/

# if 0
static bool compare(AgentCache first, AgentCache second) {
  return first.centerDistance < second.centerDistance;
}
#endif

void HLBehavior::prepare() {
  initDistanceCache();
  effectiveHorizon = horizon;
}

AgentCache HLBehavior::makeObstacleAtPoint(Vector2 p, float r, float socialMargin) {
  AgentCache obstacle = AgentCache();
  obstacle.radius = r;
  float distance;
  Vector2 relativePosition = relativePositionOfObstacleAt(p, r, distance);
  obstacle.agentSensingMargin = r;
  obstacle.sensingMargin = safetyMargin + radius + r;
  obstacle.position = p;
  setDx(obstacle, -relativePosition);
  return obstacle;
}

/*
AgentCache HLBehavior::makeObstacleWithHuman(Human* human)
{
  //printf("Add human agent cache ");

   AgentCache agent=AgentCache();
    agent.radius=radius;

    Vector2 relativePosition=human->position-position;



    float distance=relativePosition.norm();
    float minDistance=radius+0.01+HUMAN_RADIUS;
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
[%.2f,%.2f,%.2f,%.2f,%.2f]\n",relativePosition.x(),relativePosition.y(),margin,agent.sensingMargin,distance,distanceToBeSeparated,distanceToBeFar,safetyMargin,socialMargin);

    setDx(agent,-relativePosition);
    agent.va=human->velocity;
    agent.position=human->position;
    return agent;
    }*/

AgentCache HLBehavior::makeObstacleAtPoint(Vector2 obstaclePosition,
                                        Vector2 obstacleVelocity,
                                        float obstacleRadius,
                                        float socialMargin) {
  /// TODO: eliminare radius (non lo uso)

  AgentCache agent = AgentCache();
  agent.radius = radius;
  float distance;
  Vector2 relativePosition =
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

void HLBehavior::debugStaticObstacles() {
  // if(!consoleDebugging) return;
  printf("---------------------------- Static Obstacles "
         "----------------------------\r\n");
  printf("staticObstacles=[");
  for (obstacleIterator_t it = staticObstacles.begin();
       it != staticObstacles.end(); it++) {
    printf("%.3f,%.3f,", it->position.x(), it->position.y());
  }
  printf("];\n");
  printf("---------------------------- **************** "
         "----------------------------\r\n");
}

float HLBehavior::distForAngle(AgentCache *agent, Radians alpha) {
  if (agent->C < 0) {
    if (abs(normalize(alpha - agent->gamma)) < agent->visibleAngle)
      return 0;
    return NO_COLLISION;
  }

  Vector2 dv = optimalSpeed * unit(alpha) - agent->va;
  float A = dv.squaredNorm();
  // TODO(J): use dot prod
  // TODO(J 2023): maybe precompute minimal distance
  // a = optimalSpeed + agent_speed
  // A = a * a
  // B = -dist * a
  // C = dist * dist - r * r
  // D = dist * dist * A - dist * dist * A + r * r * A = r * r * a * a
  // min_dist = optimalSpeed * (-B - sqrt(D)) / A = optimalSpeed * (dist * a - r * a) (a * a)
  //          = optimalSpeed * (dist - r) / (optimalSpeed + agent_speed)
  float B = agent->dx.x() * dv.x() + agent->dx.y() * dv.y();

  if (B > 0)
    return NO_COLLISION;
  float D = B * B - A * agent->C;

  if (D < 0)
    return NO_COLLISION;

  return optimalSpeed * (-B - sqrt(D)) / A;
}

float HLBehavior::staticDistForAngle(const AgentCache *agent, Radians alpha) {
  if (agent->C < 0) {
    if (abs(normalize(alpha - agent->gamma)) < agent->visibleAngle)
      return 0;
    return NO_COLLISION;
  }
  float B = agent->dx.x() * cos(alpha) + agent->dx.y() * sin(alpha);
  if (B > 0)
    return NO_COLLISION;
  float D = B * B - agent->C;
  if (D < 0)
    return NO_COLLISION;
  return -B - sqrt(D);
}

void HLBehavior::update_target_twist(float dt) {
  if (is_wheeled()) {
    target_wheel_speeds = relax(target_wheel_speeds, desired_wheel_speeds, tau, dt);
    target_twist = twist_from_wheel_speeds(target_wheel_speeds);
  } else {
    // TODO(Jerome): same than before when I relaxed the absolute velocity, not the relative
    // but different than original paper
    target_twist = relax(target_twist, desired_twist, tau, dt);
  }
}

float *HLBehavior::collisionMap() { return &distanceCache[0]; }


/*
  void HLBehavior::updateVelocityCartesian ()
  {
    //DEBUG_CONTROLLER("updateVelocityCartesian()\r\n");

#ifdef RELAXED_VELOCITY
    //Perhaps it makes more sense to do it in polar coordinates, i.e
speed->exp(...), angle->angle(...)

    desiredVelocity=relax(velocity,desiredVelocity,tau);
    //desiredAngle=relax(0,desiredAngle,tau);
#endif



    Radians delta;

    float v=fabs(desiredVelocity.norm()/0.05);

    if(v>1) delta=desiredVelocity.Angle();
    else
delta=(Vector2(v,desiredVelocity.Angle())+Vector2(1-v,desiredAngle)).Angle();

    delta=delta.SignedNormalize();
    float angularSpeed;
    float linearSpeed=0;

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
        linearSpeed=desiredVelocity.norm()*(1-fabs(angularSpeed)/maxRotationSpeed);
      }
    //DEBUG_CONTROLLER("Angular Speed %.2f, Linear Speed %.2f
[cm/s]\r\n",100*angularSpeed,100*linearSpeed);
    leftWheelDesiredSpeed=linearSpeed-angularSpeed;
    rightWheelDesiredSpeed=linearSpeed+angularSpeed;
  }
*/

const char * HLBehavior::name = register_type<HLBehavior>("HL");


}  // namespace hl_navigation