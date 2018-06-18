/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "Navigator.h"
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

// TF Utilities

template<typename T>
void transform(const tf::TransformListener &listener, const std::string &frame, const T &in, T &out)
{
}

template<>
void transform<geometry_msgs::PoseStamped>(const tf::TransformListener &listener, const std::string &frame, const geometry_msgs::PoseStamped &in, geometry_msgs::PoseStamped &out)
{
        listener.transformPose(frame, in, out);
}

template<>
void transform<geometry_msgs::PointStamped>(const tf::TransformListener &listener, const std::string &frame, const geometry_msgs::PointStamped &in, geometry_msgs::PointStamped &out)
{
        listener.transformPoint(frame, in, out);
}

template<>
void transform<geometry_msgs::Vector3Stamped>(const tf::TransformListener &listener, const std::string &frame, const geometry_msgs::Vector3Stamped &in, geometry_msgs::Vector3Stamped &out)
{
        listener.transformVector(frame, in, out);
}

template <typename T>
bool Navigator::inOdomFrame(const T &in, T &out)
{
        try{
                transform<T>(listener, odom_frame, in, out);
        }
        catch(tf::TransformException& ex) {
                ROS_ERROR("Received an exception trying to transform a pose from %s to %s: %s", in.header.frame_id.c_str(), odom_frame.c_str(), ex.what());
                return false;
        }
        return true;
}


// ROS Subscriptions


void Navigator::setStopFromMessage(const std_msgs::Empty& msg)
{
   ROS_INFO("Got stop msg");
        stop();
}

void Navigator::setTargetFromPointMessage(const geometry_msgs::PointStamped& msg)
{
        setTargetFromPoint(msg);
}

bool Navigator::setTargetFromPoint(const geometry_msgs::PointStamped& msg)
{
        if(odom_frame=="") return false;
        geometry_msgs::PointStamped targetPoint;
        if(!inOdomFrame(msg, targetPoint)) return false;
        targetType = POINT;
        agent->targetPosition = CVector2(targetPoint.point.x,targetPoint.point.y);
        targetZ = targetPoint.point.z;
        state = MOVE;
        if(drawing_enabled) drawTarget(targetPoint.point, targetPoint.header.frame_id);
        return true;
}

void Navigator::setTargetFromPoseMessage(const geometry_msgs::PoseStamped& msg)
{
        setTargetFromPose(msg);
}

bool Navigator::setTargetFromPose(const geometry_msgs::PoseStamped& msg)
{
        if(odom_frame== "") return false;
        geometry_msgs::PoseStamped targetPose;
        if(!inOdomFrame(msg, targetPose)) return false;
        targetType = POSE;
        agent->targetPosition = CVector2(targetPose.pose.position.x,targetPose.pose.position.y);
        targetZ = targetPose.pose.position.z;
        targetAngle = tf::getYaw(targetPose.pose.orientation);
        agent->desiredAngle = CRadians(targetAngle)-agent->angle;
        state = MOVE;
        if(drawing_enabled) drawTarget(targetPose.pose.position, targetPose.header.frame_id);
        return true;
}


void Navigator::setObstaclesFromMessage(const hl_navigation_msgs::Obstacles& obstacles_msg)
{
        if(odom_frame=="") return;
        agent->clearObstacles();
        for(auto msg : obstacles_msg.obstacles)
        {
                geometry_msgs::PointStamped topPosition;
                geometry_msgs::Vector3Stamped velocity;
                if(!(inOdomFrame(msg.topPosition, topPosition) && inOdomFrame(msg.velocity, velocity))) continue;
                double minZ=topPosition.point.z - msg.height;
                double maxZ=topPosition.point.z;
                bool relevant=false;
                //Consider the cylindric obstacles iff it intersects the strip z-targetZ
                if(!((z<minZ && targetZ<minZ) || (z>maxZ && targetZ>maxZ)))
                {
                        relevant = true;
                        CVector2 p(topPosition.point.x, topPosition.point.y);
                        CVector2 v(velocity.vector.x, velocity.vector.y);
                        agent->addObstacleAtPoint(p, v, msg.radius, msg.socialMargin);
                }
                if(drawing_enabled)
                {

                        drawObstacle(topPosition,msg.height,msg.radius,msg.socialMargin,relevant);
                        drawObstacleVelocity(topPosition,velocity.vector,relevant);
                }
        }
}

void Navigator::setOdometryFromMessage(const nav_msgs::Odometry& msg)
{
        lastTimeWhenLocalized=ros::Time::now();
        odom_frame=msg.header.frame_id;
        geometry_msgs::Pose pose = msg.pose.pose;
        geometry_msgs::Twist twist = msg.twist.twist;
        // HACK: We assume that twist and pose are in the same frame!
        yaw=tf::getYaw(pose.orientation);
        z=pose.position.z;
        agent->position=CVector2(pose.position.x,pose.position.y);
        agent->angle=CRadians(yaw);
        agent->velocity=CVector2(twist.linear.x,twist.linear.y);
        agent->angularSpeed=CRadians(twist.angular.z);
}

// ROS Parameters

void Navigator::setAgentFromString(std::string behaviorName)
{
        if(behaviorName=="HL")
        {
                agent=&hlAgent;
        }
        else if(behaviorName=="ORCA")
        {
                agent=&orcaAgent;
                orcaAgent.useEffectiveCenter=false;
        }
        else if(behaviorName=="ORCA-NH")
        {
                agent=&orcaAgent;
                orcaAgent.useEffectiveCenter=true;
        }
        else if(behaviorName=="HRVO")
        {
                agent=&hrvoAgent;
        }
        else
        {
                agent=&hlAgent;
        }
}

void Navigator::readStaticParameters()
{
        double value;
        std::string agentType;
        pn.param("axis_length", value, 1.0);
        pn.param("type", agentType, std::string("TWO_WHEELED"));

        for(auto agent : agents) {
                agent->axisLength=value;
                if(agentType=="TWO_WHEELED") agent->type=TWO_WHEELED;
                else if (agentType=="HOLONOMIC") agent->type=HOLONOMIC;
                else if (agentType=="HEAD") agent->type=HEAD;
                else agent->type=TWO_WHEELED;
                pn.param("radius", value, 0.3);
                agent->radius=value;
                if(pn.getParam("maximal_speed", value)) agent->setMaxSpeed(value);
                if(pn.getParam("maximal_angular_speed",value))
                {
                        agent->setMaxAngularSpeed(value);
                }
                else if(agentType=="TWO_WHEELED")
                {
                        if(pn.getParam("maximal_rotation_speed",value))
                        {
                                agent->setMaxRotationSpeed(value);
                        }
                        else
                        {
                                agent->setMaxRotationSpeed(agent->maxSpeed);
                        }
                }
        }
        //   pn.param("maximal_vertical_speed",maximalVerticalSpeed, 1.0);

}

void Navigator::readDynamicParameters(hl_navigation::NavigationConfig &config, uint32_t level)
{
        setAgentFromString(config.behavior);
        agent->setOptimalSpeed(config.optimal_speed);
        /// TODO: distinguish between these two that are alternative in the old code
        agent->setOptimalAngularSpeed(config.optimal_angular_speed);
        agent->setOptimalRotationSpeed(config.optimal_rotation_speed);
        ///
        tauZ = config.tau_z;
        optimalVerticalSpeed=config.optimal_vertical_speed;
        agent->setTau(config.tau);
        agent->setEta(config.eta);
        agent->setRotationTau(config.rotation_tau);
        agent->setHorizon(config.horizon);
        agent->setTimeHorizon(config.time_horizon);
        agent->setSafetyMargin(config.safety_margin);
        agent->setAperture(config.aperture);
        agent->setResolution(config.resolution);
        drawing_enabled = config.drawing;
        minDeltaDistance = config.tol_distance;
        minDeltaAngle = config.tol_angle;
        rotateIfHolo = config.point_toward_target;
        minimalSpeed = config.minimal_speed;
}


// State transitions

void Navigator::turn()
{
        state = TURN;
        agent->desiredSpeed=0.0;
}

void Navigator::brake()
{
   ROS_INFO("Asked to break %d", state);
   if(state!=BRAKING)
      {
        ROS_INFO("Will break");
        state=BRAKING;
     }
}

void Navigator::stop()
{
   ROS_INFO("Asked to stop %d %d", state, as_.isActive());
      if(state!=IDLE)
      {
        ROS_INFO("Will break");
        state=IDLE;
        setMotorVelocity(0,0,0);
        if(as_.isActive())
        {
                ROS_INFO("Will abort");
                hl_navigation_msgs::GoToTargetResult r;
                as_.setAborted(r);
        }
     }
}

bool Navigator::is_at_target_point()
{

        targetDistance = (agent->targetPosition - agent->position).Length();
        //ROS_INFO("is_at_target_point? %.2f %.2f", targetDistance, minDeltaDistance);
        return targetDistance < minDeltaDistance;
}

bool Navigator::is_at_target_angle()
{
        if(targetType==POINT) return true;
        if(state==MOVE)
        {
                return (agent->desiredAngle).GetAbsoluteValue() < minDeltaAngle;
        }
        else
        {
                double da=(CRadians(targetAngle)-agent->angle).GetAbsoluteValue();
                return da < minDeltaAngle;
        }
}

void Navigator::updateTargetState()
{
        switch(state)
        {
        case MOVE:
                if(is_at_target_point())
                {
                        if(agent->type == HOLONOMIC)
                        {
                                if(is_at_target_angle())
                                {
                                        if(as_.isActive())
                                        {
                                                ROS_INFO("Set goal reached");
                                                hl_navigation_msgs::GoToTargetResult r;
                                                as_.setSucceeded(r);
                                        }
                                        brake();
                                }
                        }
                        else
                        {
                                turn();
                        }
                }
                else
                {
                        if(as_.isActive())
                        {
                                hl_navigation_msgs::GoToTargetFeedback f;
                                f.distance = targetDistance;
                                as_.publishFeedback(f);
                        }
                }
                break;
        case TURN:
                if(is_at_target_angle())
                {
                        if(as_.isActive())
                        {
                                ROS_INFO("Set goal reached");
                                hl_navigation_msgs::GoToTargetResult r;
                                as_.setSucceeded(r);
                        }
                        brake();
                }
                break;
        default:
                break;
        }
        //ROS_DEBUG("Nav state: %d",state);
}


// Run loop

void Navigator::updateLocalization(ros::Time now)
{
        if(odom_frame=="")
        {
                localized = false;
                return;
        }
        double dt = (now - lastTimeWhenLocalized).toSec();
        localized = dt < 1;
}

void Navigator::updateVerticalVelocity()
{
        //TODO complete with obstacle avoidance
        double desiredVerticalSpeed=(targetZ-z)/tauZ;
        if(desiredVerticalSpeed>optimalVerticalSpeed) desiredVerticalSpeed=optimalVerticalSpeed;
        else if(desiredVerticalSpeed<-optimalVerticalSpeed) desiredVerticalSpeed=-optimalVerticalSpeed;
        velocityZ=velocityZ+(desiredVerticalSpeed-velocityZ)/tauZ;
}

void Navigator::setMotorVelocity(double newVelocityX,double newVelocityY,double newVelocityZ, double newAngularSpeed)
{
        // Desired velocity is in robot frame
        geometry_msgs::Twist base_cmd;
        base_cmd.linear.x=newVelocityX;
        base_cmd.linear.y=newVelocityY;
        base_cmd.linear.z=newVelocityZ;
        base_cmd.angular.z=newAngularSpeed;
        navPublisher.publish(base_cmd);
        if(!publishCmdStamped)
        {
                geometry_msgs::TwistStamped msg;
                msg.header.frame_id = odom_frame;
                msg.header.stamp = ros::Time::now();
                msg.twist.angular.z=newAngularSpeed;
                CVector2 v = CVector2(newVelocityX, newVelocityY);
                v.Rotate(agent->angle);
                msg.twist.linear.x = v.GetX();
                msg.twist.linear.y = v.GetY();
                msg.twist.linear.z = newVelocityZ;
                cmdStampedPublisher.publish(msg);
        }
}

void Navigator::setMotorVelocity(double newSpeed,double newVelocityZ,double newAngularSpeed)
{
        setMotorVelocity(newSpeed, 0, newVelocityZ, newAngularSpeed);
}

void Navigator::update(const ros::TimerEvent& evt)
{
        updateLocalization(evt.current_real);
        if(!localized && state!=IDLE)
        {
                ROS_WARN("NOT localized -> break");
                brake();
                return;
        }
        if(state==IDLE) return;

        updateTargetState();
        if(state==BRAKING)
        {
           if(agent->velocity.Length() > minimalSpeed)
           {
             agent->desiredSpeed=0.0;
            agent->desiredVelocity=CVector2(0,0);
            agent->desiredAngle=CRadians(targetAngle);
          }
          else
          {
             stop();
             return;
          }
        }
        if(state==MOVE)
        {
                agent->updateDesiredVelocity();
                agent->updateRepulsiveForce();
                if(drawing_enabled)
                {
                        drawDesiredVelocity(agent->desiredSpeed, agent->desiredAngle.GetValue());
                        if(agent==&hlAgent) drawCollisionMap();
                }
        }
        if(state==TURN)
        {
                agent->desiredSpeed=0.0;
                agent->desiredVelocity=CVector2(0,0);
                agent->desiredAngle=CRadians(targetAngle)-agent->angle;
        }

        //   if(agent->type!=TWO_WHEELED && agent->insideObstacle)
        //   {
        //           //TODO move away from penetrated obstacles, non so easy to avoid livelocks. This is far more relevant in 2.5 D because top/bottom margin of obstacles can be penetrated more easely.
        //           exit_from_obstacle()
        //   }

        agent->updateVelocity();
        updateVerticalVelocity();
        if(agent->type==TWO_WHEELED || agent->type==HEAD)
        {
                setMotorVelocity(agent->desiredLinearSpeed,velocityZ,(agent->desiredAngularSpeed).GetValue());
        }
        else if(agent->type==HOLONOMIC)
        {
           CRadians desiredAngularSpeed;
                if(rotateIfHolo)
                {
                        CRadians delta, tAngle;
                        if(targetType == POSE)
                        {
                                tAngle = CRadians(targetAngle);
                        }
                        else
                        {
                                CVector2 agentToTarget=agent->targetPosition-agent->position;
                                tAngle=agentToTarget.Angle();
                        }
                        delta=(tAngle-agent->angle).SignedNormalize();
                        desiredAngularSpeed=(1.0/agent->rotationTau)*delta;
                }
                else
                {
                        desiredAngularSpeed=CRadians(0);
                }
                //setMotorVelocity(agent->desiredVelocity.GetX(),agent->desiredVelocity.GetY(),velocityZ,0);
                setMotorVelocity(agent->desiredVelocity.GetX(),agent->desiredVelocity.GetY(),velocityZ,desiredAngularSpeed.GetValue());
        }
}


void Navigator::goToTarget()
{
        hl_navigation_msgs::GoToTargetGoalConstPtr goal = as_.acceptNewGoal();
        // TODO: check the outcome.
        bool valid;
        if(goal->target_pose.header.frame_id != "")
        {
                valid = setTargetFromPose(goal->target_pose);
        }
        else
        {
                valid = setTargetFromPoint(goal->target_point);
        }
        if(!valid)
        {
                hl_navigation_msgs::GoToTargetResult r;
                const std::string msg = "Goal not valid";
                as_.setAborted(r, msg);
        }
}

void Navigator::preemptTarget()
{
        ROS_WARN("Preempted target, will idle");
        as_.setPreempted();
        brake();
}

// Constructor

Navigator::Navigator() : pn("~"), as_(n, "go_to_target", false)
{
        state = IDLE;
        navPublisher=n.advertise<geometry_msgs::Twist>("cmd_vel",1);
        ros::param::param("~publish_cmd_stamped", publishCmdStamped, false);
        cmdStampedPublisher = n.advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped",1);
        stopSubscriber = n.subscribe("stop", 1, &Navigator::setStopFromMessage,this);
        targetPoseSubscriber = n.subscribe("target_pose", 1, &Navigator::setTargetFromPoseMessage, this);
        targetPointSubscriber = n.subscribe("target_point", 1, &Navigator::setTargetFromPointMessage, this);
        obstaclesSubscriber = n.subscribe("obstacles", 1, &Navigator::setObstaclesFromMessage, this);
        odom_frame="";
        odometrySubscriber=n.subscribe("odom",1,&Navigator::setOdometryFromMessage,this);
        ns=n.resolveName("");
        if(ns.length()>1)
        {
                ns=ns.substr(2,std::string::npos);
        }
        else
        {
                ns="";
        }

        ROS_INFO("ns is %s",ns.c_str());
        initDrawing();

        agents.push_back(&orcaAgent);
        agents.push_back(&hrvoAgent);
        agents.push_back(&hlAgent);

        readStaticParameters();

        double rate;
        ros::param::param("~rate",rate,10.0);
        updatePeriod = 1.0/rate;
        hlAgent.dt = updatePeriod;

        dynamic_reconfigure::Server<hl_navigation::NavigationConfig>::CallbackType f;
        f = boost::bind(&Navigator::readDynamicParameters, this, _1, _2);
        server.setCallback(f);

        timer = n.createTimer(ros::Duration(updatePeriod), &Navigator::update, this);

        //actions

        as_.registerGoalCallback(boost::bind(&Navigator::goToTarget, this));
        as_.registerPreemptCallback(boost::bind(&Navigator::preemptTarget, this));
        as_.start();
}

Navigator::~Navigator()
{
        stop();
}
