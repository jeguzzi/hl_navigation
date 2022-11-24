/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */
#include <memory>
#include <chrono>
#include "Navigator.h"
#include "tf2/exceptions.h"
#include <tf2/utils.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

// TF Utilities


template <typename T>
bool Navigator::inOdomFrame(const T &in, T &out)
{
        try {
                tf_buffer_->transform<T>(in, out, odom_frame);
        }
        catch(tf2::TransformException& ex) {
                RCLCPP_ERROR(this->get_logger(), "Received an exception trying to transform a pose from %s to %s: %s", in.header.frame_id.c_str(), odom_frame.c_str(), ex.what());
                return false;
        }
        return true;
}


// ROS Subscriptions


void Navigator::setStopFromMessage(const std_msgs::msg::Empty& msg)
{
   RCLCPP_INFO(this->get_logger(), "Got stop msg");
        stop();
}

void Navigator::setTargetFromPointMessage(const geometry_msgs::msg::PointStamped& msg)
{
        setTargetFromPoint(msg);
}

bool Navigator::setTargetFromPoint(const geometry_msgs::msg::PointStamped& msg)
{
        if(odom_frame=="") return false;
        geometry_msgs::msg::PointStamped targetPoint;
        if(!inOdomFrame(msg, targetPoint)) return false;
        targetType = POINT;
        agent->targetPosition = CVector2(targetPoint.point.x,targetPoint.point.y);
        targetZ = targetPoint.point.z;
        state = MOVE;
        if(drawing_enabled) drawTarget(targetPoint.point, targetPoint.header.frame_id);
        return true;
}

void Navigator::setTargetFromPoseMessage(const geometry_msgs::msg::PoseStamped& msg)
{
        setTargetFromPose(msg);
}

bool Navigator::setTargetFromPose(const geometry_msgs::msg::PoseStamped& msg)
{
        if(odom_frame== "") return false;
        geometry_msgs::msg::PoseStamped targetPose;
        if(!inOdomFrame(msg, targetPose)) return false;
        targetType = POSE;
        agent->targetPosition = CVector2(targetPose.pose.position.x,targetPose.pose.position.y);
        targetZ = targetPose.pose.position.z;
        targetAngle = tf2::getYaw(targetPose.pose.orientation);
        agent->desiredAngle = CRadians(targetAngle)-agent->angle;
        state = MOVE;
        if(drawing_enabled) drawTarget(targetPose.pose.position, targetPose.header.frame_id);
        return true;
}


void Navigator::setObstaclesFromMessage(const hl_navigation_msgs::msg::Obstacles& obstacles_msg)
{
        if(odom_frame=="") return;
        agent->clearObstacles();
        for(auto msg : obstacles_msg.obstacles)
        {
                geometry_msgs::msg::PointStamped topPosition;
                geometry_msgs::msg::Vector3Stamped velocity;
                if(!(inOdomFrame(msg.top_position, topPosition) && inOdomFrame(msg.velocity, velocity))) continue;
                double minZ=topPosition.point.z - msg.height;
                double maxZ=topPosition.point.z;
                bool relevant=false;
                //Consider the cylindric obstacles iff it intersects the strip z-targetZ
                if(!((z<minZ && targetZ<minZ) || (z>maxZ && targetZ>maxZ)))
                {
                        relevant = true;
                        CVector2 p(topPosition.point.x, topPosition.point.y);
                        CVector2 v(velocity.vector.x, velocity.vector.y);
                        agent->addObstacleAtPoint(p, v, msg.radius, msg.social_margin);
                }
                if(drawing_enabled)
                {

                        drawObstacle(topPosition,msg.height,msg.radius,msg.social_margin,relevant);
                        drawObstacleVelocity(topPosition,velocity.vector,relevant);
                }
        }
}

void Navigator::setOdometryFromMessage(const nav_msgs::msg::Odometry& msg)
{
        lastTimeWhenLocalized=now();
        odom_frame=msg.header.frame_id;
        geometry_msgs::msg::Pose pose = msg.pose.pose;
        geometry_msgs::msg::Twist twist = msg.twist.twist;
        // HACK: We assume that twist and pose are in the same frame!
        yaw=tf2::getYaw(pose.orientation);
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
        std::string agent_type_name = declare_parameter("type", std::string("TWO_WHEELED"));
        agentType agent_type = TWO_WHEELED;
        if(agent_type_name=="TWO_WHEELED") agent_type=TWO_WHEELED;
        else if (agent_type_name=="HOLONOMIC") agent_type=HOLONOMIC;
        else if (agent_type_name=="HEAD") agent_type=HEAD;

        double axis_length = declare_parameter("axis_length", 1.0);
        double radius = declare_parameter("radius", 0.3);
        double maximal_angular_speed = declare_parameter("maximal_angular_speed", DEFAULT_MAX_ANGULAR_SPEED);
        double max_speed = declare_parameter("maximal_speed", 0.3);
        // TODO(Jerome): complete angular vs rotation speed
        double maximal_rotation_speed = declare_parameter("maximal_rotation_speed", DEFAULT_MAX_ANGULAR_SPEED);

        for(auto agent : agents) {
                agent->axisLength=axis_length;
                agent->type=agent_type;
                agent->radius = radius;
                agent->setMaxSpeed(max_speed);
                agent->setMaxAngularSpeed(maximal_angular_speed);
                if(agent_type==TWO_WHEELED)
                {
                  agent->setMaxRotationSpeed(maximal_rotation_speed);
                }
        }
        //   pn.param("maximal_vertical_speed",maximalVerticalSpeed, 1.0);

}

// State transitions

void Navigator::turn()
{
        state = TURN;
        agent->desiredSpeed=0.0;
}

void Navigator::brake()
{
   RCLCPP_INFO(this->get_logger(), "Asked to break %d", state);
   if(state!=BRAKING)
      {
        RCLCPP_INFO(this->get_logger(), "Will break");
        state=BRAKING;
     }
}

void Navigator::stop()
{
   // RCLCPP_INFO(this->get_logger(), "Asked to stop %d %d", state, as_.isActive());
      if(state!=IDLE)
      {
        RCLCPP_INFO(this->get_logger(), "Will break");
        state=IDLE;
        setMotorVelocity(0,0,0);

        if (goal_handle)
        {
                RCLCPP_INFO(this->get_logger(), "Will abort");
                auto r = std::make_shared<GoToTarget::Result>();
                goal_handle->abort(r);
                goal_handle = nullptr;
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
                                        if(goal_handle)
                                        {
                                                RCLCPP_INFO(this->get_logger(), "Set goal reached");
                                                auto r = std::make_shared<GoToTarget::Result>();
                                                goal_handle->succeed(r);
                                                goal_handle = nullptr;
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
                        if(goal_handle)
                        {
                                auto f = std::make_shared<GoToTarget::Feedback>();
                                f->distance = targetDistance;
                                goal_handle->publish_feedback(f);
                        }
                }
                break;
        case TURN:
                if(is_at_target_angle())
                {
                        if(goal_handle)
                        {
                                RCLCPP_INFO(this->get_logger(), "Set goal reached");
                                auto r = std::make_shared<GoToTarget::Result>();
                                goal_handle->succeed(r);
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

void Navigator::updateLocalization(rclcpp::Time now)
{
        if(odom_frame=="")
        {
                localized = false;
                return;
        }
        double dt = (now - lastTimeWhenLocalized).seconds();
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
        geometry_msgs::msg::Twist base_cmd;
        base_cmd.linear.x=newVelocityX;
        base_cmd.linear.y=newVelocityY;
        base_cmd.linear.z=newVelocityZ;
        base_cmd.angular.z=newAngularSpeed;
        navPublisher->publish(base_cmd);
        if(!publishCmdStamped)
        {
                geometry_msgs::msg::TwistStamped msg;
                msg.header.frame_id = odom_frame;
                msg.header.stamp = now();
                msg.twist.angular.z=newAngularSpeed;
                CVector2 v = CVector2(newVelocityX, newVelocityY);
                v.Rotate(agent->angle);
                msg.twist.linear.x = v.GetX();
                msg.twist.linear.y = v.GetY();
                msg.twist.linear.z = newVelocityZ;
                cmdStampedPublisher->publish(msg);
        }
}

void Navigator::setMotorVelocity(double newSpeed,double newVelocityZ,double newAngularSpeed)
{
        setMotorVelocity(newSpeed, 0, newVelocityZ, newAngularSpeed);
}

void Navigator::update()
{
        updateLocalization(now());
        if(!localized && state!=IDLE)
        {
                RCLCPP_WARN(this->get_logger(), "NOT localized -> break");
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


rcl_interfaces::msg::SetParametersResult Navigator::on_set_parameters(const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto &param : parameters) {
      if (param.get_name() == "behavior") {
          setAgentFromString(param.as_string());
      }
      else if (param.get_name() == "optimal_speed") {
          agent->setOptimalSpeed(param.as_double());
      }
      /// TODO(old): distinguish between these two that are alternative in the old code
      else if (param.get_name() == "optimal_angular_speed") {
        if ((param.as_double() - agent->optimalAngularSpeed.GetValue()) > 1e-3) {
          agent->setOptimalAngularSpeed(param.as_double());
        }
          agent->setOptimalSpeed(param.as_double());
          // TODO(Jerome): replace with set param
          // config.optimal_rotation_speed = agent->optimalRotationSpeed;
          // config.optimal_angular_speed = agent->optimalAngularSpeed.GetValue();
      }
      else if (param.get_name() == "optimal_rotation_speed") {
        if (param.as_double() != agent->optimalRotationSpeed) {
          agent->setOptimalAngularSpeed(param.as_double());
        }
          agent->setOptimalRotationSpeed(param.as_double());
          // TODO(Jerome): replace with set param
          // config.optimal_rotation_speed = agent->optimalRotationSpeed;
          // config.optimal_angular_speed = agent->optimalAngularSpeed.GetValue();
      }
      else if (param.get_name() == "tau_z") {
        tauZ = param.as_double();
      }
      else if (param.get_name() == "optimal_vertical_speed") {
        optimalVerticalSpeed = param.as_double();
      }
      else if (param.get_name() == "tau") {
        agent->setTau(param.as_double());
      }
      else if (param.get_name() == "eta") {
        agent->setEta(param.as_double());
      }
      else if (param.get_name() == "rotation_tau") {
        agent->setRotationTau(param.as_double());
      }
      else if (param.get_name() == "horizon") {
        agent->setHorizon(param.as_double());
      }
      else if (param.get_name() == "time_horizon") {
        agent->setTimeHorizon(param.as_double());
      }
      else if (param.get_name() == "safety_margin") {
        agent->setSafetyMargin(param.as_double());
      }
      else if (param.get_name() == "aperture") {
        agent->setAperture(param.as_double());
      }
      else if (param.get_name() == "resolution") {
        agent->setResolution(param.as_double());
      }
      else if (param.get_name() == "drawing") {
        drawing_enabled = param.as_bool();
      }
      else if (param.get_name() == "tol_distance") {
        minDeltaDistance = param.as_double();
      }
      else if (param.get_name() == "tol_angle") {
        minDeltaAngle = param.as_double();
      }
      else if (param.get_name() == "point_toward_target") {
        rotateIfHolo = param.as_bool();
      }
      else if (param.get_name() == "minimal_speed") {
        minimalSpeed = param.as_double();
      }
  }
  return result;
}

void Navigator::init_params() {
  declare_parameter("behavior", "HL");
  declare_parameter("optimal_speed", 0.3);
  declare_parameter("optimal_angular_speed", 0.3);
  declare_parameter("optimal_rotation_speed", 0.3);
  declare_parameter("tau_z", 1.0);
  declare_parameter("optimal_vertical_speed", 0.1);
  declare_parameter("tau", 0.5);
  declare_parameter("eta", 0.5);
  declare_parameter("rotation_tau", 0.5);
  declare_parameter("horizon", 1.0);
  declare_parameter("time_horizon", 1.0);
  declare_parameter("safety_margin", 0.1);
  declare_parameter("aperture", 3.14);
  declare_parameter("resolution", 30);
  declare_parameter("drawing", false);
  declare_parameter("tol_distance", 0.2);
  declare_parameter("tol_angle", 0.1);
  declare_parameter("point_toward_target", false);
  declare_parameter("minimal_speed", 0.05);
}

Navigator::Navigator() : Node("navigator")
{
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        state = IDLE;
        navPublisher = create_publisher<geometry_msgs::msg::Twist>("cmd_vel",1);
        publishCmdStamped = declare_parameter("publish_cmd_stamped", false);
        cmdStampedPublisher = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_stamped",1);
        stopSubscriber = create_subscription<std_msgs::msg::Empty>("stop", 1, std::bind(&Navigator::setStopFromMessage, this, _1));
        targetPoseSubscriber = create_subscription<geometry_msgs::msg::PoseStamped>("target_pose", 1, std::bind(&Navigator::setTargetFromPoseMessage, this, _1));
        targetPointSubscriber = create_subscription<geometry_msgs::msg::PointStamped>("target_point", 1, std::bind(&Navigator::setTargetFromPointMessage, this, _1));
        obstaclesSubscriber = create_subscription<hl_navigation_msgs::msg::Obstacles>("obstacles", 1, std::bind(&Navigator::setObstaclesFromMessage, this, _1));
        odom_frame="";
        odometrySubscriber = create_subscription<nav_msgs::msg::Odometry>("odom",1, std::bind(&Navigator::setOdometryFromMessage,this, _1));
        // tf_prefix = declare_parameter("tf_prefix", "").get_parameter_value();
        // if (!tf_prefix.empty() && !tf_prefix.ends_with('/')) {
        //   tf_prefix += "/";
        // }

        action_server_ = rclcpp_action::create_server<GoToTarget>(
          this,
          "go_to_target",
          std::bind(&Navigator::handle_goal, this, _1, _2),
          std::bind(&Navigator::handle_cancel, this, _1),
          std::bind(&Navigator::handle_accepted, this, _1));

        ns = get_effective_namespace();

        initDrawing();
        agents.push_back(&orcaAgent);
        agents.push_back(&hrvoAgent);
        agents.push_back(&hlAgent);

        readStaticParameters();
        init_params();

        double rate = declare_parameter("rate", 10.0);
        updatePeriod = 1.0/rate;
        hlAgent.dt = updatePeriod;

        param_callback_handle = add_on_set_parameters_callback(std::bind(&Navigator::on_set_parameters, this, _1));

        create_wall_timer(std::chrono::milliseconds((long) (1e6 * updatePeriod)), std::bind(&Navigator::update, this));

}


rclcpp_action::GoalResponse Navigator::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const GoToTarget::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  if (!goal_handle)
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  return rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse Navigator::handle_cancel(
  const std::shared_ptr<GoalHandleGoToTarget> _goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  if (_goal_handle == goal_handle) {
    goal_handle = nullptr;
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Navigator::handle_accepted(const std::shared_ptr<GoalHandleGoToTarget> _goal_handle)
{
  goal_handle = _goal_handle;
  auto goal = goal_handle->get_goal();
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
          RCLCPP_WARN(this->get_logger(), "Goal not valid");
          auto r = std::make_shared<GoToTarget::Result>();
          goal_handle->abort(r);
          goal_handle = nullptr;
  }
}

Navigator::~Navigator()
{
        stop();
}
