/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef _NAVIGATOR_H_
#define _NAVIGATOR_H_

#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/empty.hpp"
#include "hl_navigation_msgs/action/go_to_target.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"


// #include "hl_navigation_msgs/action/go_to_target_action.hpp"
#include "hl_navigation_msgs/msg/obstacles.hpp"

// #include <angles/angles.h>

#include "Agent.h"
#include "HLAgent.h"
#include "ORCAAgent.h"
#include "HRVOAgent.h"

/**
    this is the class documentation
*/

using GoToTarget = hl_navigation_msgs::action::GoToTarget;
using GoalHandleGoToTarget = rclcpp_action::ServerGoalHandle<GoToTarget>;

class Navigator : public rclcpp::Node
{

 private:

  std::string ns;
  //std::string tf_prefix;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::string odom_frame;
  rclcpp::Time lastTimeWhenLocalized;
  bool localized;
  void updateLocalization(rclcpp::Time now);
  void setOdometryFromMessage(const nav_msgs::msg::Odometry& msg);
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySubscriber;

  enum {POSE, POINT} targetType;
  void setTargetFromPointMessage(const geometry_msgs::msg::PointStamped& msg);
  void setTargetFromPoseMessage(const geometry_msgs::msg::PoseStamped& msg);
  bool setTargetFromPoint(const geometry_msgs::msg::PointStamped& msg);
  bool setTargetFromPose(const geometry_msgs::msg::PoseStamped& msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetPoseSubscriber;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr targetPointSubscriber;

  //actions

  rclcpp_action::Server<hl_navigation_msgs::action::GoToTarget>::SharedPtr action_server_;
  std::shared_ptr<GoalHandleGoToTarget> goal_handle;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GoToTarget::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGoToTarget>);
  void handle_accepted(const std::shared_ptr<GoalHandleGoToTarget>);
  //States

  Agent *agent;
  std::vector<Agent *>agents;
  HLAgent hlAgent;
  ORCAAgent orcaAgent;
  HRVOAgent hrvoAgent;

  bool rotateIfHolo;

  double z;
  double yaw;
  double targetZ;
  double targetAngle;
  double targetDistance;

  enum {MOVE,TURN,IDLE, BRAKING} state;
  bool is_at_target_point();
  bool is_at_target_angle();
  void updateTargetState();
  void stop();
  void turn();
  void brake();
  void setStopFromMessage(const std_msgs::msg::Empty& msg);
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stopSubscriber;

  bool publishCmdStamped;
  void setMotorVelocity(double speed,double verticalVelocity, double angularSpeed);
  void setMotorVelocity(double newVelocityX,double newVelocityY,double verticalVelocity,double newAngularSpeed);
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr navPublisher;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmdStampedPublisher;
  void updateVerticalVelocity();
  void update();
  rclcpp::TimerBase::SharedPtr timer_;


  void setObstaclesFromMessage(const hl_navigation_msgs::msg::Obstacles& msg);
  rclcpp::Subscription<hl_navigation_msgs::msg::Obstacles>::SharedPtr obstaclesSubscriber;

  //RVIZ drawing
  bool drawing_enabled;
  int obstacleIndex;
  double updatePeriod;
  void initDrawing();
  void drawCollisionMap();
  void drawDesiredVelocity(double s,double a);
  void drawObstacleVelocity(geometry_msgs::msg::PointStamped &p,geometry_msgs::msg::Vector3 velocity ,bool relevan);
  void drawObstacle(geometry_msgs::msg::PointStamped &p,double height,double r,double s,bool relevant);
  void drawTarget(const geometry_msgs::msg::Point &msg, std::string frame_id);

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacles_marker_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_marker_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr collision_marker_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr desired_velocity_marker_pub;

  //TF

  template <typename T>
  bool inOdomFrame(const T &in, T &out);

  //Navigation parameters

  double minDeltaAngle;
  double minDeltaDistance;
  double minDeltaHeight;
  double velocityZ;
  double tauZ;
  double optimalVerticalSpeed;
  double maximalVerticalSpeed;
  double maximalSpeed;
  double minimalSpeed;
  double maximalAngularSpeed; //rad/sec
  void readStaticParameters();
  void init_params();
  void setAgentFromString(std::string behaviorName);


  OnSetParametersCallbackHandle::SharedPtr param_callback_handle;


  rcl_interfaces::msg::SetParametersResult on_set_parameters(const std::vector<rclcpp::Parameter> &parameters);


 public:
   Navigator();
  ~Navigator();
};

#endif /* _NAVIGATOR_H_ */
