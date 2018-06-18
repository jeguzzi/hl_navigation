/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef _NAVIGATOR_H_
#define _NAVIGATOR_H_

#include <cstdlib>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>

#include <actionlib/server/simple_action_server.h>
#include <hl_navigation_msgs/GoToTargetAction.h>

#include <hl_navigation_msgs/Obstacles.h>
#include <hl_navigation/NavigationConfig.h>

#include <angles/angles.h>

#include "Agent.h"
#include "HLAgent.h"
#include "ORCAAgent.h"
#include "HRVOAgent.h"

/**
    this is the class documentation
*/

class Navigator
{

 private:
  ros::NodeHandle n;
  ros::NodeHandle pn;
  std::string ns;
  tf::TransformListener listener;
  dynamic_reconfigure::Server<hl_navigation::NavigationConfig> server;

  std::string odom_frame;
  ros::Time lastTimeWhenLocalized;
  bool localized;
  void updateLocalization(ros::Time now);
  void setOdometryFromMessage(const nav_msgs::Odometry& msg);
  ros::Subscriber odometrySubscriber;

  enum {POSE, POINT} targetType;
  void setTargetFromPointMessage(const geometry_msgs::PointStamped& msg);
  void setTargetFromPoseMessage(const geometry_msgs::PoseStamped& msg);
  bool setTargetFromPoint(const geometry_msgs::PointStamped& msg);
  bool setTargetFromPose(const geometry_msgs::PoseStamped& msg);

  ros::Subscriber targetPoseSubscriber, targetPointSubscriber;

  //actions

  actionlib::SimpleActionServer<hl_navigation_msgs::GoToTargetAction> as_;
  void goToTarget();
  void preemptTarget();

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
  void setStopFromMessage(const std_msgs::Empty& msg);
  ros::Subscriber stopSubscriber;

  bool publishCmdStamped;
  void setMotorVelocity(double speed,double verticalVelocity, double angularSpeed);
  void setMotorVelocity(double newVelocityX,double newVelocityY,double verticalVelocity,double newAngularSpeed);
  ros::Publisher navPublisher, cmdStampedPublisher;
  void updateVerticalVelocity();
  void update(const ros::TimerEvent&);
  ros::Timer timer;


  void setObstaclesFromMessage(const hl_navigation_msgs::Obstacles& msg);
  ros::Subscriber obstaclesSubscriber;

  //RVIZ drawing
  bool drawing_enabled;
  int obstacleIndex;
  double updatePeriod;
  void initDrawing();
  void drawCollisionMap();
  void drawDesiredVelocity(double s,double a);
  void drawObstacleVelocity(geometry_msgs::PointStamped &p,geometry_msgs::Vector3 velocity ,bool relevan);
  void drawObstacle(geometry_msgs::PointStamped &p,double height,double r,double s,bool relevant);
  void drawTarget(const geometry_msgs::Point &msg, std::string frame_id);
  ros::Publisher obstacles_marker_pub,target_marker_pub,collision_marker_pub,desired_velocity_marker_pub;


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
  void setAgentFromString(std::string behaviorName);
  void readDynamicParameters(hl_navigation::NavigationConfig &config, uint32_t level);

 public:

  Navigator();
  ~Navigator();
};

#endif /* _NAVIGATOR_H_ */
