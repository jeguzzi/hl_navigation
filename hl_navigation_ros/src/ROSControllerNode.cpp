/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "hl_navigation_msgs/action/go_to_target.hpp"
#include "hl_navigation_msgs/msg/obstacles.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/empty.hpp"

#include "hl_navigation/behavior.h"
#include "hl_navigation/controller.h"
#include "hl_navigation/behaviors/HL.h"
#include "hl_navigation/behaviors/HRVO.h"
#include "hl_navigation/behaviors/ORCA.h"

#include "Drawing.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using GoToTarget = hl_navigation_msgs::action::GoToTarget;
using GoalHandleGoToTarget = rclcpp_action::ServerGoalHandle<GoToTarget>;


namespace hl_navigation {

class ROSControllerNode : public Controller, public rclcpp::Node {
 public:
  ROSControllerNode()
      : Node("controller"), Controller(),
        drawing(*this, get_effective_namespace()) {
    readStaticParameters();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    navPublisher = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    publishCmdStamped = declare_parameter("publish_cmd_stamped", false);
    cmdStampedPublisher = create_publisher<geometry_msgs::msg::TwistStamped>(
        "cmd_vel_stamped", 1);
    stopSubscriber = create_subscription<std_msgs::msg::Empty>(
        "stop", 1, std::bind(&ROSControllerNode::setStopFromMessage, this, _1));
    targetPoseSubscriber = create_subscription<geometry_msgs::msg::PoseStamped>(
        "target_pose", 1,
        std::bind(&ROSControllerNode::setTargetPoseFromMsg, this, _1));
    targetPointSubscriber =
        create_subscription<geometry_msgs::msg::PointStamped>(
            "target_point", 1,
            std::bind(&ROSControllerNode::setTargetPointFromMsg, this, _1));
    obstaclesSubscriber =
        create_subscription<hl_navigation_msgs::msg::Obstacles>(
            "obstacles", 1,
            std::bind(&ROSControllerNode::setObstaclesFromMessage, this, _1));
    odom_frame = "";
    odometrySubscriber = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 1, std::bind(&ROSControllerNode::setOdometryFromMessage, this, _1));
    // tf_prefix = declare_parameter("tf_prefix", "").get_parameter_value();
    // if (!tf_prefix.empty() && !tf_prefix.ends_with('/')) {
    //   tf_prefix += "/";
    // }

    action_server_ = rclcpp_action::create_server<GoToTarget>(
        this, "go_to_target", std::bind(&ROSControllerNode::handle_goal, this, _1, _2),
        std::bind(&ROSControllerNode::handle_cancel, this, _1),
        std::bind(&ROSControllerNode::handle_accepted, this, _1));

    param_callback_handle = add_on_set_parameters_callback(
        std::bind(&ROSControllerNode::on_set_parameters, this, _1));
    double rate = declare_parameter("rate", 10.0);
    updatePeriod = 1.0 / rate;

    init_params();

    timer_ = create_wall_timer(
        std::chrono::milliseconds((unsigned)(1e3 * updatePeriod)),
        [this]() {update(updatePeriod);});
  }

 private:
  bool drawing_enabled;
  Drawing drawing;
  std::map<std::string, std::unique_ptr<Behavior>> behaviors;
  double updatePeriod;
  bool publishCmdStamped;
  HLBehavior * hl_behavior;
  std::string ns;
  std::string odom_frame;
  rclcpp::Time lastTimeWhenLocalized;
  // std::string tf_prefix;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySubscriber;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      targetPoseSubscriber;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      targetPointSubscriber;
  rclcpp::Subscription<hl_navigation_msgs::msg::Obstacles>::SharedPtr
      obstaclesSubscriber;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stopSubscriber;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr navPublisher;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      cmdStampedPublisher;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      obstacles_marker_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      target_marker_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      collision_marker_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      desired_velocity_marker_pub;
  std::shared_ptr<GoalHandleGoToTarget> goal_handle;
  rclcpp_action::Server<hl_navigation_msgs::action::GoToTarget>::SharedPtr
      action_server_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle;

  void arrived() override {
    if (goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Set goal reached");
      auto r = std::make_shared<GoToTarget::Result>();
      goal_handle->succeed(r);
      goal_handle = nullptr;
    }
  }

  void updated() override {
    if (goal_handle) {
      auto f = std::make_shared<GoToTarget::Feedback>();
      f->distance = targetDistance;
      goal_handle->publish_feedback(f);
    }
  }

  void aborted() override {
    if (goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Will abort");
      auto r = std::make_shared<GoToTarget::Result>();
      goal_handle->abort(r);
      goal_handle = nullptr;
    }
  }

  void updated_control() override {
    if (drawing_enabled) {
      drawing.drawDesiredVelocity(behavior->desiredVelocity);
      if (state == MOVE && behavior == hl_behavior) {
        drawing.drawCollisionMap(
            -hl_behavior->aperture,
            hl_behavior->angleResolution(), hl_behavior->getDistances(),
            hl_behavior->get_radius());
      }
    }
  }

  template <typename T> bool inOdomFrame(const T &in, T &out) {
    try {
      tf_buffer_->transform<T>(in, out, odom_frame);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Received an exception trying to transform a pose from %s to %s: %s",
          in.header.frame_id.c_str(), odom_frame.c_str(), ex.what());
      return false;
    }
    return true;
  }

  void setOdometryFromMessage(const nav_msgs::msg::Odometry &msg) {
    lastTimeWhenLocalized = now();
    odom_frame = msg.header.frame_id;
    const geometry_msgs::msg::Pose &pose = msg.pose.pose;
    if (odom_frame != msg.child_frame_id) {
      geometry_msgs::msg::Vector3Stamped linear, angular;
      geometry_msgs::msg::Vector3Stamped linear_odom, angular_odom;
      linear.header.frame_id = msg.child_frame_id;
      angular.header.frame_id = msg.child_frame_id;
      linear.vector = msg.twist.twist.linear;
      // TODO(Jerome): not needed
      angular.vector = msg.twist.twist.angular;
      if (inOdomFrame(linear, linear_odom) &&
          inOdomFrame(angular, angular_odom)) {
        behavior->velocity = Vector2(linear_odom.vector.x, linear_odom.vector.y);
        behavior->angularSpeed = angular_odom.vector.z;
      } else {
        RCLCPP_INFO(
            get_logger(),
            "Cannot set odometry: cannot transform %s to odometry frame %s",
            msg.child_frame_id.c_str(), odom_frame.c_str());
        return;
      }
    } else {
      const geometry_msgs::msg::Twist &twist = msg.twist.twist;
      behavior->velocity = Vector2(twist.linear.x, twist.linear.y);
      behavior->angularSpeed = twist.angular.z;
    }
    // CHANGED: We don't assume anymore that twist and pose are in the same
    // frame!
    set_pose(Vector3{pose.position.x, pose.position.y, pose.position.z},
             tf2::getYaw(pose.orientation));
  }

  bool setTargetPointFromMsg(const geometry_msgs::msg::PointStamped &msg) {
    if (odom_frame == "") {
      RCLCPP_WARN(this->get_logger(),
                  "Cannot set target point: odometry frame is not known yet");
      return false;
    }
    geometry_msgs::msg::PointStamped targetPoint;
    if (!inOdomFrame(msg, targetPoint)) {
      RCLCPP_WARN(
          this->get_logger(),
          "Cannot set target point: cannot transform to odometry frame %s",
          odom_frame.c_str());
      return false;
    }
    set_target_point(Vector3{targetPoint.point.x, targetPoint.point.y, targetPoint.point.z});
    if (drawing_enabled) {
        drawing.drawTarget(targetPoint.point, targetPoint.header.frame_id, distance_tolerance);
    }
    return true;
  }

  bool setTargetPoseFromMsg(const geometry_msgs::msg::PoseStamped &msg) {
    if (odom_frame == "")
      return false;
    geometry_msgs::msg::PoseStamped targetPose;
    if (!inOdomFrame(msg, targetPose))
      return false;
    set_target_pose(Vector3{targetPose.pose.position.x, targetPose.pose.position.y,
                            targetPose.pose.position.z},
                    tf2::getYaw(targetPose.pose.orientation));
    if (drawing_enabled) {
        drawing.drawTarget(targetPose.pose.position,
                           targetPose.header.frame_id, distance_tolerance);
    }
    return true;
  }

  void setStopFromMessage(const std_msgs::msg::Empty &msg) {
    RCLCPP_INFO(this->get_logger(), "Got stop msg");
    stop();
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToTarget::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    if (!goal_handle)
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    return rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleGoToTarget> _goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    if (_goal_handle == goal_handle) {
      goal_handle = nullptr;
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGoToTarget> _goal_handle) {
    goal_handle = _goal_handle;
    auto goal = goal_handle->get_goal();
    // TODO(old): check the outcome.
    bool valid;
    if (goal->target_pose.header.frame_id != "") {
      valid = setTargetPoseFromMsg(goal->target_pose);
    } else {
      valid = setTargetPointFromMsg(goal->target_point);
    }
    if (!valid) {
      RCLCPP_WARN(this->get_logger(), "Goal not valid");
      auto r = std::make_shared<GoToTarget::Result>();
      goal_handle->abort(r);
      goal_handle = nullptr;
    } else {
      RCLCPP_INFO(this->get_logger(), "Start moving");
      state = MOVE;
    }
  }

  virtual void set_target_twist(const Twist2D & twist, float vertical_speed) override {
    // Desired velocity is in robot frame
    geometry_msgs::msg::Twist base_cmd;
    base_cmd.linear.x = twist.longitudinal;
    base_cmd.linear.y = twist.lateral;
    // TODO(Jerome): check ... is nan in
    // base_cmd.linear.z=newVelocityZ;
    base_cmd.angular.z = twist.angular;
    navPublisher->publish(base_cmd);
    // RCLCPP_INFO(this->get_logger(), "navPublisher publish %.3f %.3f %.3f",
    // newVelocityX,
    //             newVelocityY, newAngularSpeed);
    if (publishCmdStamped) {
      geometry_msgs::msg::TwistStamped msg;
      msg.header.frame_id = odom_frame;
      msg.header.stamp = now();
      msg.twist.angular.z = twist.angular;
      Vector2 v = Vector2(twist.longitudinal, twist.lateral);
      v = rotate(v, behavior->angle);
      msg.twist.linear.x = v.x();
      msg.twist.linear.y = v.y();
      msg.twist.linear.z = vertical_speed;
      cmdStampedPublisher->publish(msg);
    }
  }

  void setObstaclesFromMessage(
      const hl_navigation_msgs::msg::Obstacles &obstacles_msg) {
    if (odom_frame == "")
      return;
    std::vector<Cylinder> obstacles;
    for (auto msg : obstacles_msg.obstacles) {
      geometry_msgs::msg::PointStamped topPosition;
      geometry_msgs::msg::Vector3Stamped velocity;
      if (!(inOdomFrame(msg.top_position, topPosition) &&
            inOdomFrame(msg.velocity, velocity)))
        continue;
      Vector3 p{topPosition.point.x, topPosition.point.y, topPosition.point.z - msg.height};
      Vector3 v(velocity.vector.x, velocity.vector.y, velocity.vector.z);
      obstacles.emplace_back(p, msg.radius, msg.height, msg.social_margin, v);
      if (drawing_enabled) {
        drawing.drawObstacle(topPosition, msg.height, msg.radius,
                             msg.social_margin, true, updatePeriod);
        drawing.drawObstacleVelocity(topPosition, velocity.vector, true, updatePeriod);
      }
    }
    set_neighbors(obstacles);
  }

  void readStaticParameters() {
    std::string agent_type_name =
        declare_parameter("type", std::string("TWO_WHEELED"));
    agent_type_t agent_type = TWO_WHEELED;
    if (agent_type_name == "TWO_WHEELED")
      agent_type = TWO_WHEELED;
    else if (agent_type_name == "HOLONOMIC")
      agent_type = HOLONOMIC;
    else if (agent_type_name == "HEAD")
      agent_type = HEAD;
    else if (agent_type_name == "FOUR_WHEELED_OMNI")
      agent_type = HEAD;

    double axis_length = declare_parameter("axis_length", 1.0);
    double radius = declare_parameter("radius", 0.3);

    for (auto const &p : Behavior::all_behaviors()) {
      behaviors.emplace(p.first, p.second(agent_type, radius, axis_length));
    }

    hl_behavior = dynamic_cast<HLBehavior *>(behaviors["HL"].get());


    double maximal_angular_speed =
        declare_parameter("maximal_angular_speed", 10.0);
    double max_speed = declare_parameter("maximal_speed", 0.3);
    // TODO(Jerome): complete angular vs rotation speed
    // double maximal_rotation_speed =
    //     declare_parameter("maximal_rotation_speed", DEFAULT_MAX_ANGULAR_SPEED);

    for (auto & p : behaviors) {
      // p.second->axisLength = axis_length;
      // p.second->type = behavior_type;
      // p.second->radius = radius;
      p.second->set_max_speed(max_speed);
      p.second->set_max_angular_speed(maximal_angular_speed);
      // TODO(Jerome): reenable after sync angular vs rotation speed

      // if (agent_type == TWO_WHEELED) {
      //   p.second->setMaxRotationSpeed(maximal_rotation_speed);
      //   // behavior->setMaxRotationSpeed(maximal_speed);
      // }
    }
    //   pn.param("maximal_vertical_speed",maximalVerticalSpeed, 1.0);
  }

  void init_params() {
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

  void setBehaviorFromString(std::string behaviorName) {
    if (behaviorName == "HL") {
      behavior = behaviors["HL"].get();
    } else if (behaviorName == "ORCA") {
      behavior = behaviors["ORCA"].get();
      dynamic_cast<ORCABehavior *>(behavior)->useEffectiveCenter = false;
    } else if (behaviorName == "ORCA-NH") {
      behavior = behaviors["ORCA"].get();
      dynamic_cast<ORCABehavior *>(behavior)->useEffectiveCenter = true;
    } else if (behaviorName == "HRVO") {
      behavior = behaviors["HRVO"].get();
    } else if (behaviorName == "Dummy") {
      behavior = behaviors["Dummy"].get();
    } else {
      behavior = behaviors["HL"].get();
    }
  }

  rcl_interfaces::msg::SetParametersResult
  on_set_parameters(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &param : parameters) {
      RCLCPP_INFO(this->get_logger(), "param %s", param.get_name().c_str());
      if (param.get_name() == "behavior") {
        setBehaviorFromString(param.as_string());
      } else if (param.get_name() == "optimal_speed") {
        behavior->set_optimal_speed(param.as_double());
      } else if (param.get_name() == "optimal_angular_speed") {
        behavior->set_optimal_angular_speed(param.as_double());
      } else if (param.get_name() == "tau_z") {
        tauZ = param.as_double();
      } else if (param.get_name() == "optimal_vertical_speed") {
        optimalVerticalSpeed = param.as_double();
      } else if (param.get_name() == "tau") {
        hl_behavior->setTau(param.as_double());
      } else if (param.get_name() == "eta") {
        hl_behavior->setEta(param.as_double());
      } else if (param.get_name() == "rotation_tau") {
        behavior->set_rotation_tau(param.as_double());
      } else if (param.get_name() == "horizon") {
        behavior->set_horizon(param.as_double());
      // TODO(Jerome): reenable
      // } else if (param.get_name() == "time_horizon") {
      //   behavior->setTimeHorizon(param.as_double());
      } else if (param.get_name() == "safety_margin") {
        behavior->set_safety_margin(param.as_double());
      } else if (param.get_name() == "aperture") {
        hl_behavior->setAperture(param.as_double());
      } else if (param.get_name() == "resolution") {
        hl_behavior->setResolution(param.as_int());
      } else if (param.get_name() == "drawing") {
        drawing_enabled = param.as_bool();
      } else if (param.get_name() == "tol_distance") {
        distance_tolerance = param.as_double();
      } else if (param.get_name() == "tol_angle") {
        angle_tolerance = param.as_double();
      // TODO(Jerome): reenable
      // else if (param.get_name() == "point_toward_target") {
      //   rotateIfHolo = param.as_bool();
      } else if (param.get_name() == "minimal_speed") {
        speed_tolerance = param.as_double();
      }
    }
    return result;
  }
};

}  // namespace hl_navigation

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hl_navigation::ROSControllerNode>();
  rclcpp::spin(node);
  RCLCPP_INFO(rclcpp::get_logger("hl_navigation"), "Shutting down");
  rclcpp::shutdown();
  return 0;
}
