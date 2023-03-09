/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

// TODO(Jerome): localization check

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#define _USE_MATH_DEFINES
#include <cmath>
#include <memory>

#include "./MarkersPublisher.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "hl_navigation/behavior.h"
#include "hl_navigation/behaviors/HL.h"
#include "hl_navigation/behaviors/HRVO.h"
#include "hl_navigation/behaviors/ORCA.h"
#include "hl_navigation/controller.h"
#include "hl_navigation/controller_3d.h"
#include "hl_navigation_msgs/action/go_to_target.hpp"
#include "hl_navigation_msgs/msg/neighbors.hpp"
#include "hl_navigation_msgs/msg/obstacles.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using std::placeholders::_1;
using std::placeholders::_2;
using GoToTarget = hl_navigation_msgs::action::GoToTarget;
using GoalHandleGoToTarget = rclcpp_action::ServerGoalHandle<GoToTarget>;

namespace hl_navigation {

static std::shared_ptr<Kinematic> make_kinematic(const std::string &name,
                                                 float max_speed,
                                                 float max_angular_speed,
                                                 float axis) {
  if (name == "two_wheeled") {
    return std::make_shared<TwoWheeled>(max_speed, axis);
  } else if (name == "four_wheeled") {
    return std::make_shared<FourWheeled>(max_speed, axis);
  } else if (name == "forward") {
    return std::make_shared<Forward>(max_speed, max_angular_speed);
  } else {
    return std::make_shared<Holonomic>(max_speed, max_angular_speed);
  }
}

static Behavior::Heading heading_from_string(const std::string &name) {
  if (name == "target_point") {
    return Behavior::Heading::target_point;
  }
  if (name == "target_angle") {
    return Behavior::Heading::target_angle;
  }
  if (name == "target_angular_speed") {
    return Behavior::Heading::target_angle;
  }
  if (name == "velocity") {
    return Behavior::Heading::velocity;
  }
  return Behavior::Heading::idle;
}

static Vector3 vector_from(const geometry_msgs::msg::Vector3 &v) {
  return {v.x, v.y, v.z};
}

static Vector3 point_from(const geometry_msgs::msg::Point &v) {
  return {v.x, v.y, v.z};
}

static Eigen::Isometry3f transform_from(
    const geometry_msgs::msg::Transform &t) {
  return Eigen::Isometry3f(
      Eigen::Translation3f(t.translation.x, t.translation.y, t.translation.z) *
      Eigen::Quaternionf(t.rotation.w, t.rotation.x, t.rotation.y,
                         t.rotation.z));
}

static Eigen::Isometry3f transform_from(const geometry_msgs::msg::Pose &t) {
  return Eigen::Isometry3f(
      Eigen::Translation3f(t.position.x, t.position.y, t.position.z) *
      Eigen::Quaternionf(t.orientation.w, t.orientation.x, t.orientation.y,
                         t.orientation.z));
}

static float yaw_from(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2 * (q.w * q.z + q.x * q.y),
                    1 - 2 * (q.y * q.y + q.z * q.z));
}

// static float yaw_from(const Eigen::Quaternionf &q) {
//   return std::atan2(2 * (q.w() * q.z() + q.x() * q.y()),
//                     1 - 2 * (q.y() * q.y() + q.z() * q.z()));
// }

static float yaw_from(const Eigen::Isometry3f &t) {
  const auto rpy = t.linear().eulerAngles(2, 1, 0);
  if (abs(rpy[1]) > M_PI_2 && abs(rpy[2]) > M_PI_2) {
    return rpy[0] + M_PI;
  }
  return rpy[0];
}

static Pose3 pose_from(const Eigen::Isometry3f &value) {
  return {value.translation(), yaw_from(value)};
}

static Pose3 pose_from(const geometry_msgs::msg::Pose &pose) {
  return {point_from(pose.position), yaw_from(pose.orientation)};
}

static Twist3 twist_from(const geometry_msgs::msg::Twist &t) {
  return {vector_from(t.linear), static_cast<Radians>(t.angular.z), false};
}

class ROSControllerNode : public rclcpp::Node {
 public:
  ROSControllerNode()
      : rclcpp::Node("controller"),
        nav_controller(),
        markers_pub(*this),
        fixed_frame(""),
        goal_handle(nullptr) {
    const double rate = declare_parameter("rate", 10.0);
    update_period = 1.0 / rate;
    param_callback_handle = add_on_set_parameters_callback(
        std::bind(&ROSControllerNode::on_set_parameters, this, _1));
    init_params();
    tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);
    cmd_publisher = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    if (should_publish_cmd_stamped) {
      cmd_stamped_publisher =
          create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_stamped",
                                                             1);
    }
    stop_subscriber = create_subscription<std_msgs::msg::Empty>(
        "stop", 1, std::bind(&ROSControllerNode::stop_cb, this, _1));
    target_pose_subscriber =
        create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_pose", 1,
            std::bind(&ROSControllerNode::target_pose_cb, this, _1));
    // TODO(Jerome 2023): complete
    target_twist_subscriber =
        create_subscription<geometry_msgs::msg::TwistStamped>(
            "target_twist", 1,
            std::bind(&ROSControllerNode::target_twist_cb, this, _1));
    target_point_subscriber =
        create_subscription<geometry_msgs::msg::PointStamped>(
            "target_point", 1,
            std::bind(&ROSControllerNode::target_point_cb, this, _1));
    obstacles_subscriber =
        create_subscription<hl_navigation_msgs::msg::Obstacles>(
            "obstacles", 1,
            std::bind(&ROSControllerNode::obstacles_cb, this, _1));
    neighbors_subscriber =
        create_subscription<hl_navigation_msgs::msg::Neighbors>(
            "neighbors", 1,
            std::bind(&ROSControllerNode::neighbors_cb, this, _1));
    odometry_subscriber = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 1, std::bind(&ROSControllerNode::odom_cb, this, _1));
    action_server = rclcpp_action::create_server<GoToTarget>(
        this, "go_to", std::bind(&ROSControllerNode::handle_goal, this, _1, _2),
        std::bind(&ROSControllerNode::handle_cancel, this, _1),
        std::bind(&ROSControllerNode::handle_accepted, this, _1));
    nav_controller.set_cmd_cb(
        std::bind(&ROSControllerNode::publish_cmd, this, _1));
    timer = create_wall_timer(
        std::chrono::milliseconds((unsigned)(1e3 * update_period)),
        std::bind(&ROSControllerNode::update, this));
    RCLCPP_INFO(get_logger(), "Ready");
  }

 private:
  Controller3 nav_controller;
  MarkersPublisher markers_pub;
  std::map<std::string, std::shared_ptr<Behavior>> behaviors;
  double update_period;
  bool should_publish_cmd_stamped;
  std::string fixed_frame;
  rclcpp::Time last_localization_stamp;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      target_pose_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      target_point_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      target_twist_subscriber;
  rclcpp::Subscription<hl_navigation_msgs::msg::Obstacles>::SharedPtr
      obstacles_subscriber;
  rclcpp::Subscription<hl_navigation_msgs::msg::Neighbors>::SharedPtr
      neighbors_subscriber;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_subscriber;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      cmd_stamped_publisher;
  std::shared_ptr<GoalHandleGoToTarget> goal_handle;
  rclcpp_action::Server<hl_navigation_msgs::action::GoToTarget>::SharedPtr
      action_server;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle;

  void update() {
    if (goal_handle && goal_handle->is_canceling()) {
      auto result = std::make_shared<GoToTarget::Result>();
      goal_handle->canceled(result);
      goal_handle = nullptr;
      nav_controller.stop();
    }
    nav_controller.update_3d(update_period);
  }

  void done(Action::State state) {
    if (goal_handle) {
      auto r = std::make_shared<GoToTarget::Result>();
      if (state == Action::State::success) {
        RCLCPP_INFO(get_logger(), "Set goal reached");
        goal_handle->succeed(r);
      } else {
        RCLCPP_INFO(get_logger(), "Aborted");
        goal_handle->abort(r);
      }
      goal_handle = nullptr;
    }
  }

  void running(float time_remaining) {
    if (goal_handle) {
      auto f = std::make_shared<GoToTarget::Feedback>();
      // TODO(J): Modify interface
      f->distance = time_remaining;
      goal_handle->publish_feedback(f);
      if (markers_pub.enabled) {
        Behavior *behavior = nav_controller.get_behavior().get();
        HLBehavior *hl = dynamic_cast<HLBehavior *>(behavior);
        if (hl) {
          // TODO(Jerome): should publish them only when moving, not turning
          markers_pub.publish_hl_collisions(hl->get_collision_distance());
        }
        if (behavior) {
          markers_pub.publish_desired_velocity(
              behavior->get_desired_velocity());
        }
      }
    }
  }

  std::optional<Eigen::Isometry3f> get_transform(const std::string &from,
                                                 const std::string &to,
                                                 const tf2::TimePoint &tp) {
    try {
      return transform_from(
          tf_buffer->lookupTransform(from, to, tf2::TimePointZero).transform);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "No transform from %s to %s: %s", from.c_str(),
                   to.c_str(), ex.what());
      return std::nullopt;
    }
  }

  std::optional<Twist3> twist_from_msg(
      const geometry_msgs::msg::TwistStamped &msg,
      const std::string &frame_id) {
    if (frame_id == msg.header.frame_id) {
      return twist_from(msg.twist);
    }
    auto t = get_transform(frame_id, msg.header.frame_id,
                           tf2_ros::fromMsg(msg.header.stamp));
    if (!t) return std::nullopt;
    auto linear = t->linear() * vector_from(msg.twist.linear);
    auto angular = t->linear() * vector_from(msg.twist.angular);
    return Twist3{linear, angular.z()};
  }

  std::optional<Vector3> vector_from_msg(
      const geometry_msgs::msg::Vector3Stamped &msg,
      const std::string &frame_id) {
    if (frame_id == msg.header.frame_id) {
      return vector_from(msg.vector);
    }
    auto t = get_transform(frame_id, msg.header.frame_id,
                           tf2_ros::fromMsg(msg.header.stamp));
    if (!t) return std::nullopt;
    return t->linear() * vector_from(msg.vector);
  }

  std::optional<Vector3> point_from_msg(
      const geometry_msgs::msg::PointStamped &msg,
      const std::string &frame_id) {
    if (frame_id == msg.header.frame_id) {
      return point_from(msg.point);
    }
    auto t = get_transform(frame_id, msg.header.frame_id,
                           tf2_ros::fromMsg(msg.header.stamp));
    if (!t) return std::nullopt;
    return *t * point_from(msg.point);
  }

  std::optional<Pose3> pose_from_msg(const geometry_msgs::msg::PoseStamped &msg,
                                     const std::string &frame_id) {
    if (frame_id == msg.header.frame_id) {
      return pose_from(msg.pose);
    }
    auto t = get_transform(frame_id, msg.header.frame_id,
                           tf2_ros::fromMsg(msg.header.stamp));
    if (!t) return std::nullopt;
    return pose_from(*t * transform_from(msg.pose));
  }

  std::optional<std::tuple<Pose3, Twist3>> odom_from_msg(
      const nav_msgs::msg::Odometry &msg, const std::string &frame_id) {
    Pose3 pose;
    Twist3 twist;
    std::optional<Eigen::Isometry3f> t;
    if (frame_id == msg.header.frame_id) {
      pose = pose_from(msg.pose.pose);
    } else {
      t = get_transform(frame_id, msg.header.frame_id,
                        tf2_ros::fromMsg(msg.header.stamp));
      if (!t) return std::nullopt;
      pose = pose_from(*t * transform_from(msg.pose.pose));
    }
    if (frame_id == msg.child_frame_id) {
      twist = twist_from(msg.twist.twist);
    } else {
      if (msg.child_frame_id != msg.header.frame_id) {
        t = get_transform(frame_id, msg.child_frame_id,
                          tf2_ros::fromMsg(msg.header.stamp));
        if (!t) return std::nullopt;
      }
      auto linear = t->linear() * vector_from(msg.twist.twist.linear);
      auto angular = t->linear() * vector_from(msg.twist.twist.angular);
      twist = {linear, angular.z()};
    }
    return std::make_tuple(pose, twist);
  }

  void odom_cb(const nav_msgs::msg::Odometry &msg) {
    last_localization_stamp = now();
    auto odom = odom_from_msg(msg, fixed_frame);
    if (!odom) return;
    const auto &[pose, twist] = *odom;
    RCLCPP_INFO(get_logger(),
                "Odom -> %.2f %.2f %.2f %.2f-- %.2f %.2f %.2f %.2f",
                pose.position.x(), pose.position.y(), pose.position.z(),
                pose.orientation, twist.velocity.x(), twist.velocity.y(),
                twist.velocity.z(), twist.angular_speed);
    nav_controller.set_twist(twist);
    nav_controller.set_pose(pose);
  }

  void target_point_cb(const geometry_msgs::msg::PointStamped &msg) {
    if (goal_handle) return;
    auto target = point_from_msg(msg, fixed_frame);
    if (target) {
      nav_controller.follow_point(*target);
      if (markers_pub.enabled) {
        markers_pub.publish_target(*target, fixed_frame, 0.0);
      }
    }
  }

  void target_pose_cb(const geometry_msgs::msg::PoseStamped &msg) {
    if (goal_handle) return;
    auto target = pose_from_msg(msg, fixed_frame);
    if (target) {
      nav_controller.follow_pose(*target);
      if (markers_pub.enabled) {
        markers_pub.publish_target(*target, fixed_frame, 0.0);
      }
    }
  }

  void target_twist_cb(const geometry_msgs::msg::TwistStamped &msg) {
    if (goal_handle) return;
    auto target = twist_from_msg(msg, fixed_frame);
    if (target) {
      nav_controller.follow_twist(*target);
    }
  }

  void stop() {
    if (goal_handle) {
      auto r = std::make_shared<GoToTarget::Result>();
      goal_handle->abort(r);
      goal_handle = nullptr;
    }
    nav_controller.stop();
    publish_cmd({});
  }

  void stop_cb([[maybe_unused]] const std_msgs::msg::Empty &msg) { stop(); }

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const GoToTarget::Goal> goal) {
    RCLCPP_INFO(get_logger(), "Received goal request");
    if (!goal_handle && nav_controller.get_behavior())
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    return rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleGoToTarget> _goal_handle) {
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    if (_goal_handle == goal_handle) {
      return rclcpp_action::CancelResponse::ACCEPT;
    } else {
      return rclcpp_action::CancelResponse::REJECT;
    }
  }

  void handle_accepted(
      const std::shared_ptr<GoalHandleGoToTarget> _goal_handle) {
    goal_handle = _goal_handle;
    auto goal = goal_handle->get_goal();
    std::optional<Vector3> target_point;
    std::optional<Pose3> target_pose;
    if (goal->target_pose.header.frame_id != "") {
      target_pose = pose_from_msg(goal->target_pose, fixed_frame);
    } else {
      target_point = point_from_msg(goal->target_point, fixed_frame);
    }
    if (!target_pose && !target_point) {
      RCLCPP_WARN(get_logger(), "Goal not valid");
      auto r = std::make_shared<GoToTarget::Result>();
      goal_handle->abort(r);
      goal_handle = nullptr;
    } else if (target_point) {
      RCLCPP_WARN(get_logger(), "go_to_position");
      auto action = nav_controller.go_to_position(*target_point,
                                                  goal->position_tolerance);
      action->running_cb = std::bind(&ROSControllerNode::running, this, _1);
      action->done_cb = std::bind(&ROSControllerNode::done, this, _1);
      if (markers_pub.enabled) {
        markers_pub.publish_target(*target_point, fixed_frame,
                                   goal->position_tolerance);
      }
    } else {
      RCLCPP_WARN(get_logger(), "go_to_pose");
      auto action = nav_controller.go_to_pose(
          *target_pose, goal->position_tolerance, goal->orientation_tolerance);
      action->running_cb = std::bind(&ROSControllerNode::running, this, _1);
      action->done_cb = std::bind(&ROSControllerNode::done, this, _1);
      if (markers_pub.enabled) {
        markers_pub.publish_target(*target_pose, fixed_frame,
                                   goal->position_tolerance,
                                   goal->orientation_tolerance);
      }
    }
  }

  void publish_cmd(const Twist3 &twist) {
    if (should_publish_cmd_stamped) {
      // publish in odom frame
      geometry_msgs::msg::TwistStamped msg;
      msg.header.frame_id = fixed_frame;
      msg.header.stamp = now();
      if (twist.relative) {
        const auto behavior = nav_controller.get_behavior();
        if (!behavior) return;
        Twist2 a_twist = behavior->to_absolute(twist.project());
        msg.twist.linear.x = a_twist.velocity.x();
        msg.twist.linear.y = a_twist.velocity.y();
      } else {
        msg.twist.linear.x = twist.velocity.x();
        msg.twist.linear.y = twist.velocity.y();
      }
      msg.twist.linear.z = twist.velocity.z();
      msg.twist.angular.z = twist.angular_speed;
      cmd_stamped_publisher->publish(msg);
    } else {
      // publish in robot frame
      geometry_msgs::msg::Twist msg;
      if (!twist.relative) {
        const auto behavior = nav_controller.get_behavior();
        if (!behavior) return;
        Twist2 r_twist = behavior->to_relative(twist.project());
        msg.linear.x = r_twist.velocity.x();
        msg.linear.y = r_twist.velocity.y();
      } else {
        msg.linear.x = twist.velocity.x();
        msg.linear.y = twist.velocity.y();
      }
      msg.linear.z = twist.velocity.z();
      msg.angular.z = twist.angular_speed;
      cmd_publisher->publish(msg);
    }
  }

  void neighbors_cb(const hl_navigation_msgs::msg::Neighbors &msg) {
    std::vector<Neighbor3> neighbors;
    for (auto msg : msg.neighbors) {
      auto position = point_from_msg(msg.obstacle.top_position, fixed_frame);
      if (!position) return;
      auto velocity = vector_from_msg(msg.velocity, fixed_frame);
      if (!velocity) return;
      *position -= Vector3(0.0f, 0.0f, msg.obstacle.height);
      neighbors.emplace_back(*position, msg.obstacle.radius, msg.obstacle.height,
                             velocity->head<2>(), msg.id);
    }
    nav_controller.set_neighbors(neighbors);
    if (markers_pub.enabled) {
      markers_pub.publish_neighbors(neighbors, fixed_frame);
    }
  }

  void obstacles_cb(const hl_navigation_msgs::msg::Obstacles &msg) {
    std::vector<Cylinder> obstacles;
    for (auto msg : msg.obstacles) {
      auto position = point_from_msg(msg.top_position, fixed_frame);
      if (!position) return;
      *position -= Vector3(0.0f, 0.0f, msg.height);
      obstacles.emplace_back(*position, msg.radius, msg.height);
    }
    nav_controller.set_static_obstacles(obstacles);
    if (markers_pub.enabled) {
      markers_pub.publish_obstacles(obstacles, fixed_frame);
    }
  }

  HLBehavior *hl_behavior() {
    if (behaviors.count("HL")) {
      return dynamic_cast<HLBehavior *>(behaviors["HL"].get());
    }
    return nullptr;
  }

  ORCABehavior *orca_behavior() {
    if (behaviors.count("ORCA")) {
      return dynamic_cast<ORCABehavior *>(behaviors["ORCA"].get());
    }
    return nullptr;
  }

  void init_params() {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.read_only = true;
    auto kinematic = make_kinematic(
        declare_parameter("kinematic.type", std::string("two_wheeled"),
                          param_desc),
        declare_parameter("kinematic.max_speed", 1.0, param_desc),
        declare_parameter("kinematic.max_angular_speed", 1.0, param_desc),
        declare_parameter("kinematic.axis", 1.0, param_desc));
    const float radius = declare_parameter("radius", 0.0, param_desc);
    for (auto const &p : Behavior::all_behaviors()) {
      behaviors.emplace(p.first, p.second(kinematic, radius));
    }
    should_publish_cmd_stamped =
        declare_parameter("publish_cmd_stamped", false, param_desc);
    fixed_frame = declare_parameter("frame_id", "world", param_desc);
    param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "The name of the obstacle avoidance behavior";
    declare_parameter("behavior", "HL", param_desc);
    declare_parameter("altitude.tau", 1.0);
    declare_parameter("altitude.optimal_speed", 0.1);
    declare_parameter("speed_tolerance", 0.05);
    declare_parameter("optimal_speed", 0.3);
    declare_parameter("optimal_angular_speed", 0.3);
    declare_parameter("rotation_tau", 0.5);
    declare_parameter("horizon", 1.0);
    declare_parameter("safety_margin", 0.1);
    declare_parameter("heading", "idle");
    declare_parameter("hl.tau", 0.5);
    declare_parameter("hl.eta", 0.5);
    declare_parameter("hl.aperture", 3.14);
    declare_parameter("hl.resolution", 30);
    declare_parameter("orca.time_horizon", 1.0);
    declare_parameter("drawing", false);
  }

  void set_behavior(std::string behavior_name) {
    RCLCPP_WARN(get_logger(), "set_behavior to %s", behavior_name.c_str());
    if (behaviors.count(behavior_name)) {
      nav_controller.set_behavior(behaviors[behavior_name]);
    }
    if (behavior_name == "ORCA") {
      dynamic_cast<ORCABehavior *>(behaviors["ORCA"].get())
          ->should_use_effective_center(false);
    }
    if (behavior_name == "ORCA-NH") {
      nav_controller.set_behavior(behaviors["ORCA"]);
      dynamic_cast<ORCABehavior *>(behaviors["ORCA"].get())
          ->should_use_effective_center(true);
    }
  }

  void set_optimal_speed(float value) {
    for (const auto &[_, b] : behaviors) {
      b->set_optimal_speed(value);
    }
  }
  void set_optimal_angular_speed(float value) {
    for (const auto &[_, b] : behaviors) {
      b->set_optimal_angular_speed(value);
    }
  }
  void set_rotation_tau(float value) {
    for (const auto &[_, b] : behaviors) {
      b->set_rotation_tau(value);
    }
  }
  void set_safety_margin(float value) {
    for (const auto &[_, b] : behaviors) {
      b->set_optimal_speed(value);
    }
  }
  void set_horizon(float value) {
    for (const auto &[_, b] : behaviors) {
      b->set_optimal_speed(value);
    }
  }
  void set_heading_behavior(Behavior::Heading value) {
    for (const auto &[_, b] : behaviors) {
      b->set_heading_behavior(value);
    }
  }

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
      const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &param : parameters) {
      std::string name = param.get_name();
      // RCLCPP_INFO(get_logger(), "on_set_parameter %s", name.c_str());
      if (name.find("hl.") != std::string::npos) {
        name = name.substr(3);
        HLBehavior *behavior = hl_behavior();
        if (!behavior) continue;
        if (name == "tau") {
          behavior->set_tau(param.as_double());
        } else if (name == "eta") {
          behavior->set_eta(param.as_double());
        } else if (name == "aperture") {
          behavior->set_aperture(param.as_double());
        } else if (name == "resolution") {
          behavior->set_resolution(param.as_int());
        }
        continue;
      }
      if (name.find("orca.") != std::string::npos) {
        name = name.substr(3);
        ORCABehavior *behavior = orca_behavior();
        if (!behavior) continue;
        if (name == "time_horizon") {
          behavior->set_time_horizon(param.as_double());
        }
        continue;
      }
      if (name == "behavior") {
        set_behavior(param.as_string());
      } else if (name == "optimal_speed") {
        set_optimal_speed(param.as_double());
      } else if (name == "optimal_angular_speed") {
        set_optimal_angular_speed(param.as_double());
      } else if (name == "rotation_tau") {
        set_rotation_tau(param.as_double());
      } else if (name == "safety_margin") {
        set_safety_margin(param.as_double());
      } else if (name == "horizon") {
        set_horizon(param.as_double());
      } else if (name == "heading") {
        // TODO(Jerome): check value
        set_heading_behavior(heading_from_string(param.as_string()));
      } else if (name == "altitude.enabled") {
        nav_controller.should_be_limited_to_2d(!param.as_bool());
      } else if (name == "altitude.tau") {
        nav_controller.set_altitude_tau(param.as_double());
      } else if (name == "altitude.optimal_speed") {
        nav_controller.set_altitude_optimal_speed(param.as_double());
      } else if (name == "drawing") {
        markers_pub.enabled = param.as_bool();
      } else if (name == "speed_tolerance") {
        nav_controller.set_speed_tolerance(param.as_double());
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
