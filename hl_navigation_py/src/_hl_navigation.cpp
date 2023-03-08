#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <vector>

#include "hl_navigation/behavior.h"
#include "hl_navigation/behaviors/Dummy.h"
#include "hl_navigation/behaviors/HL.h"
#include "hl_navigation/behaviors/HRVO.h"
#include "hl_navigation/behaviors/ORCA.h"
#include "hl_navigation/cached_collision_computation.h"
#include "hl_navigation/collision_computation.h"
#include "hl_navigation/common.h"
#include "hl_navigation/controller.h"
#include "hl_navigation/kinematic.h"

using namespace hl_navigation;
namespace py = pybind11;
PYBIND11_MODULE(_hl_navigation, m) {
  py::class_<Twist2>(m, "Twist2")
      .def(py::init<Vector2, float, bool>(), py::arg("velocity"),
           py::arg("angular_speed") = 0.0f, py::arg("relative") = false)
      .def_readwrite("velocity", &Twist2::velocity)
      .def_readwrite("angular_speed", &Twist2::angular_speed)
      .def_readwrite("relative", &Twist2::relative)
      .def("rotate", &Twist2::rotate)
      .def("__repr__", [](const Twist2 &v) {
        return ("Twist2[" + std::to_string(v.relative) + "]<" +
                std::to_string(v.velocity.x()) + ", " +
                std::to_string(v.velocity.y()) + ", " +
                std::to_string(v.angular_speed) + ">");
      });

  py::class_<Pose2>(m, "Pose2")
      .def(py::init<Vector2, float>(), py::arg("position"),
           py::arg("orientation") = 0.0f)
      .def_readwrite("position", &Pose2::position)
      .def_readwrite("orientation", &Pose2::orientation)
      .def("rotate", &Pose2::rotate)
      .def("integrate", &Pose2::integrate)
      .def("__repr__", [](const Pose2 &v) {
        return ("Pose2<" + std::to_string(v.position.x()) + ", " +
                std::to_string(v.position.y()) + ", " +
                std::to_string(v.orientation) + ">");
      });

  py::class_<Disc>(m, "Disc")
      .def(py::init<Vector2, float, float, Vector2>(), py::arg("position"),
           py::arg("radius"), py::arg("social_margin") = 0.0,
           py::arg("velocity") = Vector2(0.0, 0.0))
      .def_readonly("position", &Disc::position)
      .def_readonly("radius", &Disc::radius)
      .def_readonly("social_margin", &Disc::social_margin)
      .def_readonly("velocity", &Disc::velocity);

  py::class_<LineSegment>(m, "LineSegment")
      .def(py::init<Vector2, Vector2>(), py::arg("p1"), py::arg("p2"))
      .def_readonly("p1", &LineSegment::p1)
      .def_readonly("p2", &LineSegment::p2)
      .def_readonly("e1", &LineSegment::e1)
      .def_readonly("e2", &LineSegment::e2)
      .def_readonly("length", &LineSegment::length);

  py::class_<Kinematic, std::shared_ptr<Kinematic>>(m, "Kinematic")
      .def_property("max_speed", &Kinematic::get_max_speed,
                    &Kinematic::set_max_speed)
      .def_property("max_angular_speed", &Kinematic::get_max_angular_speed,
                    &Kinematic::set_max_angular_speed)
      .def_property("is_wheeled", &Kinematic::is_wheeled, nullptr)
      .def_property("dof", &Kinematic::dof, nullptr)
      .def("feasible", &Kinematic::feasible);

  py::class_<Holonomic, Kinematic, std::shared_ptr<Holonomic>>(m, "Holonomic")
      .def(py::init<float, float>(), py::arg("max_speed"),
           py::arg("max_angular_speed"));

  py::class_<Forward, Kinematic, std::shared_ptr<Forward>>(m, "Forward")
      .def(py::init<float, float>(), py::arg("max_speed"),
           py::arg("max_angular_speed"));

  py::class_<Wheeled, Kinematic, std::shared_ptr<Wheeled>>(m, "Wheeled")
      .def_property("axis", &Wheeled::get_axis, nullptr)
      .def("twist", &Wheeled::twist)
      .def("wheel_speeds", &Wheeled::twist);

  py::class_<TwoWheeled, Wheeled, std::shared_ptr<TwoWheeled>>(m, "TwoWheeled")
      .def(py::init<float, float>(), py::arg("max_speed"), py::arg("axis"));

  py::class_<FourWheeled, Wheeled, std::shared_ptr<FourWheeled>>(m,
                                                                 "FourWheeled")
      .def(py::init<float, float>(), py::arg("max_speed"), py::arg("axis"));

  py::enum_<Behavior::Heading>(m, "BehaviorHeading")
      .value("idle", Behavior::Heading::idle)
      .value("target_point", Behavior::Heading::target_point)
      .value("target_angle", Behavior::Heading::target_angle)
      .value("velocity", Behavior::Heading::velocity)
      .export_values();

  py::enum_<Behavior::Mode>(m, "BehaviorMode")
      .value("move", Behavior::Mode::move)
      .value("turn", Behavior::Mode::turn)
      .value("stop", Behavior::Mode::stop)
      .value("follow", Behavior::Mode::follow)
      .export_values();

  py::class_<Behavior, std::shared_ptr<Behavior>>(m, "Behavior")
      .def_property("kinematic", &Behavior::get_kinematic, nullptr)
      .def_property("radius", &Behavior::get_radius, &Behavior::set_radius)
      .def_property("max_speed", &Behavior::get_max_speed,
                    &Behavior::set_max_speed)
      .def_property("max_angular_speed", &Behavior::get_max_angular_speed,
                    &Behavior::set_max_angular_speed)

      .def_property("optimal_speed", &Behavior::get_optimal_speed,
                    &Behavior::set_optimal_speed)
      .def_property("optimal_angular_speed",
                    &Behavior::get_optimal_angular_speed,
                    &Behavior::set_optimal_angular_speed)
      .def_property("rotation_tau", &Behavior::get_rotation_tau,
                    &Behavior::set_rotation_tau)
      .def_property("safety_margin", &Behavior::get_safety_margin,
                    &Behavior::set_safety_margin)
      .def_property("horizon", &Behavior::get_horizon, &Behavior::set_horizon)

      .def_property("pose", &Behavior::get_pose, &Behavior::set_pose)
      .def_property("position", &Behavior::get_position,
                    &Behavior::set_position)
      .def_property("orientation", &Behavior::get_orientation,
                    &Behavior::set_orientation)
      .def_property(
          "twist", [](const Behavior &self) { return self.get_twist(); },
          &Behavior::set_twist)
      .def("get_twist", &Behavior::get_twist, py::arg("relative") = false)
      .def_property(
          "velocity", [](const Behavior &self) { return self.get_velocity(); },
          [](Behavior &self, const Vector2 v) { return self.set_velocity(v); })
      .def("get_velocity", &Behavior::get_velocity, py::arg("relative") = false)
      .def("set_velocity", &Behavior::set_velocity, py::arg("velocity"),
           py::arg("relative") = false)
      .def_property("angular_speed", &Behavior::get_angular_speed,
                    &Behavior::set_angular_speed)
      .def_property("wheel_speeds", &Behavior::get_wheel_speeds,
                    &Behavior::set_wheel_speeds)
      .def_property(
          "actuated_twist",
          [](const Behavior &self) { return self.get_actuated_twist(); },
          &Behavior::set_actuated_twist)
      .def("get_actuated_twist", &Behavior::get_actuated_twist,
           py::arg("relative") = false)
      .def_property(
          "velocity", [](const Behavior &self) { return self.get_velocity(); },
          [](Behavior &self, const Vector2 v) { return self.set_velocity(v); })
      .def_property("actuated_wheel_speeds",
                    &Behavior::get_actuated_wheel_speeds, nullptr)
      .def("actuate",
           py::overload_cast<const Twist2 &, float>(&Behavior::actuate))
      .def("actuate", py::overload_cast<float>(&Behavior::actuate))

      .def_property("heading_behavior", &Behavior::get_heading_behavior,
                    &Behavior::set_heading_behavior)
      .def_property("target_position", &Behavior::get_target_position,
                    &Behavior::set_target_position)
      .def_property("target_orientation", &Behavior::get_target_orientation,
                    &Behavior::set_target_orientation)
      .def_property("target_velocity", &Behavior::get_target_velocity,
                    &Behavior::set_target_velocity)
      .def_property("target_angular_speed", &Behavior::get_target_angular_speed,
                    &Behavior::set_target_angular_speed)

      .def("cmd_twist",
           py::overload_cast<float, bool, Behavior::Mode, bool>(
               &Behavior::cmd_twist),
           py::arg("time_step"), py::arg("relative"),
           py::arg("mode") = Behavior::Mode::move,
           py::arg("set_as_actuated") = true)

      .def("to_frame", &Behavior::to_frame)
      .def("wheel_speeds_from_twist", &Behavior::wheel_speeds_from_twist)
      .def("twist_from_wheel_speeds", &Behavior::twist_from_wheel_speeds);

  m.def("behavior_with_name", &Behavior::behavior_with_name);
  m.def("behavior_names", &Behavior::behavior_names);

  py::class_<GeometricState, std::shared_ptr<GeometricState>>(m,
                                                              "GeometricState")
      .def_property("neighbors", &GeometricState::get_neighbors,
                    &GeometricState::set_neighbors)
      .def_property("static_obstacles", &GeometricState::get_static_obstacles,
                    &GeometricState::set_static_obstacles)
      .def_property("line_obstacles", &GeometricState::get_line_obstacles,
                    &GeometricState::set_line_obstacles);

  py::class_<HLBehavior, GeometricState, Behavior, std::shared_ptr<HLBehavior>>(
      m, "HLBehavior")
      .def(py::init<std::shared_ptr<Kinematic>, float>(), py::arg("kinematic"),
           py::arg("radius"))
      .def_property("eta", &HLBehavior::get_eta, &HLBehavior::set_eta)
      .def_property("tau", &HLBehavior::get_tau, &HLBehavior::set_tau)
      .def_property("aperture", &HLBehavior::get_aperture,
                    &HLBehavior::set_aperture)
      .def_property("resolution", &HLBehavior::get_resolution,
                    &HLBehavior::set_resolution)
      .def_property("angular_resolution", &HLBehavior::get_angular_resolution,
                    nullptr)
      .def_property("collision_distances", &HLBehavior::get_collision_distance,
                    nullptr);

  py::class_<ORCABehavior, GeometricState, Behavior,
             std::shared_ptr<ORCABehavior>>(m, "ORCABehavior")
      .def(py::init<std::shared_ptr<Kinematic>, float>(), py::arg("kinematic"),
           py::arg("radius"))
      .def_property("time_horizon", &ORCABehavior::get_time_horizon,
                    &ORCABehavior::set_time_horizon)
      .def_property("is_using_effective_center",
                    &ORCABehavior::is_using_effective_center,
                    &ORCABehavior::should_use_effective_center);

  py::class_<HRVOBehavior, GeometricState, Behavior,
             std::shared_ptr<HRVOBehavior>>(m, "HRVOBehavior")
      .def(py::init<std::shared_ptr<Kinematic>, float>(), py::arg("kinematic"),
           py::arg("radius"));

  py::class_<DummyBehavior, Behavior, std::shared_ptr<DummyBehavior>>(
      m, "DummyBehavior")
      .def(py::init<std::shared_ptr<Kinematic>, float>(), py::arg("kinematic"),
           py::arg("radius"));

  py::enum_<Action::State>(m, "ActionState")
      .value("idle", Action::State::idle)
      .value("running", Action::State::running)
      .value("failure", Action::State::failure)
      .value("success", Action::State::success)
      .export_values();

  py::class_<Action, std::shared_ptr<Action>>(m, "Action")
      .def_readonly("state", &Action::state)
      .def_property("done", &Action::done, nullptr)
      .def_property("running", &Action::done, nullptr)
      .def("abort", &Action::abort)
      .def_readwrite("running_cb", &Action::running_cb)
      .def_readwrite("done_cb", &Action::done_cb);

  py::class_<Controller>(m, "Controller")
      .def(py::init<std::shared_ptr<Behavior>, float, float>(),
           py::arg("behavior") = nullptr,
           py::arg("compute_relative_twist") = true,
           py::arg("set_twist_as_automatically_actuated") = true)
      .def_property("state", &Controller::get_state, nullptr)
      .def_property("idle", &Controller::idle, nullptr)
      .def_property("behavior", &Controller::get_behavior,
                    &Controller::set_behavior)
      .def_property("speed_tolerance", &Controller::get_speed_tolerance,
                    &Controller::set_speed_tolerance)
      .def("go_to_position", &Controller::go_to_position)
      .def("go_to_pose", &Controller::go_to_pose)
      .def("follow_point", &Controller::follow_point)
      .def("follow_pose", &Controller::follow_pose)
      .def("follow_velocity", &Controller::follow_velocity)
      .def("follow_twist", &Controller::follow_twist)
      .def("update", &Controller::update)
      .def("set_cmd_cb", &Controller::set_cmd_cb)
      .def("stop", &Controller::stop);

  py::class_<CollisionComputation>(m, "CollisionComputation")
      .def(py::init<>())
      .def("setup",
           py::overload_cast<Pose2, float, const std::vector<LineSegment> &,
                             const std::vector<Disc> &,
                             const std::vector<Disc> &>(
               &CollisionComputation::setup))
      .def("static_free_distance", &CollisionComputation::static_free_distance,
           py::arg("angle"), py::arg("max_distance"),
           py::arg("include_neighbors") = true)

      .def("dynamic_free_distance",
           &CollisionComputation::dynamic_free_distance, py::arg("angle"),
           py::arg("max_distance"), py::arg("speed"))
      .def("get_free_distance_for_sector",
           &CollisionComputation::get_free_distance_for_sector, py::arg("from"),
           py::arg("length"), py::arg("resolution"), py::arg("max_distance"),
           py::arg("dynamic"), py::arg("speed") = 0.0f);

  py::class_<CachedCollisionComputation, CollisionComputation>(
      m, "CachedCollisionComputation")
      .def(py::init<>())
      .def_property("resolution", &CachedCollisionComputation::get_resolution,
                    &CachedCollisionComputation::set_resolution)
      .def_property("min_angle", &CachedCollisionComputation::get_min_angle,
                    &CachedCollisionComputation::set_min_angle)
      .def_property("length", &CachedCollisionComputation::get_length,
                    &CachedCollisionComputation::set_length)
      .def_property("max_distance",
                    &CachedCollisionComputation::get_max_distance,
                    &CachedCollisionComputation::set_max_distance)
      .def_property("speed", &CachedCollisionComputation::get_speed,
                    &CachedCollisionComputation::set_speed)
      .def("get_free_distance", &CachedCollisionComputation::get_free_distance,
           py::arg("dynamic"));

  // py::class_<Cylinder>(m, "Cylinder")
  //     .def(py::init<Vector3, float, float, float, Vector3>(),
  //          py::arg("position"), py::arg("radius"), py::arg("height") = -1.0,
  //          py::arg("social_margin") = 0.0, py::arg("velocity") = Vector2(0.0,
  //          0.0))
  //     .def_readonly("center", &Cylinder::position)
  //     .def_readonly("radius", &Cylinder::radius)
  //     .def_readonly("social_margin", &Cylinder::social_margin)
  //     .def_readonly("velocity", &Cylinder::velocity);
}
