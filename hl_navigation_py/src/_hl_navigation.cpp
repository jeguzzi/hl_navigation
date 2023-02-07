#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "hl_navigation/behavior.h"
#include "hl_navigation/behaviors/HL.h"
#include "hl_navigation/behaviors/ORCA.h"
#include "hl_navigation/behaviors/HRVO.h"
#include "hl_navigation/Controller.h"

using namespace hl_navigation;
namespace py = pybind11;
PYBIND11_MODULE(_hl_navigation, m) {
    py::class_<Behavior>(m, "Behavior")
        // .def_readwrite("type", &Behavior::type)
        // .def_readwrite("radius", &Behavior::radius)
        .def_readwrite("position", &Behavior::position)
        .def_readwrite("angle", &Behavior::angle)
        .def_readwrite("velocity", &Behavior::velocity)
        .def_readwrite("angularSpeed", &Behavior::angularSpeed)
        .def_readwrite("target_position", &Behavior::targetPosition)
        .def_readwrite("target_angle", &Behavior::targetPosition)
        .def_property("target_velocity", &Behavior::get_target_velocity, nullptr)
        .def_readwrite("desired_velocity", &Behavior::desiredVelocity)
        .def_readwrite("desired_twist", &Behavior::desired_twist)
        .def_readwrite("target_twist", &Behavior::target_twist)
        .def_readwrite("desired_wheel_speeds", &Behavior::desired_wheel_speeds)
        .def_readwrite("target_wheel_speeds", &Behavior::target_wheel_speeds)
        .def_property("max_speed", &Behavior::get_max_speed, &Behavior::set_max_speed)
        .def_property("max_angular_speed", &Behavior::get_max_angular_speed,
                      &Behavior::set_max_angular_speed)
        .def_property("horizon", &Behavior::get_horizon, &Behavior::set_horizon)
        .def_property("safety_margin", &Behavior::get_safety_margin, &Behavior::set_safety_margin)
        .def_property("optimal_speed", &Behavior::get_optimal_speed, &Behavior::set_optimal_speed)
        .def_property("optimal_angular_speed", &Behavior::get_optimal_angular_speed,
                      &Behavior::set_optimal_angular_speed)
        .def_property("heading_behavior", &Behavior::get_heading_behavior,
                      &Behavior::set_heading_behavior)
        .def_property("rotation_tau", &Behavior::get_rotation_tau,
                      &Behavior::set_rotation_tau)
        .def_property("radius", &Behavior::get_radius, nullptr)
        .def_property("is_wheeled", &Behavior::is_wheeled, nullptr)
        .def_property("is_omnidirectional", &Behavior::is_omnidirectional, nullptr)
        .def_property("target_velocity", &Behavior::get_target_velocity, nullptr)
        .def("update", &Behavior::update)
        .def("set_desired_twist", &Behavior::set_desired_twist)
        .def("set_wheel_speeds", &Behavior::set_wheel_speeds)
        .def("twist_from_wheel_speeds", &Behavior::twist_from_wheel_speeds)
        .def("wheel_speeds_from_twist", &Behavior::wheel_speeds_from_twist)
        .def("set_neighbors", &Behavior::set_neighbors)
        .def("set_static_obstacles", &Behavior::set_static_obstacles)
        .def("set_line_obstacles", &Behavior::set_line_obstacles);

    m.def("behavior_with_name",  &Behavior::behavior_with_name);

    py::class_<HLBehavior, Behavior>(m, "HLBehavior")
        .def(py::init<agent_type_t, float, float>(), py::arg("type"), py::arg("radius"),
             py::arg("wheel_axis") = 0.0)
        .def("get_distances", &HLBehavior::getDistances)
        .def("distance_to_segment", &HLBehavior::distance_to_segment)
        .def_readwrite("resolution", &HLBehavior::resolution)
        .def_readwrite("aperture", &HLBehavior::aperture)
        .def("set_tau", &HLBehavior::setTau);

    py::class_<ORCABehavior, Behavior>(m, "ORCABehavior")
        .def(py::init<agent_type_t, float, float>(), py::arg("type"), py::arg("radius"),
             py::arg("wheel_axis") = 0.0)
        .def_property("time_horizon", &ORCABehavior::getTimeHorizon, &ORCABehavior::setTimeHorizon)
        .def_property("control_step", &ORCABehavior::getTimeStep, &ORCABehavior::setTimeStep);

    py::class_<HRVOBehavior, Behavior>(m, "HRVOBehavior")
        .def(py::init<agent_type_t, float, float>(), py::arg("type"), py::arg("radius"),
             py::arg("wheel_axis") = 0.0);

    py::class_<Twist2D>(m, "Twist2D")
        .def(py::init<float, float, float>())
        .def_readwrite("longitudinal", &Twist2D::longitudinal)
        .def_readwrite("lateral", &Twist2D::lateral)
        .def_readwrite("angular", &Twist2D::angular)
        .def("__repr__", [](const Twist2D &v) {
            return "(" + std::to_string(v.longitudinal) + ", " + std::to_string(v.lateral) +
                   ", " + std::to_string(v.angular) + ")";});

    py::class_<Disc>(m, "Disc")
        .def(py::init<Vector2, float, float, Vector2>(),
             py::arg("center"), py::arg("radius"),
             py::arg("social_margin") = 0.0, py::arg("velocity") = Vector2(0.0, 0.0))
        .def_readonly("center", &Disc::position)
        .def_readonly("radius", &Disc::radius)
        .def_readonly("social_margin", &Disc::social_margin)
        .def_readonly("velocity", &Disc::velocity);

    py::class_<LineSegment>(m, "LineSegment")
        .def(py::init<Vector2, Vector2>(),
             py::arg("p1"), py::arg("p2"))
        .def_readonly("p1", &LineSegment::p1)
        .def_readonly("p2", &LineSegment::p2)
        .def_readonly("e1", &LineSegment::e1)
        .def_readonly("e2", &LineSegment::e2)
        .def_readonly("length", &LineSegment::length);

    py::class_<Cylinder>(m, "Cylinder")
        .def(py::init<Vector3, float, float, float, Vector3>(),
             py::arg("position"), py::arg("radius"), py::arg("height") = -1.0,
             py::arg("social_margin") = 0.0, py::arg("velocity") = Vector2(0.0, 0.0))
        .def_readonly("center", &Cylinder::position)
        .def_readonly("radius", &Cylinder::radius)
        .def_readonly("social_margin", &Cylinder::social_margin)
        .def_readonly("velocity", &Cylinder::velocity);

    py::enum_<agent_type_t>(m, "AgentType")
        .value("HOLONOMIC", agent_type_t::HOLONOMIC)
        .value("TWO_WHEELED", agent_type_t::TWO_WHEELED)
        .value("HEAD", agent_type_t::HEAD)
        .value("FOUR_WHEELED_OMNI", agent_type_t::FOUR_WHEELED_OMNI)
        .export_values();

    py::enum_<heading_t>(m, "HeadingBehavior")
        .value("IDLE", heading_t::IDLE)
        .value("TARGET_POINT", TARGET_POINT)
        .value("TARGET_ANGLE", TARGET_ANGLE)
        .value("DESIRED_ANGLE", DESIRED_ANGLE)
        .export_values();

    py::enum_<Controller::State>(m, "ControllerState")
        .value("MOVE", Controller::State::MOVE)
        .value("TURN", Controller::State::TURN)
        .value("BRAKING", Controller::State::BRAKING)
        .value("IDLE", Controller::State::IDLE)
        .export_values();

    py::class_<Controller>(m, "Controller")
        .def(py::init<>())
        .def_readwrite("state", &Controller::state)
        .def_readwrite("behavior", &Controller::behavior)
        .def("set_target_point",
             py::overload_cast<const Vector3 &>(&Controller::set_target_point))
        .def("set_target_point",
             py::overload_cast<const Vector2 &>(&Controller::set_target_point))
        .def("set_target_pose",
             py::overload_cast<const Vector3 &, Radians>(&Controller::set_target_pose))
        .def("set_target_pose",
              py::overload_cast<const Vector2 &, Radians>(&Controller::set_target_pose))
        .def("set_pose",
             py::overload_cast<const Vector3 &, Radians>(&Controller::set_pose))
        .def("set_pose",
             py::overload_cast<const Vector2 &, Radians>(&Controller::set_pose))
        .def("update", &Controller::update)
        .def("set_neighbors",
             py::overload_cast<const std::vector<Disc> &>(&Controller::set_neighbors))
        .def("set_neighbors",
             py::overload_cast<const std::vector<Cylinder> &>(&Controller::set_neighbors))
        .def("stop", &Controller::stop)
        .def_readwrite("speed_tolerance", &Controller::speed_tolerance)
        .def_readwrite("angle_tolerance", &Controller::speed_tolerance)
        .def_readwrite("distance_tolerance", &Controller::distance_tolerance);
}
