#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include "Agent.h"
#include "HLAgent.h"
#include "ORCAAgent.h"
#include "HRVOAgent.h"
#include "Controller.h"

namespace py = pybind11;
PYBIND11_MODULE(hl_navigation_py, m) {
    py::class_<Agent>(m, "Agent")
        // .def_readwrite("type", &Agent::type)
        // .def_readwrite("radius", &Agent::radius)
        .def_readwrite("position", &Agent::position)
        .def_readwrite("angle", &Agent::angle)
        .def_readwrite("velocity", &Agent::velocity)
        .def_readwrite("angularSpeed", &Agent::angularSpeed)
        .def_readwrite("target_position", &Agent::targetPosition)
        .def_readwrite("target_angle", &Agent::targetPosition)
        .def_property("target_velocity", &Agent::get_target_velocity, nullptr)
        .def_readwrite("desired_velocity", &Agent::desiredVelocity)
        .def_readwrite("desired_twist", &Agent::desired_twist)
        .def_readwrite("target_twist", &Agent::target_twist)
        .def_readwrite("desired_wheel_speeds", &Agent::desired_wheel_speeds)
        .def_readwrite("target_wheel_speeds", &Agent::target_wheel_speeds)
        .def_property("max_speed", &Agent::get_max_speed, &Agent::set_max_speed)
        .def_property("max_angular_speed", &Agent::get_max_angular_speed,
                      &Agent::set_max_angular_speed)
        .def_property("horizon", &Agent::get_horizon, &Agent::set_horizon)
        .def_property("safety_margin", &Agent::get_safety_margin, &Agent::set_safety_margin)
        .def_property("optimal_speed", &Agent::get_optimal_speed, &Agent::set_optimal_speed)
        .def_property("optimal_angular_speed", &Agent::get_optimal_angular_speed,
                      &Agent::set_optimal_angular_speed)
        .def_property("heading_behavior", &Agent::get_heading_behavior,
                      &Agent::set_heading_behavior)
        .def_property("rotation_tau", &Agent::get_rotation_tau,
                      &Agent::set_rotation_tau)
        .def_property("is_wheeled", &Agent::is_wheeled, nullptr)
        .def_property("is_omnidirectional", &Agent::is_omnidirectional, nullptr)
        .def_property("target_velocity", &Agent::get_target_velocity, nullptr)
        .def("update", &Agent::update)
        .def("set_desired_twist", &Agent::set_desired_twist)
        .def("set_wheel_speeds", &Agent::set_wheel_speeds)
        .def("twist_from_wheel_speeds", &Agent::twist_from_wheel_speeds)
        .def("wheel_speeds_from_twist", &Agent::wheel_speeds_from_twist)
        .def("set_neighbors", &Agent::set_neighbors)
        .def("set_static_obstacles", &Agent::set_static_obstacles);

    m.def("agent_with_name",  &Agent::agent_with_name);

    py::class_<HLAgent, Agent>(m, "HLAgent")
        .def(py::init<agent_type_t, float, float>(), py::arg("type"), py::arg("radius"),
             py::arg("wheel_axis") = 0.0);

    py::class_<ORCAAgent, Agent>(m, "ORCAAgent")
        .def(py::init<agent_type_t, float, float>(), py::arg("type"), py::arg("radius"),
             py::arg("wheel_axis") = 0.0)
        .def("set_time_horizon", &ORCAAgent::setTimeHorizon);

    py::class_<HRVOAgent, Agent>(m, "HRVOAgent")
        .def(py::init<agent_type_t, float, float>(), py::arg("type"), py::arg("radius"),
             py::arg("wheel_axis") = 0.0);

    py::class_<CVector2>(m, "CVector2")
        .def(py::init<float, float>())
        .def_property("x", &CVector2::GetX, nullptr)
        .def_property("y", &CVector2::GetY, nullptr)
        .def(py::self + py::self)
        .def(py::self += py::self)
        .def(py::self *= float())
        .def(float() * py::self)
        .def(py::self * float())
        .def(-py::self)
        .def("__repr__", [](const CVector2 &v) {
            return "(" + std::to_string(v.GetX()) + ", " + std::to_string(v.GetY()) + ")";});

    py::class_<CRadians>(m, "CRadians")
        .def(py::init<float>())
        .def(py::self + py::self)
        .def(py::self += py::self)
        .def(py::self * float())
        .def(py::self *= float())
        .def(float() * py::self)
        .def(-py::self)
        .def("value", &CRadians::GetValue);

    py::class_<Twist2D>(m, "Twist2D")
        .def(py::init<float, float, float>())
        .def_readwrite("longitudinal", &Twist2D::longitudinal)
        .def_readwrite("lateral", &Twist2D::lateral)
        .def_readwrite("angular", &Twist2D::angular)
        .def("__repr__", [](const Twist2D &v) {
            return "(" + std::to_string(v.longitudinal) + ", " + std::to_string(v.lateral) +
                   ", " + std::to_string(v.angular) + ")";});

    py::class_<Disc>(m, "Disc")
        .def(py::init<CVector2, float, float, CVector2>(),
             py::arg("position"), py::arg("radius"),
             py::arg("social_margin") = 0.0, py::arg("velocity") = CVector2(0.0, 0.0));

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
        .def_readwrite("agent", &Controller::agent)
        .def("set_target_point", &Controller::set_target_point)
        .def("set_target_pose", &Controller::set_target_point)
        .def("update", &Controller::update)
        .def("stop", &Controller::stop)
        .def_readwrite("speed_tolerance", &Controller::speed_tolerance)
        .def_readwrite("angle_tolerance", &Controller::speed_tolerance)
        .def_readwrite("distance_tolerance", &Controller::distance_tolerance);
}
