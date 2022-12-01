#include <pybind11/pybind11.h>
#include <pybind11/operators.h>

#include "Agent.h"
#include "HLAgent.h"
#include "ORCAAgent.h"
#include "HRVOAgent.h"
#include "Controller.h"

namespace py = pybind11;
PYBIND11_MODULE(hl_navigation_py, m) {
    py::class_<Agent>(m, "Agent")
        .def_readwrite("type", &Agent::type)
        .def_readwrite("radius", &Agent::radius)
        .def_readwrite("position", &Agent::position)
        .def_readwrite("angle", &Agent::angle)
        .def_readwrite("targetPosition", &Agent::targetPosition)
        .def_readwrite("velocity", &Agent::velocity)
        .def_readwrite("desiredVelocity", &Agent::desiredVelocity)
        .def_readwrite("desiredLinearSpeed", &Agent::desiredLinearSpeed)
        .def_readwrite("desiredAngularSpeed", &Agent::desiredAngularSpeed)
        .def_readwrite("axisLength", &Agent::axisLength)
        .def("addObstacleAtPoint",
             static_cast<void (Agent::*)(CVector2, float, float)>(&Agent::addObstacleAtPoint))
        .def("addAgentAtPoint",
             static_cast<void (Agent::*)(CVector2, CVector2, float, float)>(
              &Agent::addObstacleAtPoint))
        .def("setMaxSpeed", &Agent::setMaxSpeed)
        .def("setMaxAngularSpeed", &Agent::setMaxAngularSpeed)
        .def("setMaxRotationSpeed", &Agent::setMaxRotationSpeed)
        .def("setHorizon", &Agent::setHorizon)
        .def("setSafetyMargin", &Agent::setSafetyMargin)
        .def("clearObstacles", &Agent::clearObstacles)
        .def("updateDesiredVelocity", &Agent::updateDesiredVelocity)
        .def("updateVelocity", &Agent::updateVelocity)
        .def("setOptimalSpeed", &Agent::setOptimalSpeed)
        .def("setOptimalRotationSpeed", &Agent::setOptimalRotationSpeed)
        .def("velocity_from_wheel_speed", &Agent::velocity_from_wheel_speed)
        .def("wheel_speed_from_velocity", &Agent::wheel_speed_from_velocity);

    m.def("agent_with_name",  &Agent::agent_with_name);

    py::class_<HLAgent, Agent>(m, "HLAgent")
        .def(py::init<>());

    py::class_<ORCAAgent, Agent>(m, "ORCAAgent")
        .def(py::init<>());

    py::class_<HRVOAgent, Agent>(m, "HRVOAgent")
        .def(py::init<>());

    py::class_<CVector2>(m, "CVector2")
        .def(py::init<float, float>())
        .def("get_x", &CVector2::GetX)
        .def("get_y", &CVector2::GetY)
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
        .def("value", &CRadians::GetValue);

    py::enum_<agentType>(m, "AgentType")
        .value("HOLONOMIC", agentType::HOLONOMIC)
        .value("TWO_WHEELED", agentType::TWO_WHEELED)
        .value("HEAD", agentType::HEAD)
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
        .def("setTargetPoint", &Controller::setTargetPoint)
        .def("update", &Controller::update)
        .def_readwrite("minimalSpeed", &Controller::minimalSpeed)
        .def_readwrite("minDeltaDistance", &Controller::minDeltaDistance);
}
