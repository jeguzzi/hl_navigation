#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <vector>

#include "docstrings.h"
#include "hl_navigation/behavior.h"
#include "hl_navigation/behaviors/HL.h"
#include "hl_navigation/behaviors/HRVO.h"
#include "hl_navigation/behaviors/ORCA.h"
#include "hl_navigation/behaviors/dummy.h"
#include "hl_navigation/cached_collision_computation.h"
#include "hl_navigation/collision_computation.h"
#include "hl_navigation/common.h"
#include "hl_navigation/controller.h"
#include "hl_navigation/kinematics.h"
#include "hl_navigation/plugins.h"
#include "hl_navigation/yaml/core.h"
#include "hl_navigation/yaml/yaml.h"
#include "hl_navigation_py/register.h"
#include "hl_navigation_py/yaml.h"

using namespace hl_navigation;
namespace py = pybind11;

template <typename T>
static std::string to_string(const T &value) {
  return std::to_string(value);
}

template <>
std::string to_string(const Vector2 &value) {
  return "(" + std::to_string(value[0]) + ", " + std::to_string(value[1]) + ")";
}

template <>
std::string to_string(const bool &value) {
  return value ? "True" : "False";
}

template <>
std::string to_string(const Pose2 &value) {
  return "Pose2(" + to_string(value.position) + ", " +
         std::to_string(value.orientation) + ")";
}

template <>
std::string to_string(const Twist2 &value) {
  return "Twist2(" + to_string(value.velocity) + ", " +
         std::to_string(value.angular_speed) + ", " +
         to_string(value.relative) + ")";
}

template <>
std::string to_string(const Disc &value) {
  return "Disc(" + to_string(value.position) + ", " +
         std::to_string(value.radius) + ")";
}

template <>
std::string to_string(const Neighbor &value) {
  return "Neighbor(" + to_string<Disc>(value) + ", " +
         to_string(value.velocity) + ", " + std::to_string(value.id) + ")";
}

template <>
std::string to_string(const LineSegment &value) {
  return "LineSegment(" + to_string(value.p1) + ", " + to_string(value.p2) +
         ")";
}

class PyBehavior : public Behavior, virtual public PyHasRegister<Behavior> {
 public:
  /* Inherit the constructors */
  using Behavior::Behavior;
  using Native = Behavior;

  /* Trampolines (need one for each virtual function) */
  Twist2 cmd_twist(float time_step, Mode mode, bool relative,
                   bool set_as_actuated) override {
    PYBIND11_OVERRIDE(Twist2, Behavior, cmd_twist, time_step, mode, relative,
                      set_as_actuated);
  }
  Vector2 compute_desired_velocity(float time_step) override {
    PYBIND11_OVERRIDE(Vector2, Behavior, compute_desired_velocity, time_step);
  }
  Twist2 twist_towards_velocity(const Vector2 &absolute_velocity,
                                bool relative) override {
    PYBIND11_OVERRIDE(Twist2, Behavior, twist_towards_velocity,
                      absolute_velocity, relative);
  }
  Twist2 cmd_twist_towards_target(float time_step, bool relative) override {
    PYBIND11_OVERRIDE(Twist2, Behavior, cmd_twist_towards_target, time_step,
                      relative);
  }
  Twist2 cmd_twist_towards_target_orientation(float time_step,
                                              bool relative) override {
    PYBIND11_OVERRIDE(Twist2, Behavior, cmd_twist_towards_target_orientation,
                      time_step, relative);
  }
  Twist2 cmd_twist_towards_stopping(float time_step, bool relative) override {
    PYBIND11_OVERRIDE(Twist2, Behavior, cmd_twist_towards_stopping, time_step,
                      relative);
  }

  // HACK(J): should not happen but as of now, it can be that get_type returns
  // ''
  const Properties &get_properties() const override {
    const std::string t = get_type();
    if (type_properties().count(t)) {
      return type_properties().at(t);
    }
    return Native::get_properties();
  };

  py::object py_kinematics;

  void set_kinematics_py(const py::object &value) {
    py_kinematics = value;
    set_kinematics(value.cast<std::shared_ptr<Kinematics>>());
  }
};

class PyKinematics : public Kinematics,
                     virtual public PyHasRegister<Kinematics> {
 public:
  /* Inherit the constructors */
  using Kinematics::Kinematics;
  using Native = Kinematics;

  /* Trampolines (need one for each virtual function) */
  Twist2 feasible(const Twist2 &twist) const override {
    PYBIND11_OVERRIDE_PURE(Twist2, Kinematics, feasible, twist);
  }
  bool is_wheeled() const override {
    PYBIND11_OVERRIDE_PURE(bool, Kinematics, is_wheeled);
  }
  unsigned dof() const override {
    PYBIND11_OVERRIDE_PURE(bool, Kinematics, dof);
  }

  float get_max_angular_speed() const override {
    PYBIND11_OVERRIDE(float, Kinematics, get_max_angular_speed);
  }

  // HACK(J): should not happen but as of now, it can be that get_type returns
  // ''
  const Properties &get_properties() const override {
    const std::string t = get_type();
    if (type_properties().count(t)) {
      return type_properties().at(t);
    }
    return Native::get_properties();
  };
};

PYBIND11_MODULE(_hl_navigation, m) {
  py::options options;
  // options.disable_function_signatures();
  options.disable_enum_members_docstring();

  declare_register<Behavior>(m, "Behavior");
  declare_register<Kinematics>(m, "Kinematics");

  py::class_<Property>(m, "Property", DOC(hl_navigation_Property))
      .def_readonly("description", &Property::description,
                    DOC(hl_navigation_Property, description))
      .def_readonly("owner_type_name", &Property::owner_type_name,
                    DOC(hl_navigation_Property, owner_type_name))
      .def_readonly("default_value", &Property::default_value,
                    DOC(hl_navigation_Property, default_value))
      .def_readonly("type_name", &Property::type_name,
                    DOC(hl_navigation_Property, type_name));

  py::class_<HasProperties, std::shared_ptr<HasProperties>>(
      m, "HasProperties", DOC(hl_navigation_HasProperties))
      .def("get", &HasProperties::get, py::arg("name"),
           DOC(hl_navigation_HasProperties, get))
      .def("set", &HasProperties::set, py::arg("name"), py::arg("value"),
           DOC(hl_navigation_HasProperties, set))
      .def_property("properties", &HasProperties::get_properties, nullptr,
                    DOC(hl_navigation_HasProperties, property_properties));

  py::class_<Twist2>(m, "Twist2", DOC(hl_navigation_Twist2))
      .def(py::init<Vector2, float, bool>(), py::arg("velocity"),
           py::arg("angular_speed") = 0.0f, py::arg("relative") = false,
           DOC(hl_navigation_Twist2, Twist2))
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def_readwrite("velocity", &Twist2::velocity,
                     DOC(hl_navigation_Twist2, velocity))
      .def_readwrite("angular_speed", &Twist2::angular_speed,
                     DOC(hl_navigation_Twist2, angular_speed))
      .def_readwrite("relative", &Twist2::relative,
                     DOC(hl_navigation_Twist2, relative))
      .def("rotate", &Twist2::rotate, py::arg("angle"),
           DOC(hl_navigation_Twist2, rotate))
      .def("__repr__", &to_string<Twist2>);

  py::class_<Pose2>(m, "Pose2", DOC(hl_navigation_Pose2))
      .def(py::init<Vector2, float>(), py::arg("position"),
           py::arg("orientation") = 0.0f, DOC(hl_navigation_Pose2, Pose2))
      .def_readwrite("position", &Pose2::position,
                     DOC(hl_navigation_Pose2, position))
      .def_readwrite("orientation", &Pose2::orientation,
                     DOC(hl_navigation_Pose2, orientation))
      .def("rotate", &Pose2::rotate, py::arg("angle"),
           DOC(hl_navigation_Pose2, rotate))
      .def("integrate", &Pose2::integrate, py::arg("twist"), py::arg("time"),
           DOC(hl_navigation_Pose2, rotate))
      .def("__repr__", &to_string<Pose2>);

  py::class_<Disc>(m, "Disc", DOC(hl_navigation_Disc))
      .def(py::init<Vector2, float>(), py::arg("position"), py::arg("radius"),
           DOC(hl_navigation_Disc, Disc))
      .def_readwrite("position", &Disc::position,
                     DOC(hl_navigation_Disc, position))
      .def_readwrite("radius", &Disc::radius, DOC(hl_navigation_Disc, radius))
      .def("__repr__", &to_string<Disc>);

  py::class_<Neighbor, Disc>(m, "Neighbor", DOC(hl_navigation_Neighbor))
      .def(py::init<Vector2, float, Vector2, int>(), py::arg("position"),
           py::arg("radius"), py::arg("velocity") = Vector2(0.0, 0.0),
           py::arg("id") = 0, DOC(hl_navigation_Neighbor, Neighbor))
      .def_readwrite("velocity", &Neighbor::velocity,
                     DOC(hl_navigation_Neighbor, velocity))
      .def_readwrite("id", &Neighbor::id, DOC(hl_navigation_Neighbor, id))
      .def("__repr__", &to_string<Neighbor>);

  py::class_<LineSegment>(m, "LineSegment", DOC(hl_navigation_LineSegment))
      .def(py::init<Vector2, Vector2>(), py::arg("p1"), py::arg("p2"),
           DOC(hl_navigation_LineSegment, LineSegment))
      .def_readonly("p1", &LineSegment::p1, DOC(hl_navigation_LineSegment, p1))
      .def_readonly("p2", &LineSegment::p2, DOC(hl_navigation_LineSegment, p2))
      .def_readonly("e1", &LineSegment::e1, DOC(hl_navigation_LineSegment, e1))
      .def_readonly("e2", &LineSegment::e2, DOC(hl_navigation_LineSegment, e2))
      .def_readonly("length", &LineSegment::length,
                    DOC(hl_navigation_LineSegment, length))
      .def("distance_from_point",
           py::overload_cast<const Vector2 &>(&LineSegment::distance,
                                              py::const_),
           py::arg("point"), DOC(hl_navigation_LineSegment, distance))
      .def("distance_from_disc",
           py::overload_cast<const Disc &, bool>(&LineSegment::distance,
                                                 py::const_),
           py::arg("disc"), py::arg("penetration") = false,
           DOC(hl_navigation_LineSegment, distance, 2))
      .def("__repr__", &to_string<LineSegment>);

  py::class_<Kinematics, PyKinematics, HasRegister<Kinematics>,
             std::shared_ptr<Kinematics>>(m, "Kinematics",
                                          DOC(hl_navigation_Kinematics))
      .def(py::init<float, float>(), py::arg("max_speed"),
           py::arg("max_angular_speed") = 0.0,
           DOC(hl_navigation_Kinematics, Kinematics))
      .def_property("max_speed", &Kinematics::get_max_speed,
                    &Kinematics::set_max_speed,
                    DOC(hl_navigation_Kinematics, property_max_speed))
      .def_property("max_angular_speed", &Kinematics::get_max_angular_speed,
                    &Kinematics::set_max_angular_speed,
                    DOC(hl_navigation_Kinematics, property_max_angular_speed))
      .def_property("is_wheeled", &Kinematics::is_wheeled, nullptr,
                    DOC(hl_navigation_Kinematics, property_is_wheeled))
      .def_property("dof", &Kinematics::dof, nullptr,
                    DOC(hl_navigation_Kinematics, property_dof))
      .def_property(
          "type", [](Kinematics *obj) { return obj->get_type(); }, nullptr,
          DOC(hl_navigation_HasRegister, property_type))
      .def("feasible", &Kinematics::feasible,
           DOC(hl_navigation_Kinematics, feasible));

  py::class_<Holonomic, Kinematics, std::shared_ptr<Holonomic>>(
      m, "Holonomic", DOC(hl_navigation_Holonomic))
      .def(py::init<float, float>(), py::arg("max_speed"),
           py::arg("max_angular_speed"),
           DOC(hl_navigation_Holonomic, Holonomic));

  py::class_<Forward, Kinematics, std::shared_ptr<Forward>>(
      m, "Forward", DOC(hl_navigation_Forward))
      .def(py::init<float, float>(), py::arg("max_speed"),
           py::arg("max_angular_speed"), DOC(hl_navigation_Forward, Forward));

  py::class_<Wheeled, Kinematics, std::shared_ptr<Wheeled>>(
      m, "Wheeled", DOC(hl_navigation_Wheeled))
      .def_property("axis", &Wheeled::get_axis, nullptr,
                    DOC(hl_navigation_Wheeled, property_axis))
      .def("twist", &Wheeled::twist, DOC(hl_navigation_Wheeled, twist))
      .def("wheel_speeds", &Wheeled::wheel_speeds,
           DOC(hl_navigation_Wheeled, wheel_speeds));

  py::class_<TwoWheeled, Wheeled, std::shared_ptr<TwoWheeled>>(
      m, "TwoWheeled", DOC(hl_navigation_TwoWheeled))
      .def(py::init<float, float>(), py::arg("max_speed"), py::arg("axis"),
           DOC(hl_navigation_TwoWheeled, TwoWheeled));

  py::class_<FourWheeled, Wheeled, std::shared_ptr<FourWheeled>>(
      m, "FourWheeled", DOC(hl_navigation_FourWheeled))
      .def(py::init<float, float>(), py::arg("max_speed"), py::arg("axis"),
           DOC(hl_navigation_FourWheeled, FourWheeled));

  py::class_<SocialMargin::Modulation,
             std::shared_ptr<SocialMargin::Modulation>>(
      m, "SocialMarginModulation", DOC(hl_navigation_SocialMargin_Modulation))
      .def(
          "__call__",
          [](const SocialMargin::Modulation *mod, float margin,
             std::optional<float> distance) {
            if (distance) {
              return (*mod)(margin, *distance);
            }
            return (*mod)(margin);
          },
          py::arg("margin"), py::arg("distance") = py::none(),
          DOC(hl_navigation_SocialMargin_Modulation, operator_call));

  py::class_<SocialMargin::ZeroModulation, SocialMargin::Modulation,
             std::shared_ptr<SocialMargin::ZeroModulation>>(
      m, "SocialMarginZeroModulation",
      DOC(hl_navigation_SocialMargin_ZeroModulation))
      .def(py::init<>(),
           DOC(hl_navigation_SocialMargin_Modulation, Modulation));
  py::class_<SocialMargin::ConstantModulation, SocialMargin::Modulation,
             std::shared_ptr<SocialMargin::ConstantModulation>>(
      m, "SocialMarginConstantModulation",
      DOC(hl_navigation_SocialMargin_ConstantModulation))
      .def(py::init<>(),
           DOC(hl_navigation_SocialMargin_Modulation, Modulation));
  py::class_<SocialMargin::LinearModulation, SocialMargin::Modulation,
             std::shared_ptr<SocialMargin::LinearModulation>>(
      m, "SocialMarginLinearModulation",
      DOC(hl_navigation_SocialMargin_LinearModulation))
      .def(py::init<float>(), py::arg("upper_distance"),
           DOC(hl_navigation_SocialMargin_LinearModulation, LinearModulation));
  py::class_<SocialMargin::QuadraticModulation, SocialMargin::Modulation,
             std::shared_ptr<SocialMargin::QuadraticModulation>>(
      m, "SocialMarginQuadraticModulation",
      DOC(hl_navigation_SocialMargin_QuadraticModulation))
      .def(py::init<float>(), py::arg("upper_distance"),
           DOC(hl_navigation_SocialMargin_QuadraticModulation,
               QuadraticModulation));
  py::class_<SocialMargin::LogisticModulation, SocialMargin::Modulation,
             std::shared_ptr<SocialMargin::LogisticModulation>>(
      m, "SocialMarginLogisticModulation",
      DOC(hl_navigation_SocialMargin_LogisticModulation))
      .def(py::init<>(),
           DOC(hl_navigation_SocialMargin_Modulation, Modulation));

  py::class_<SocialMargin, std::shared_ptr<SocialMargin>>(
      m, "SocialMargin", DOC(hl_navigation_SocialMargin))
      .def(py::init<float>(), py::arg("value") = 0.0f,
           DOC(hl_navigation_SocialMargin, SocialMargin))
      .def_property("modulation", &SocialMargin::get_modulation,
                    &SocialMargin::set_modulation,
                    DOC(hl_navigation_SocialMargin, property_modulation))
      .def(
          "get",
          [](SocialMargin *sm, std::optional<unsigned> type,
             std::optional<float> distance) {
            if (!type) {
              return sm->get();
            }
            if (!distance) {
              return sm->get(*type);
            }
            return sm->get(*type, *distance);
          },
          py::arg("type") = py::none(), py::arg("distance") = py::none(),
          DOC(hl_navigation_SocialMargin, get))
      .def(
          "set",
          [](SocialMargin *sm, float value, std::optional<unsigned> type) {
            if (!type) {
              return sm->set(value);
            }
            return sm->set(*type, value);
          },
          py::arg("value"), py::arg("type") = py::none(),
          DOC(hl_navigation_SocialMargin, set));

  py::class_<Behavior, PyBehavior, HasRegister<Behavior>, HasProperties,
             std::shared_ptr<Behavior>>
      behavior(m, "Behavior", DOC(hl_navigation_Behavior));

  py::enum_<Behavior::Heading>(behavior, "Heading",
                               DOC(hl_navigation_Behavior_Heading))
      .value("idle", Behavior::Heading::idle,
             DOC(hl_navigation_Behavior_Heading, idle))
      .value("target_point", Behavior::Heading::target_point,
             DOC(hl_navigation_Behavior_Heading, target_point))
      .value("target_angle", Behavior::Heading::target_angle,
             DOC(hl_navigation_Behavior_Heading, target_angle))
      .value("target_angular_speed", Behavior::Heading::target_angular_speed,
             DOC(hl_navigation_Behavior_Heading, target_angular_speed))
      .value("velocity", Behavior::Heading::velocity,
             DOC(hl_navigation_Behavior_Heading, velocity));


  py::enum_<Behavior::Mode>(behavior, "Mode", DOC(hl_navigation_Behavior_Mode))
      .value("move", Behavior::Mode::move,
             DOC(hl_navigation_Behavior_Mode, move))
      .value("turn", Behavior::Mode::turn,
             DOC(hl_navigation_Behavior_Mode, turn))
      .value("stop", Behavior::Mode::stop,
             DOC(hl_navigation_Behavior_Mode, stop))
      .value("follow", Behavior::Mode::follow,
             DOC(hl_navigation_Behavior_Mode, follow));

  behavior
      .def(py::init<std::shared_ptr<Kinematics>, float>(),
           py::arg("kinematics") = py::none(), py::arg("radius") = 0.0,
           DOC(hl_navigation_Behavior, Behavior))
      .def_property(
          "kinematics", &Behavior::get_kinematics,
          py::cpp_function(&Behavior::set_kinematics, py::keep_alive<1, 2>()),
          DOC(hl_navigation_Behavior, property_kinematics))
      .def_property("radius", &Behavior::get_radius, &Behavior::set_radius,
                    DOC(hl_navigation_Behavior, property_radius))
      .def_property("max_speed", &Behavior::get_max_speed,
                    &Behavior::set_max_speed,
                    DOC(hl_navigation_Behavior, property_max_speed))
      .def_property("max_angular_speed", &Behavior::get_max_angular_speed,
                    &Behavior::set_max_angular_speed,
                    DOC(hl_navigation_Behavior, property_max_angular_speed))
      .def_property("optimal_speed", &Behavior::get_optimal_speed,
                    &Behavior::set_optimal_speed,
                    DOC(hl_navigation_Behavior, property_optimal_speed))
      .def_property("optimal_angular_speed",
                    &Behavior::get_optimal_angular_speed,
                    &Behavior::set_optimal_angular_speed,
                    DOC(hl_navigation_Behavior, property_optimal_angular_speed))
      .def_property("rotation_tau", &Behavior::get_rotation_tau,
                    &Behavior::set_rotation_tau,
                    DOC(hl_navigation_Behavior, property_rotation_tau))
      .def_property("safety_margin", &Behavior::get_safety_margin,
                    &Behavior::set_safety_margin,
                    DOC(hl_navigation_Behavior, property_safety_margin))
      .def_property("horizon", &Behavior::get_horizon, &Behavior::set_horizon,
                    DOC(hl_navigation_Behavior, property_horizon))

      .def_property("pose", &Behavior::get_pose, &Behavior::set_pose,
                    DOC(hl_navigation_Behavior, property_pose))
      .def_property("position", &Behavior::get_position,
                    &Behavior::set_position,
                    DOC(hl_navigation_Behavior, property_position))
      .def_property("orientation", &Behavior::get_orientation,
                    &Behavior::set_orientation,
                    DOC(hl_navigation_Behavior, property_orientation))
      .def_property("default_cmd_frame", &Behavior::default_cmd_frame, nullptr,
                    DOC(hl_navigation_Behavior, default_cmd_frame))
      .def_readonly("social_margin", &Behavior::social_margin,
                    DOC(hl_navigation_Behavior, social_margin))
      .def_property(
          "twist", [](const Behavior &self) { return self.get_twist(); },
          &Behavior::set_twist, DOC(hl_navigation_Behavior, property_twist))
      .def("get_twist", &Behavior::get_twist, py::arg("relative") = false,
           DOC(hl_navigation_Behavior, get_twist))
      .def_property(
          "velocity", [](const Behavior &self) { return self.get_velocity(); },
          [](Behavior &self, const Vector2 v) { return self.set_velocity(v); },
          DOC(hl_navigation_Behavior, property_velocity))
      .def("get_velocity", &Behavior::get_velocity, py::arg("relative") = false,
           DOC(hl_navigation_Behavior, get_velocity))
      .def("set_velocity", &Behavior::set_velocity, py::arg("velocity"),
           py::arg("relative") = false,
           DOC(hl_navigation_Behavior, set_velocity))
      .def_property("angular_speed", &Behavior::get_angular_speed,
                    &Behavior::set_angular_speed,
                    DOC(hl_navigation_Behavior, property_angular_speed))
      .def_property("wheel_speeds", &Behavior::get_wheel_speeds,
                    &Behavior::set_wheel_speeds,
                    DOC(hl_navigation_Behavior, property_wheel_speeds))
      .def_property(
          "actuated_twist",
          [](const Behavior &self) { return self.get_actuated_twist(); },
          &Behavior::set_actuated_twist,
          DOC(hl_navigation_Behavior, property_actuated_twist))
      .def("get_actuated_twist", &Behavior::get_actuated_twist,
           py::arg("relative") = false,
           DOC(hl_navigation_Behavior, get_actuated_twist))
      .def_property(
          "velocity", [](const Behavior &self) { return self.get_velocity(); },
          [](Behavior &self, const Vector2 v) { return self.set_velocity(v); },
          DOC(hl_navigation_Behavior, property_velocity))
      .def_property("actuated_wheel_speeds",
                    &Behavior::get_actuated_wheel_speeds, nullptr,
                    DOC(hl_navigation_Behavior, property_actuated_wheel_speeds))
      .def("actuate",
           py::overload_cast<const Twist2 &, float>(&Behavior::actuate),
           py::arg("twist"), py::arg("time"),
           DOC(hl_navigation_Behavior, actuate))
      .def("actuate", py::overload_cast<float>(&Behavior::actuate),
           py::arg("time"), DOC(hl_navigation_Behavior, actuate, 2))

      .def_property("heading_behavior", &Behavior::get_heading_behavior,
                    &Behavior::set_heading_behavior,
                    DOC(hl_navigation_Behavior, property_heading_behavior))
      .def_property("target_pose", &Behavior::get_target_pose,
                    &Behavior::set_target_pose,
                    DOC(hl_navigation_Behavior, property_target_pose))
      .def_property("target_position", &Behavior::get_target_position,
                    &Behavior::set_target_position,
                    DOC(hl_navigation_Behavior, property_target_position))
      .def_property("target_orientation", &Behavior::get_target_orientation,
                    &Behavior::set_target_orientation,
                    DOC(hl_navigation_Behavior, property_target_orientation))
      .def_property("target_velocity", &Behavior::get_target_velocity,
                    &Behavior::set_target_velocity,
                    DOC(hl_navigation_Behavior, property_target_velocity))
      .def_property("target_angular_speed", &Behavior::get_target_angular_speed,
                    &Behavior::set_target_angular_speed,
                    DOC(hl_navigation_Behavior, property_target_angular_speed))
      .def("cmd_twist",
           py::overload_cast<float, Behavior::Mode, std::optional<bool>, bool>(
               &Behavior::cmd_twist),
           py::arg("time_step"), py::arg("mode") = Behavior::Mode::move,
           py::arg("relative") = py::none(), py::arg("set_as_actuated") = true,
           DOC(hl_navigation_Behavior, cmd_twist))
      .def_property("desired_velocity", &Behavior::get_desired_velocity,
                    nullptr,
                    DOC(hl_navigation_Behavior, property_desired_velocity))
      .def_property(
          "type", [](Behavior *obj) { return obj->get_type(); }, nullptr,
          DOC(hl_navigation_HasRegister, property_type))
      .def("to_frame", &Behavior::to_frame,
           DOC(hl_navigation_Behavior, to_frame))
      .def("wheel_speeds_from_twist", &Behavior::wheel_speeds_from_twist,
           DOC(hl_navigation_Behavior, wheel_speeds_from_twist))
      .def("twist_from_wheel_speeds", &Behavior::twist_from_wheel_speeds,
           DOC(hl_navigation_Behavior, twist_from_wheel_speeds));

  m.def("behavior_has_geometric_state", [](const Behavior *obj) {
    return (dynamic_cast<const GeometricState *>(obj)) != nullptr;
  });

  py::class_<GeometricState, std::shared_ptr<GeometricState>>(
      m, "GeometricState", DOC(hl_navigation_GeometricState))
      .def(py::init<>(), DOC(hl_navigation_GeometricState, GeometricState))
      .def_property("neighbors", &GeometricState::get_neighbors,
                    &GeometricState::set_neighbors,
                    DOC(hl_navigation_GeometricState, property_neighbors))
      .def_property(
          "static_obstacles", &GeometricState::get_static_obstacles,
          &GeometricState::set_static_obstacles,
          DOC(hl_navigation_GeometricState, property_static_obstacles))
      .def_property("line_obstacles", &GeometricState::get_line_obstacles,
                    &GeometricState::set_line_obstacles,
                    DOC(hl_navigation_GeometricState, property_line_obstacles));

  py::class_<HLBehavior, GeometricState, Behavior, std::shared_ptr<HLBehavior>>(
      m, "HLBehavior", DOC(hl_navigation_HLBehavior))
      .def(py::init<std::shared_ptr<Kinematics>, float>(),
           py::arg("kinematics") = py::none(), py::arg("radius") = 0.0f,
           DOC(hl_navigation_HLBehavior, HLBehavior))
      .def_property("eta", &HLBehavior::get_eta, &HLBehavior::set_eta,
                    DOC(hl_navigation_HLBehavior, property_eta))
      .def_property("tau", &HLBehavior::get_tau, &HLBehavior::set_tau,
                    DOC(hl_navigation_HLBehavior, property_tau))
      .def_property("aperture", &HLBehavior::get_aperture,
                    &HLBehavior::set_aperture,
                    DOC(hl_navigation_HLBehavior, property_aperture))
      .def_property("resolution", &HLBehavior::get_resolution,
                    &HLBehavior::set_resolution,
                    DOC(hl_navigation_HLBehavior, property_resolution))
      .def_property("angular_resolution", &HLBehavior::get_angular_resolution,
                    nullptr,
                    DOC(hl_navigation_HLBehavior, property_angular_resolution))
      .def("get_collision_distance", &HLBehavior::get_collision_distance,
           DOC(hl_navigation_HLBehavior, get_collision_distance));

  py::class_<ORCABehavior, GeometricState, Behavior,
             std::shared_ptr<ORCABehavior>>(m, "ORCABehavior",
                                            DOC(hl_navigation_ORCABehavior))
      .def(py::init<std::shared_ptr<Kinematics>, float>(),
           py::arg("kinematics") = py::none(), py::arg("radius") = 0.0f,
           DOC(hl_navigation_ORCABehavior, ORCABehavior))
      .def_property("time_horizon", &ORCABehavior::get_time_horizon,
                    &ORCABehavior::set_time_horizon,
                    DOC(hl_navigation_ORCABehavior, property_time_horizon))
      .def_property("is_using_effective_center",
                    &ORCABehavior::is_using_effective_center,
                    &ORCABehavior::should_use_effective_center,
                    DOC(hl_navigation_ORCABehavior, is_using_effective_center));

  py::class_<HRVOBehavior, GeometricState, Behavior,
             std::shared_ptr<HRVOBehavior>>(m, "HRVOBehavior",
                                            DOC(hl_navigation_HRVOBehavior))
      .def(py::init<std::shared_ptr<Kinematics>, float>(),
           py::arg("kinematics") = py::none(), py::arg("radius") = 0.0f,
           DOC(hl_navigation_HRVOBehavior, HRVOBehavior));

  py::class_<DummyBehavior, Behavior, std::shared_ptr<DummyBehavior>>(
      m, "DummyBehavior", DOC(hl_navigation_DummyBehavior))
      .def(py::init<std::shared_ptr<Kinematics>, float>(),
           py::arg("kinematics") = py::none(), py::arg("radius") = 0.0f,
           DOC(hl_navigation_Behavior, Behavior));

  py::class_<Action, std::shared_ptr<Action>> action(m, "Action",
                                                     DOC(hl_navigation_Action));

  py::enum_<Action::State>(action, "State", DOC(hl_navigation_Action_State))
      .value("idle", Action::State::idle, DOC(hl_navigation_Action_State, idle))
      .value("running", Action::State::running,
             DOC(hl_navigation_Action_State, running))
      .value("failure", Action::State::failure,
             DOC(hl_navigation_Action_State, failure))
      .value("success", Action::State::success,
             DOC(hl_navigation_Action_State, success));

  action.def_readonly("state", &Action::state, DOC(hl_navigation_Action, state))
      .def_property("done", &Action::done, nullptr,
                    DOC(hl_navigation_Action, done))
      .def_property("running", &Action::running, nullptr,
                    DOC(hl_navigation_Action, running))
      .def("abort", &Action::abort, DOC(hl_navigation_Action, abort))
      .def_readwrite("running_cb", &Action::running_cb,
                     DOC(hl_navigation_Action, running_cb))
      .def_readwrite("done_cb", &Action::done_cb,
                     DOC(hl_navigation_Action, done_cb));

  py::class_<Controller>(m, "Controller", DOC(hl_navigation_Controller))
      .def(py::init<std::shared_ptr<Behavior>, float, float>(),
           py::arg("behavior") = nullptr,
           py::arg("compute_relative_twist") = true,
           py::arg("set_cmd_twist_as_actuated") = true,
           DOC(hl_navigation_Controller, Controller))
      .def_property("state", &Controller::get_state, nullptr,
                    DOC(hl_navigation_Controller, property_state))
      .def_property("idle", &Controller::idle, nullptr,
                    DOC(hl_navigation_Controller, idle))
      .def_property("behavior", &Controller::get_behavior,
                    &Controller::set_behavior,
                    DOC(hl_navigation_Controller, property_behavior))
      .def_property("speed_tolerance", &Controller::get_speed_tolerance,
                    &Controller::set_speed_tolerance,
                    DOC(hl_navigation_Controller, property_speed_tolerance))
      .def("go_to_position", &Controller::go_to_position,
           DOC(hl_navigation_Controller, go_to_position))
      .def("go_to_pose", &Controller::go_to_pose,
           DOC(hl_navigation_Controller, go_to_pose))
      .def("follow_point", &Controller::follow_point,
           DOC(hl_navigation_Controller, follow_point))
      .def("follow_pose", &Controller::follow_pose,
           DOC(hl_navigation_Controller, follow_pose))
      .def("follow_velocity", &Controller::follow_velocity,
           DOC(hl_navigation_Controller, follow_velocity))
      .def("follow_twist", &Controller::follow_twist,
           DOC(hl_navigation_Controller, follow_twist))
      .def("update", &Controller::update, DOC(hl_navigation_Controller, update))
      .def("set_cmd_cb", &Controller::set_cmd_cb,
           DOC(hl_navigation_Controller, set_cmd_cb))
      .def("stop", &Controller::stop, DOC(hl_navigation_Controller, stop));

  py::class_<CollisionComputation>(m, "CollisionComputation",
                                   DOC(hl_navigation_CollisionComputation))
      .def(py::init<>(),
           DOC(hl_navigation_CollisionComputation, CollisionComputation))
      .def("setup",
           py::overload_cast<Pose2, float, const std::vector<LineSegment> &,
                             const std::vector<Disc> &,
                             const std::vector<Neighbor> &>(
               &CollisionComputation::setup),
           DOC(hl_navigation_CollisionComputation, setup, 2))
      .def("static_free_distance", &CollisionComputation::static_free_distance,
           py::arg("angle"), py::arg("max_distance"),
           py::arg("include_neighbors") = true,
           DOC(hl_navigation_CollisionComputation, static_free_distance))

      .def("dynamic_free_distance",
           &CollisionComputation::dynamic_free_distance, py::arg("angle"),
           py::arg("max_distance"), py::arg("speed"),
           DOC(hl_navigation_CollisionComputation, dynamic_free_distance))
      .def("get_free_distance_for_sector",
           &CollisionComputation::get_free_distance_for_sector, py::arg("from"),
           py::arg("length"), py::arg("resolution"), py::arg("max_distance"),
           py::arg("dynamic"), py::arg("speed") = 0.0f,
           DOC(hl_navigation_CollisionComputation,
               get_free_distance_for_sector));

  py::class_<CachedCollisionComputation, CollisionComputation>(
      m, "CachedCollisionComputation",
      DOC(hl_navigation_CachedCollisionComputation))
      .def(py::init<>(), DOC(hl_navigation_CachedCollisionComputation,
                             CachedCollisionComputation))
      .def_property(
          "resolution", &CachedCollisionComputation::get_resolution,
          &CachedCollisionComputation::set_resolution,
          DOC(hl_navigation_CachedCollisionComputation, property_resolution))
      .def_property(
          "min_angle", &CachedCollisionComputation::get_min_angle,
          &CachedCollisionComputation::set_min_angle,
          DOC(hl_navigation_CachedCollisionComputation, property_min_angle))
      .def_property(
          "length", &CachedCollisionComputation::get_length,
          &CachedCollisionComputation::set_length,
          DOC(hl_navigation_CachedCollisionComputation, property_length))
      .def_property(
          "max_distance", &CachedCollisionComputation::get_max_distance,
          &CachedCollisionComputation::set_max_distance,
          DOC(hl_navigation_CachedCollisionComputation, property_max_distance))
      .def_property(
          "speed", &CachedCollisionComputation::get_speed,
          &CachedCollisionComputation::set_speed,
          DOC(hl_navigation_CachedCollisionComputation, property_speed))
      .def("get_free_distance", &CachedCollisionComputation::get_free_distance,
           py::arg("dynamic"),
           DOC(hl_navigation_CachedCollisionComputation, get_free_distance));

  m.def("load_behavior", &YAML::load_string_py<PyBehavior>, py::arg("value"),
        R"doc(
Load a behavior from a YAML string.

:return:
  The loaded behavior or ``None`` if loading fails.)doc");
  m.def("load_kinematics", &YAML::load_string_py<PyKinematics>,
        py::arg("value"), R"doc(
Load a kinematics from a YAML string.

:return:
  The loaded kinematics or ``None`` if loading fails.)doc");
  m.def("dump", &YAML::dump<Behavior>, py::arg("behavior"),
        "Dump a behavior to a YAML-string");
  m.def("dump", &YAML::dump<Kinematics>, py::arg("kinematics"),
        "Dump a kinematics to a YAML-string");

  m.def("load_plugins", &load_plugins,
        py::arg("environement_variable") = default_plugins_env_name);
}
