#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <vector>

#include "docstrings.h"
#include "hl_navigation_core/behavior.h"
#include "hl_navigation_core/behaviors/HL.h"
#include "hl_navigation_core/behaviors/HRVO.h"
#include "hl_navigation_core/behaviors/ORCA.h"
#include "hl_navigation_core/behaviors/dummy.h"
#include "hl_navigation_core/cached_collision_computation.h"
#include "hl_navigation_core/collision_computation.h"
#include "hl_navigation_core/common.h"
#include "hl_navigation_core/controller.h"
#include "hl_navigation_core/kinematics.h"
#include "hl_navigation_core/plugins.h"
#include "hl_navigation_core/yaml/core.h"
#include "hl_navigation_core/yaml/yaml.h"
#include "hl_navigation_py/register.h"
#include "hl_navigation_py/yaml.h"

using namespace hl_navigation::core;
namespace py = pybind11;

template <typename T>
static std::string to_string(const T &value) {
  return std::to_string(value);
}

template <>
std::string to_string(const Vector2 &value) {
  return "(" + std::to_string(value[0]) + ", " + std::to_string(value[1]) + ")";
}

// template <>
// std::string to_string(const bool &value) {
//   return value ? "True" : "False";
// }

template <>
std::string to_string(const Frame &frame) {
  return frame == Frame::relative ? "Frame.relative" : "Frame.absolute";
}

template <>
std::string to_string(const Pose2 &value) {
  return "Pose2(" + to_string(value.position) + ", " +
         std::to_string(value.orientation) + ")";
}

template <>
std::string to_string(const Twist2 &value) {
  return "Twist2(" + to_string(value.velocity) + ", " +
         std::to_string(value.angular_speed) + ", " + to_string(value.frame) +
         ")";
}

template <>
std::string to_string(const Target &value) {
  std::string r = "Target(";
  bool first = true;
  if (value.position) {
    if (!first) r += ", ";
    r += "position=" + to_string(*value.position);
    first = false;
  }
  if (value.orientation) {
    if (!first) r += ", ";
    r += "orientation=" + to_string(*value.orientation);
    first = false;
  }
  if (value.direction) {
    if (!first) r += ", ";
    r += "direction=" + to_string(*value.direction);
    first = false;
  }
  if (value.speed) {
    if (!first) r += ", ";
    r += "speed=" + to_string(*value.speed);
    first = false;
  }
  if (value.angular_speed) {
    if (!first) r += ", ";
    r += "angular_speed=" + to_string(*value.angular_speed);
    first = false;
  }
  if (value.position_tolerance) {
    if (!first) r += ", ";
    r += "position_tolerance=" + to_string(value.position_tolerance);
    first = false;
  }
  if (value.orientation_tolerance) {
    if (!first) r += ", ";
    r += "orientation_tolerance=" + to_string(value.orientation_tolerance);
    first = false;
  }
  r += ")";
  return r;
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
  Twist2 compute_cmd(float time_step, std::optional<Frame> frame) override {
    PYBIND11_OVERRIDE(Twist2, Behavior, compute_cmd, time_step, frame);
  }
  Vector2 desired_velocity_towards_point(const Vector2 &point, float speed,
                                         float time_step) override {
    PYBIND11_OVERRIDE(Vector2, Behavior, desired_velocity_towards_point, point,
                      speed, time_step);
  }
  Vector2 desired_velocity_towards_velocity(const Vector2 &velocity,
                                            float time_step) override {
    PYBIND11_OVERRIDE(Vector2, Behavior, desired_velocity_towards_velocity,
                      velocity, time_step);
  }
  Twist2 twist_towards_velocity(const Vector2 &absolute_velocity,
                                Frame frame) override {
    PYBIND11_OVERRIDE(Twist2, Behavior, twist_towards_velocity,
                      absolute_velocity, frame);
  }
  Twist2 cmd_twist_towards_point(const Vector2 &point, float speed,
                                 float time_step, Frame frame) override {
    PYBIND11_OVERRIDE(Twist2, Behavior, cmd_twist_towards_point, point, speed,
                      time_step, frame);
  }
  Twist2 cmd_twist_towards_velocity(const Vector2 &velocity, float time_step,
                                    Frame frame) override {
    PYBIND11_OVERRIDE(Twist2, Behavior, cmd_twist_towards_velocity, velocity,
                      time_step, frame);
  }
  Twist2 cmd_twist_towards_orientation(float orientation, float angular_speed,
                                       float time_step, Frame frame) override {
    PYBIND11_OVERRIDE(Twist2, Behavior, cmd_twist_towards_orientation,
                      orientation, angular_speed, time_step, frame);
  }
  Twist2 cmd_twist_towards_angular_speed(float angular_speed, float time_step,
                                         Frame frame) override {
    PYBIND11_OVERRIDE(Twist2, Behavior, cmd_twist_towards_angular_speed,
                      angular_speed, time_step, frame);
  }
  Twist2 cmd_twist_towards_stopping(float time_step, Frame frame) override {
    PYBIND11_OVERRIDE(Twist2, Behavior, cmd_twist_towards_stopping, time_step,
                      frame);
  }

  EnvironmentState *get_environment_state() override {
    PYBIND11_OVERRIDE(EnvironmentState *, Behavior, get_environment_state);
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

  py::class_<Property>(m, "Property", DOC(hl_navigation, core, Property))
      .def_readonly("description", &Property::description,
                    DOC(hl_navigation, core, Property, description))
      .def_readonly("owner_type_name", &Property::owner_type_name,
                    DOC(hl_navigation, core, Property, owner_type_name))
      .def_readonly("default_value", &Property::default_value,
                    DOC(hl_navigation, core, Property, default_value))
      .def_readonly("type_name", &Property::type_name,
                    DOC(hl_navigation, core, Property, type_name));

  py::class_<HasProperties, std::shared_ptr<HasProperties>>(
      m, "HasProperties", DOC(hl_navigation, core, HasProperties))
      .def("get", &HasProperties::get, py::arg("name"),
           DOC(hl_navigation, core, HasProperties, get))
      .def("set", &HasProperties::set, py::arg("name"), py::arg("value"),
           DOC(hl_navigation, core, HasProperties, set))
      .def_property(
          "properties", &HasProperties::get_properties, nullptr,
          DOC(hl_navigation, core, HasProperties, property_properties));

  py::enum_<Frame>(m, "Frame", DOC(hl_navigation, core, Frame))
      .value("relative", Frame::relative,
             DOC(hl_navigation, core, Frame, relative))
      .value("absolute", Frame::absolute,
             DOC(hl_navigation, core, Frame, absolute));

  py::class_<Twist2>(m, "Twist2", DOC(hl_navigation, core, Twist2))
      .def(py::init<Vector2, float, Frame>(), py::arg("velocity"),
           py::arg("angular_speed") = 0.0f, py::arg("frame") = Frame::absolute,
           DOC(hl_navigation, core, Twist2, Twist2))
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def_readwrite("velocity", &Twist2::velocity,
                     DOC(hl_navigation, core, Twist2, velocity))
      .def_readwrite("angular_speed", &Twist2::angular_speed,
                     DOC(hl_navigation, core, Twist2, angular_speed))
      .def_readwrite("frame", &Twist2::frame,
                     DOC(hl_navigation, core, Twist2, frame))
      .def("rotate", &Twist2::rotate, py::arg("angle"),
           DOC(hl_navigation, core, Twist2, rotate))
      .def("__repr__", &to_string<Twist2>);

  py::class_<Pose2>(m, "Pose2", DOC(hl_navigation, core, Pose2))
      .def(py::init<Vector2, float>(), py::arg("position"),
           py::arg("orientation") = 0.0f,
           DOC(hl_navigation, core, Pose2, Pose2))
      .def_readwrite("position", &Pose2::position,
                     DOC(hl_navigation, core, Pose2, position))
      .def_readwrite("orientation", &Pose2::orientation,
                     DOC(hl_navigation, core, Pose2, orientation))
      .def("rotate", &Pose2::rotate, py::arg("angle"),
           DOC(hl_navigation, core, Pose2, rotate))
      .def("integrate", &Pose2::integrate, py::arg("twist"),
           py::arg("time_step"), DOC(hl_navigation, core, Pose2, rotate))
      .def("__repr__", &to_string<Pose2>);

  py::class_<Target>(m, "Target", DOC(hl_navigation, core, Target))
      .def(py::init<std::optional<Vector2>, std::optional<Radians>,
                    std::optional<float>, std::optional<Vector2>,
                    std::optional<float>, float, float>(),
           py::arg("position") = py::none(),
           py::arg("orientation") = py::none(), py::arg("speed") = py::none(),
           py::arg("direction") = py::none(),
           py::arg("angular_speed") = py::none(),
           py::arg("position_tolerance") = 0.0f,
           py::arg("orientation_tolerance") = 0.0f,
           DOC(hl_navigation, core, Target, Target))
      .def_readwrite("position", &Target::position,
                     DOC(hl_navigation, core, Target, position))
      .def_readwrite("orientation", &Target::orientation,
                     DOC(hl_navigation, core, Target, orientation))
      .def_readwrite("speed", &Target::speed,
                     DOC(hl_navigation, core, Target, speed))
      .def_readwrite("angular_speed", &Target::angular_speed,
                     DOC(hl_navigation, core, Target, angular_speed))
      .def_readwrite("direction", &Target::direction,
                     DOC(hl_navigation, core, Target, direction))
      .def_readwrite("position_tolerance", &Target::position_tolerance,
                     DOC(hl_navigation, core, Target, position_tolerance))
      .def_readwrite("orientation_tolerance", &Target::orientation_tolerance,
                     DOC(hl_navigation, core, Target, orientation_tolerance))
      .def("satisfied",
           py::overload_cast<const Vector2 &>(&Target::satisfied, py::const_),
           DOC(hl_navigation, core, Target, satisfied))
      .def("satisfied",
           py::overload_cast<float>(&Target::satisfied, py::const_),
           DOC(hl_navigation, core, Target, satisfied, 2))
      .def("satisfied",
           py::overload_cast<const Pose2 &>(&Target::satisfied, py::const_),
           DOC(hl_navigation, core, Target, satisfied, 3))
      .def_property("valid", &Target::valid, nullptr,
                    DOC(hl_navigation, core, Target, valid))
      .def_static("Point", &Target::Point, py::arg("point"),
                  py::arg("tolerance") = 0.0f,
                  DOC(hl_navigation, core, Target, Point))
      .def_static("Pose", &Target::Pose, py::arg("pose"),
                  py::arg("position_tolerance") = 0.0f,
                  py::arg("orientation_tolerance") = 0.0f,
                  DOC(hl_navigation, core, Target, Pose))
      .def_static("Orientation", &Target::Orientation, py::arg("orientation"),
                  py::arg("tolerance") = 0.0f,
                  DOC(hl_navigation, core, Target, Orientation))
      .def_static("Velocity", &Target::Velocity, py::arg("velocity"),
                  DOC(hl_navigation, core, Target, Velocity))
      .def_static("Direction", &Target::Direction, py::arg("direction"),
                  DOC(hl_navigation, core, Target, Direction))
      .def_static("Twist", &Target::Twist, py::arg("twist"),
                  DOC(hl_navigation, core, Target, Twist))
      .def_static("Stop", &Target::Point,
                  DOC(hl_navigation, core, Target, Stop))
      .def("__repr__", &to_string<Target>);

  py::class_<Disc>(m, "Disc", DOC(hl_navigation, core, Disc))
      .def(py::init<Vector2, float>(), py::arg("position"), py::arg("radius"),
           DOC(hl_navigation, core, Disc, Disc))
      .def_readwrite("position", &Disc::position,
                     DOC(hl_navigation, core, Disc, position))
      .def_readwrite("radius", &Disc::radius,
                     DOC(hl_navigation, core, Disc, radius))
      .def("__repr__", &to_string<Disc>);

  py::class_<Neighbor, Disc>(m, "Neighbor", DOC(hl_navigation, core, Neighbor))
      .def(py::init<Vector2, float, Vector2, int>(), py::arg("position"),
           py::arg("radius"), py::arg("velocity") = Vector2(0.0, 0.0),
           py::arg("id") = 0, DOC(hl_navigation, core, Neighbor, Neighbor))
      .def_readwrite("velocity", &Neighbor::velocity,
                     DOC(hl_navigation, core, Neighbor, velocity))
      .def_readwrite("id", &Neighbor::id,
                     DOC(hl_navigation, core, Neighbor, id))
      .def("__repr__", &to_string<Neighbor>);

  py::class_<LineSegment>(m, "LineSegment",
                          DOC(hl_navigation, core, LineSegment))
      .def(py::init<Vector2, Vector2>(), py::arg("p1"), py::arg("p2"),
           DOC(hl_navigation, core, LineSegment, LineSegment))
      .def_readonly("p1", &LineSegment::p1,
                    DOC(hl_navigation, core, LineSegment, p1))
      .def_readonly("p2", &LineSegment::p2,
                    DOC(hl_navigation, core, LineSegment, p2))
      .def_readonly("e1", &LineSegment::e1,
                    DOC(hl_navigation, core, LineSegment, e1))
      .def_readonly("e2", &LineSegment::e2,
                    DOC(hl_navigation, core, LineSegment, e2))
      .def_readonly("length", &LineSegment::length,
                    DOC(hl_navigation, core, LineSegment, length))
      .def("distance_from_point",
           py::overload_cast<const Vector2 &>(&LineSegment::distance,
                                              py::const_),
           py::arg("point"), DOC(hl_navigation, core, LineSegment, distance))
      .def("distance_from_disc",
           py::overload_cast<const Disc &, bool>(&LineSegment::distance,
                                                 py::const_),
           py::arg("disc"), py::arg("penetration") = false,
           DOC(hl_navigation, core, LineSegment, distance, 2))
      .def("__repr__", &to_string<LineSegment>);

  py::class_<Kinematics, PyKinematics, HasRegister<Kinematics>,
             std::shared_ptr<Kinematics>>(m, "Kinematics",
                                          DOC(hl_navigation, core, Kinematics))
      .def(py::init<float, float>(), py::arg("max_speed"),
           py::arg("max_angular_speed") = 0.0,
           DOC(hl_navigation, core, Kinematics, Kinematics))
      .def_property("max_speed", &Kinematics::get_max_speed,
                    &Kinematics::set_max_speed,
                    DOC(hl_navigation, core, Kinematics, property_max_speed))
      .def_property(
          "max_angular_speed", &Kinematics::get_max_angular_speed,
          &Kinematics::set_max_angular_speed,
          DOC(hl_navigation, core, Kinematics, property_max_angular_speed))
      .def_property("is_wheeled", &Kinematics::is_wheeled, nullptr,
                    DOC(hl_navigation, core, Kinematics, property_is_wheeled))
      .def_property("dof", &Kinematics::dof, nullptr,
                    DOC(hl_navigation, core, Kinematics, property_dof))
      .def_property(
          "type", [](Kinematics *obj) { return obj->get_type(); }, nullptr,
          DOC(hl_navigation, core, HasRegister, property_type))
      .def("feasible", &Kinematics::feasible,
           DOC(hl_navigation, core, Kinematics, feasible));

  py::class_<OmnidirectionalKinematics, Kinematics,
             std::shared_ptr<OmnidirectionalKinematics>>(
      m, "OmnidirectionalKinematics",
      DOC(hl_navigation, core, OmnidirectionalKinematics))
      .def(py::init<float, float>(), py::arg("max_speed"),
           py::arg("max_angular_speed"),
           DOC(hl_navigation, core, OmnidirectionalKinematics,
               OmnidirectionalKinematics));

  py::class_<AheadKinematics, Kinematics, std::shared_ptr<AheadKinematics>>(
      m, "AheadKinematics", DOC(hl_navigation, core, AheadKinematics))
      .def(py::init<float, float>(), py::arg("max_speed"),
           py::arg("max_angular_speed"),
           DOC(hl_navigation, core, AheadKinematics, AheadKinematics));

  py::class_<WheeledKinematics, Kinematics, std::shared_ptr<WheeledKinematics>>(
      m, "WheeledKinematics", DOC(hl_navigation, core, WheeledKinematics))
      .def_property("axis", &WheeledKinematics::get_axis, nullptr,
                    DOC(hl_navigation, core, WheeledKinematics, property_axis))
      .def("twist", &WheeledKinematics::twist,
           DOC(hl_navigation, core, WheeledKinematics, twist))
      .def("wheel_speeds", &WheeledKinematics::wheel_speeds,
           DOC(hl_navigation, core, WheeledKinematics, wheel_speeds));

  py::class_<TwoWheelsDifferentialDriveKinematics, WheeledKinematics,
             std::shared_ptr<TwoWheelsDifferentialDriveKinematics>>(
      m, "TwoWheelsDifferentialDriveKinematics",
      DOC(hl_navigation, core, TwoWheelsDifferentialDriveKinematics))
      .def(py::init<float, float>(), py::arg("max_speed"), py::arg("axis"),
           DOC(hl_navigation, core, TwoWheelsDifferentialDriveKinematics,
               TwoWheelsDifferentialDriveKinematics));

  py::class_<FourWheelsOmniDriveKinematics, WheeledKinematics,
             std::shared_ptr<FourWheelsOmniDriveKinematics>>(
      m, "FourWheelsOmniDriveKinematics",
      DOC(hl_navigation, core, FourWheelsOmniDriveKinematics))
      .def(py::init<float, float>(), py::arg("max_speed"), py::arg("axis"),
           DOC(hl_navigation, core, FourWheelsOmniDriveKinematics,
               FourWheelsOmniDriveKinematics));

  py::class_<SocialMargin::Modulation,
             std::shared_ptr<SocialMargin::Modulation>>(
      m, "SocialMarginModulation",
      DOC(hl_navigation, core, SocialMargin_Modulation))
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
          DOC(hl_navigation, core, SocialMargin_Modulation, operator_call));

  py::class_<SocialMargin::ZeroModulation, SocialMargin::Modulation,
             std::shared_ptr<SocialMargin::ZeroModulation>>(
      m, "SocialMarginZeroModulation",
      DOC(hl_navigation, core, SocialMargin_ZeroModulation))
      .def(py::init<>(),
           DOC(hl_navigation, core, SocialMargin_Modulation, Modulation));
  py::class_<SocialMargin::ConstantModulation, SocialMargin::Modulation,
             std::shared_ptr<SocialMargin::ConstantModulation>>(
      m, "SocialMarginConstantModulation",
      DOC(hl_navigation, core, SocialMargin_ConstantModulation))
      .def(py::init<>(),
           DOC(hl_navigation, core, SocialMargin_Modulation, Modulation));
  py::class_<SocialMargin::LinearModulation, SocialMargin::Modulation,
             std::shared_ptr<SocialMargin::LinearModulation>>(
      m, "SocialMarginLinearModulation",
      DOC(hl_navigation, core, SocialMargin_LinearModulation))
      .def(py::init<float>(), py::arg("upper_distance"),
           DOC(hl_navigation, core, SocialMargin_LinearModulation,
               LinearModulation));
  py::class_<SocialMargin::QuadraticModulation, SocialMargin::Modulation,
             std::shared_ptr<SocialMargin::QuadraticModulation>>(
      m, "SocialMarginQuadraticModulation",
      DOC(hl_navigation, core, SocialMargin_QuadraticModulation))
      .def(py::init<float>(), py::arg("upper_distance"),
           DOC(hl_navigation, core, SocialMargin_QuadraticModulation,
               QuadraticModulation));
  py::class_<SocialMargin::LogisticModulation, SocialMargin::Modulation,
             std::shared_ptr<SocialMargin::LogisticModulation>>(
      m, "SocialMarginLogisticModulation",
      DOC(hl_navigation, core, SocialMargin_LogisticModulation))
      .def(py::init<>(),
           DOC(hl_navigation, core, SocialMargin_Modulation, Modulation));

  py::class_<SocialMargin, std::shared_ptr<SocialMargin>>(
      m, "SocialMargin", DOC(hl_navigation, core, SocialMargin))
      .def(py::init<float>(), py::arg("value") = 0.0f,
           DOC(hl_navigation, core, SocialMargin, SocialMargin))
      .def_property("modulation", &SocialMargin::get_modulation,
                    &SocialMargin::set_modulation,
                    DOC(hl_navigation, core, SocialMargin, property_modulation))
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
          DOC(hl_navigation, core, SocialMargin, get))
      .def(
          "set",
          [](SocialMargin *sm, float value, std::optional<unsigned> type) {
            if (!type) {
              return sm->set(value);
            }
            return sm->set(*type, value);
          },
          py::arg("value"), py::arg("type") = py::none(),
          DOC(hl_navigation, core, SocialMargin, set));

  py::class_<Behavior, PyBehavior, HasRegister<Behavior>, HasProperties,
             std::shared_ptr<Behavior>>
      behavior(m, "Behavior", DOC(hl_navigation, core, Behavior));

  py::enum_<Behavior::Heading>(behavior, "Heading",
                               DOC(hl_navigation, core, Behavior_Heading))
      .value("idle", Behavior::Heading::idle,
             DOC(hl_navigation, core, Behavior_Heading, idle))
      .value("target_point", Behavior::Heading::target_point,
             DOC(hl_navigation, core, Behavior_Heading, target_point))
      .value("target_angle", Behavior::Heading::target_angle,
             DOC(hl_navigation, core, Behavior_Heading, target_angle))
      .value("target_angular_speed", Behavior::Heading::target_angular_speed,
             DOC(hl_navigation, core, Behavior_Heading, target_angular_speed))
      .value("velocity", Behavior::Heading::velocity,
             DOC(hl_navigation, core, Behavior_Heading, velocity));

  behavior
      .def(py::init<std::shared_ptr<Kinematics>, float>(),
           py::arg("kinematics") = py::none(), py::arg("radius") = 0.0,
           DOC(hl_navigation, core, Behavior, Behavior))
      .def_property(
          "kinematics", &Behavior::get_kinematics,
          py::cpp_function(&Behavior::set_kinematics, py::keep_alive<1, 2>()),
          DOC(hl_navigation, core, Behavior, property_kinematics))
      .def_property("radius", &Behavior::get_radius, &Behavior::set_radius,
                    DOC(hl_navigation, core, Behavior, property_radius))
      .def_property("max_speed", &Behavior::get_max_speed,
                    &Behavior::set_max_speed,
                    DOC(hl_navigation, core, Behavior, property_max_speed))
      .def_property(
          "max_angular_speed", &Behavior::get_max_angular_speed,
          &Behavior::set_max_angular_speed,
          DOC(hl_navigation, core, Behavior, property_max_angular_speed))
      .def_property("optimal_speed", &Behavior::get_optimal_speed,
                    &Behavior::set_optimal_speed,
                    DOC(hl_navigation, core, Behavior, property_optimal_speed))
      .def_property(
          "optimal_angular_speed", &Behavior::get_optimal_angular_speed,
          &Behavior::set_optimal_angular_speed,
          DOC(hl_navigation, core, Behavior, property_optimal_angular_speed))
      .def_property("rotation_tau", &Behavior::get_rotation_tau,
                    &Behavior::set_rotation_tau,
                    DOC(hl_navigation, core, Behavior, property_rotation_tau))
      .def_property("safety_margin", &Behavior::get_safety_margin,
                    &Behavior::set_safety_margin,
                    DOC(hl_navigation, core, Behavior, property_safety_margin))
      .def_property("horizon", &Behavior::get_horizon, &Behavior::set_horizon,
                    DOC(hl_navigation, core, Behavior, property_horizon))

      .def_property("pose", &Behavior::get_pose, &Behavior::set_pose,
                    DOC(hl_navigation, core, Behavior, property_pose))
      .def_property("position", &Behavior::get_position,
                    &Behavior::set_position,
                    DOC(hl_navigation, core, Behavior, property_position))
      .def_property("orientation", &Behavior::get_orientation,
                    &Behavior::set_orientation,
                    DOC(hl_navigation, core, Behavior, property_orientation))
      .def_property("default_cmd_frame", &Behavior::default_cmd_frame, nullptr,
                    DOC(hl_navigation, core, Behavior, default_cmd_frame))
      .def_property(
          "assume_cmd_is_actuated", &Behavior::get_assume_cmd_is_actuated,
          &Behavior::set_assume_cmd_is_actuated,
          DOC(hl_navigation, core, Behavior, property_assume_cmd_is_actuated))

      .def_readonly("social_margin", &Behavior::social_margin,
                    DOC(hl_navigation, core, Behavior, social_margin))
      .def_property(
          "twist", [](const Behavior &self) { return self.get_twist(); },
          &Behavior::set_twist,
          DOC(hl_navigation, core, Behavior, property_twist))
      .def("get_twist", &Behavior::get_twist,
           py::arg("frame") = Frame::absolute,
           DOC(hl_navigation, core, Behavior, get_twist))
      .def_property(
          "velocity", [](const Behavior &self) { return self.get_velocity(); },
          [](Behavior &self, const Vector2 v) { return self.set_velocity(v); },
          DOC(hl_navigation, core, Behavior, property_velocity))
      .def("get_velocity", &Behavior::get_velocity,
           py::arg("frame") = Frame::absolute,
           DOC(hl_navigation, core, Behavior, get_velocity))
      .def("set_velocity", &Behavior::set_velocity, py::arg("velocity"),
           py::arg("frame") = Frame::absolute,
           DOC(hl_navigation, core, Behavior, set_velocity))
      .def_property("angular_speed", &Behavior::get_angular_speed,
                    &Behavior::set_angular_speed,
                    DOC(hl_navigation, core, Behavior, property_angular_speed))
      .def_property("wheel_speeds", &Behavior::get_wheel_speeds,
                    &Behavior::set_wheel_speeds,
                    DOC(hl_navigation, core, Behavior, property_wheel_speeds))
      .def_property(
          "actuated_twist",
          [](const Behavior &self) { return self.get_actuated_twist(); },
          &Behavior::set_actuated_twist,
          DOC(hl_navigation, core, Behavior, property_actuated_twist))
      .def("get_actuated_twist", &Behavior::get_actuated_twist,
           py::arg("frame") = Frame::absolute,
           DOC(hl_navigation, core, Behavior, get_actuated_twist))
      .def_property(
          "velocity", [](const Behavior &self) { return self.get_velocity(); },
          [](Behavior &self, const Vector2 v) { return self.set_velocity(v); },
          DOC(hl_navigation, core, Behavior, property_velocity))
      .def_property(
          "actuated_wheel_speeds", &Behavior::get_actuated_wheel_speeds,
          nullptr,
          DOC(hl_navigation, core, Behavior, property_actuated_wheel_speeds))
      .def("actuate",
           py::overload_cast<const Twist2 &, float>(&Behavior::actuate),
           py::arg("twist"), py::arg("time_step"),
           DOC(hl_navigation, core, Behavior, actuate))
      .def("actuate", py::overload_cast<float>(&Behavior::actuate),
           py::arg("time_step"), DOC(hl_navigation, core, Behavior, actuate, 2))
      .def_property(
          "heading_behavior", &Behavior::get_heading_behavior,
          &Behavior::set_heading_behavior,
          DOC(hl_navigation, core, Behavior, property_heading_behavior))
      .def_property("target", &Behavior::get_target, &Behavior::set_target,
                    DOC(hl_navigation, core, Behavior, property_target))
      .def("check_if_target_satisfied", &Behavior::check_if_target_satisfied,
           DOC(hl_navigation, core, Behavior, check_if_target_satisfied))
      .def("compute_cmd", &Behavior::compute_cmd, py::arg("time_step"),
           py::arg("frame") = py::none(),
           DOC(hl_navigation, core, Behavior, compute_cmd))
      .def_property(
          "desired_velocity", &Behavior::get_desired_velocity, nullptr,
          DOC(hl_navigation, core, Behavior, property_desired_velocity))
      .def_property(
          "type", [](Behavior *obj) { return obj->get_type(); }, nullptr,
          DOC(hl_navigation, core, HasRegister, property_type))
      .def("to_frame", &Behavior::to_frame,
           DOC(hl_navigation, core, Behavior, to_frame))
      .def("feasible_speed", &Behavior::feasible_speed,
           DOC(hl_navigation, core, Behavior, feasible_speed))
      .def("feasible_angular_speed", &Behavior::feasible_angular_speed,
           DOC(hl_navigation, core, Behavior, feasible_angular_speed))
      .def("feasible_twist", &Behavior::feasible_twist,
           DOC(hl_navigation, core, Behavior, feasible_twist))

      .def("estimate_time_until_target_satisfied",
           &Behavior::estimate_time_until_target_satisfied,
           DOC(hl_navigation, core, Behavior,
               estimate_time_until_target_satisfied))
      .def_property(
          "environment_state",
          py::cpp_function(&Behavior::get_environment_state,
                           py::return_value_policy::reference),
          nullptr,
          DOC(hl_navigation, core, Behavior, property_environment_state))
      .def("get_environment_state", &Behavior::get_environment_state,
           py::return_value_policy::reference,
           DOC(hl_navigation, core, Behavior, get_environment_state))
      .def("wheel_speeds_from_twist", &Behavior::wheel_speeds_from_twist,
           DOC(hl_navigation, core, Behavior, wheel_speeds_from_twist))
      .def("twist_from_wheel_speeds", &Behavior::twist_from_wheel_speeds,
           DOC(hl_navigation, core, Behavior, twist_from_wheel_speeds));

  m.def("behavior_has_geometric_state", [](Behavior *obj) {
    return (dynamic_cast<GeometricState *>(obj->get_environment_state())) !=
           nullptr;
  });

  py::class_<EnvironmentState, std::shared_ptr<EnvironmentState>>(
      m, "EnvironmentState", DOC(hl_navigation, core, EnvironmentState));

  py::class_<GeometricState, EnvironmentState, std::shared_ptr<GeometricState>>(
      m, "GeometricState", DOC(hl_navigation, core, GeometricState))
      .def(py::init<>(),
           DOC(hl_navigation, core, GeometricState, GeometricState))
      .def_property(
          "neighbors", &GeometricState::get_neighbors,
          &GeometricState::set_neighbors,
          DOC(hl_navigation, core, GeometricState, property_neighbors))
      .def_property(
          "static_obstacles", &GeometricState::get_static_obstacles,
          &GeometricState::set_static_obstacles,
          DOC(hl_navigation, core, GeometricState, property_static_obstacles))
      .def_property(
          "line_obstacles", &GeometricState::get_line_obstacles,
          &GeometricState::set_line_obstacles,
          DOC(hl_navigation, core, GeometricState, property_line_obstacles));

  py::class_<HLBehavior, Behavior, std::shared_ptr<HLBehavior>>(
      m, "HLBehavior", DOC(hl_navigation, core, HLBehavior))
      .def(py::init<std::shared_ptr<Kinematics>, float>(),
           py::arg("kinematics") = py::none(), py::arg("radius") = 0.0f,
           DOC(hl_navigation, core, HLBehavior, HLBehavior))
      .def_property("eta", &HLBehavior::get_eta, &HLBehavior::set_eta,
                    DOC(hl_navigation, core, HLBehavior, property_eta))
      .def_property("tau", &HLBehavior::get_tau, &HLBehavior::set_tau,
                    DOC(hl_navigation, core, HLBehavior, property_tau))
      .def_property("aperture", &HLBehavior::get_aperture,
                    &HLBehavior::set_aperture,
                    DOC(hl_navigation, core, HLBehavior, property_aperture))
      .def_property("resolution", &HLBehavior::get_resolution,
                    &HLBehavior::set_resolution,
                    DOC(hl_navigation, core, HLBehavior, property_resolution))
      .def_property(
          "angular_resolution", &HLBehavior::get_angular_resolution, nullptr,
          DOC(hl_navigation, core, HLBehavior, property_angular_resolution))
      .def("get_collision_distance", &HLBehavior::get_collision_distance,
           DOC(hl_navigation, core, HLBehavior, get_collision_distance));

  py::class_<ORCABehavior, Behavior, std::shared_ptr<ORCABehavior>>(
      m, "ORCABehavior", DOC(hl_navigation, core, ORCABehavior))
      .def(py::init<std::shared_ptr<Kinematics>, float>(),
           py::arg("kinematics") = py::none(), py::arg("radius") = 0.0f,
           DOC(hl_navigation, core, ORCABehavior, ORCABehavior))
      .def_property(
          "time_horizon", &ORCABehavior::get_time_horizon,
          &ORCABehavior::set_time_horizon,
          DOC(hl_navigation, core, ORCABehavior, property_time_horizon))
      .def_property(
          "is_using_effective_center", &ORCABehavior::is_using_effective_center,
          &ORCABehavior::should_use_effective_center,
          DOC(hl_navigation, core, ORCABehavior, is_using_effective_center));

  py::class_<HRVOBehavior, Behavior, std::shared_ptr<HRVOBehavior>>(
      m, "HRVOBehavior", DOC(hl_navigation, core, HRVOBehavior))
      .def(py::init<std::shared_ptr<Kinematics>, float>(),
           py::arg("kinematics") = py::none(), py::arg("radius") = 0.0f,
           DOC(hl_navigation, core, HRVOBehavior, HRVOBehavior));

  py::class_<DummyBehavior, Behavior, std::shared_ptr<DummyBehavior>>(
      m, "DummyBehavior", DOC(hl_navigation, core, DummyBehavior))
      .def(py::init<std::shared_ptr<Kinematics>, float>(),
           py::arg("kinematics") = py::none(), py::arg("radius") = 0.0f,
           DOC(hl_navigation, core, Behavior, Behavior));

  py::class_<Action, std::shared_ptr<Action>> action(
      m, "Action", DOC(hl_navigation, core, Action));

  py::enum_<Action::State>(action, "State",
                           DOC(hl_navigation, core, Action_State))
      .value("idle", Action::State::idle,
             DOC(hl_navigation, core, Action_State, idle))
      .value("running", Action::State::running,
             DOC(hl_navigation, core, Action_State, running))
      .value("failure", Action::State::failure,
             DOC(hl_navigation, core, Action_State, failure))
      .value("success", Action::State::success,
             DOC(hl_navigation, core, Action_State, success));

  action
      .def_readonly("state", &Action::state,
                    DOC(hl_navigation, core, Action, state))
      .def_property("done", &Action::done, nullptr,
                    DOC(hl_navigation, core, Action, done))
      .def_property("running", &Action::running, nullptr,
                    DOC(hl_navigation, core, Action, running))
      .def("abort", &Action::abort, DOC(hl_navigation, core, Action, abort))
      .def_readwrite("running_cb", &Action::running_cb,
                     DOC(hl_navigation, core, Action, running_cb))
      .def_readwrite("done_cb", &Action::done_cb,
                     DOC(hl_navigation, core, Action, done_cb));

  py::class_<Controller>(m, "Controller", DOC(hl_navigation, core, Controller))
      .def(py::init<std::shared_ptr<Behavior>>(), py::arg("behavior") = nullptr,
           DOC(hl_navigation, core, Controller, Controller))
      .def_property("state", &Controller::get_state, nullptr,
                    DOC(hl_navigation, core, Controller, property_state))
      .def_property("idle", &Controller::idle, nullptr,
                    DOC(hl_navigation, core, Controller, idle))
      .def_property("behavior", &Controller::get_behavior,
                    &Controller::set_behavior,
                    DOC(hl_navigation, core, Controller, property_behavior))
      .def_property(
          "speed_tolerance", &Controller::get_speed_tolerance,
          &Controller::set_speed_tolerance,
          DOC(hl_navigation, core, Controller, property_speed_tolerance))
      .def("go_to_position", &Controller::go_to_position,
           DOC(hl_navigation, core, Controller, go_to_position))
      .def("go_to_pose", &Controller::go_to_pose,
           DOC(hl_navigation, core, Controller, go_to_pose))
      .def("follow_point", &Controller::follow_point,
           DOC(hl_navigation, core, Controller, follow_point))
      .def("follow_pose", &Controller::follow_pose,
           DOC(hl_navigation, core, Controller, follow_pose))
      .def("follow_direction", &Controller::follow_direction,
           DOC(hl_navigation, core, Controller, follow_direction))
      .def("follow_velocity", &Controller::follow_velocity,
           DOC(hl_navigation, core, Controller, follow_velocity))
      .def("follow_twist", &Controller::follow_twist,
           DOC(hl_navigation, core, Controller, follow_twist))
      .def("update", &Controller::update,
           DOC(hl_navigation, core, Controller, update))
      .def("set_cmd_cb", &Controller::set_cmd_cb,
           DOC(hl_navigation, core, Controller, set_cmd_cb))
      .def("stop", &Controller::stop,
           DOC(hl_navigation, core, Controller, stop));

  py::class_<CollisionComputation>(
      m, "CollisionComputation", DOC(hl_navigation, core, CollisionComputation))
      .def(py::init<>(),
           DOC(hl_navigation, core, CollisionComputation, CollisionComputation))
      .def("setup",
           py::overload_cast<Pose2, float, const std::vector<LineSegment> &,
                             const std::vector<Disc> &,
                             const std::vector<Neighbor> &>(
               &CollisionComputation::setup),
           DOC(hl_navigation, core, CollisionComputation, setup, 2))
      .def("static_free_distance", &CollisionComputation::static_free_distance,
           py::arg("angle"), py::arg("max_distance"),
           py::arg("include_neighbors") = true,
           DOC(hl_navigation, core, CollisionComputation, static_free_distance))

      .def(
          "dynamic_free_distance", &CollisionComputation::dynamic_free_distance,
          py::arg("angle"), py::arg("max_distance"), py::arg("speed"),
          DOC(hl_navigation, core, CollisionComputation, dynamic_free_distance))
      .def("get_free_distance_for_sector",
           &CollisionComputation::get_free_distance_for_sector, py::arg("from"),
           py::arg("length"), py::arg("resolution"), py::arg("max_distance"),
           py::arg("dynamic"), py::arg("speed") = 0.0f,
           DOC(hl_navigation, core, CollisionComputation,
               get_free_distance_for_sector));

  py::class_<CachedCollisionComputation, CollisionComputation>(
      m, "CachedCollisionComputation",
      DOC(hl_navigation, core, CachedCollisionComputation))
      .def(py::init<>(), DOC(hl_navigation, core, CachedCollisionComputation,
                             CachedCollisionComputation))
      .def_property("resolution", &CachedCollisionComputation::get_resolution,
                    &CachedCollisionComputation::set_resolution,
                    DOC(hl_navigation, core, CachedCollisionComputation,
                        property_resolution))
      .def_property("min_angle", &CachedCollisionComputation::get_min_angle,
                    &CachedCollisionComputation::set_min_angle,
                    DOC(hl_navigation, core, CachedCollisionComputation,
                        property_min_angle))
      .def_property(
          "length", &CachedCollisionComputation::get_length,
          &CachedCollisionComputation::set_length,
          DOC(hl_navigation, core, CachedCollisionComputation, property_length))
      .def_property("max_distance",
                    &CachedCollisionComputation::get_max_distance,
                    &CachedCollisionComputation::set_max_distance,
                    DOC(hl_navigation, core, CachedCollisionComputation,
                        property_max_distance))
      .def_property(
          "speed", &CachedCollisionComputation::get_speed,
          &CachedCollisionComputation::set_speed,
          DOC(hl_navigation, core, CachedCollisionComputation, property_speed))
      .def("get_free_distance", &CachedCollisionComputation::get_free_distance,
           py::arg("dynamic"),
           DOC(hl_navigation, core, CachedCollisionComputation,
               get_free_distance));

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
