#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

#include <vector>

#include "hl_navigation/behavior.h"
#include "hl_navigation/kinematic.h"
#include "hl_navigation/yaml/yaml.h"
#include "hl_navigation_py/register.h"
#include "hl_navigation_py/yaml.h"
#include "hl_navigation_sim/experiment.h"
#include "hl_navigation_sim/scenario.h"
#include "hl_navigation_sim/scenarios/antipodal.h"
#include "hl_navigation_sim/scenarios/simple.h"
#include "hl_navigation_sim/scenarios/simple_with_init.h"
#include "hl_navigation_sim/world.h"
#include "hl_navigation_sim/yaml/experiment.h"
#include "hl_navigation_sim/yaml/scenario.h"
#include "hl_navigation_sim/yaml/world.h"

using namespace hl_navigation;
using namespace hl_navigation_sim;
namespace py = pybind11;

template <typename T>
struct get<T, py::object> {
  static T *ptr(const py::object &c) { return c.cast<T *>(); }
};

struct PyBehavior : public Behavior {
  using C = py::object;
  using Native = Behavior;

  static py::object make_type(const std::string &type) {
    py::module_ nav = py::module_::import("hl_navigation");
    return nav.attr("Behavior").attr("make_type")(type);
  }

  // Should cache
  static std::map<std::string, Properties> type_properties() {
    py::module_ nav = py::module_::import("hl_navigation");
    auto value = nav.attr("Behavior").attr("type_properties");
    return value.cast<std::map<std::string, Properties>>();
  }
};

struct PyKinematic : public Kinematic {
  using C = py::object;
  using Native = Kinematic;

  static py::object make_type(const std::string &type) {
    py::module_ nav = py::module_::import("hl_navigation");
    return nav.attr("Kinematic").attr("make_type")(type);
  }

  // Should cache
  static std::map<std::string, Properties> type_properties() {
    py::module_ nav = py::module_::import("hl_navigation");
    auto value = nav.attr("Kinematic").attr("type_properties");
    return value.cast<std::map<std::string, Properties>>();
  }
};

struct PyTask : Task, virtual PyHasRegister<Task> {
  /* Inherit the constructors */
  using Task::Task;
  using PyHasRegister<Task>::C;
  using Native = Task;

  void update(Agent *agent) override {
    PYBIND11_OVERRIDE(void, Task, update, agent);
  }

  bool done() const override { PYBIND11_OVERRIDE(bool, Task, done); }

  const Properties &get_properties() const override {
    const std::string t = get_type();
    if (type_properties().count(t)) {
      return type_properties().at(t);
    }
    return Native::get_properties();
  };
};

struct PyStateEstimation : StateEstimation,
                           virtual PyHasRegister<StateEstimation> {
  /* Inherit the constructors */
  using StateEstimation::StateEstimation;
  using PyHasRegister<StateEstimation>::C;
  using Native = StateEstimation;

  void update(Agent *agent) const override {
    PYBIND11_OVERRIDE(void, StateEstimation, update, agent);
  }

  void prepare(Agent *agent) const override {
    PYBIND11_OVERRIDE(void, StateEstimation, prepare, agent);
  }

  const Properties &get_properties() const override {
    const std::string t = get_type();
    if (type_properties().count(t)) {
      return type_properties().at(t);
    }
    return Native::get_properties();
  };
};

class PyAgent : public Agent {
 public:
  using B = PyBehavior;
  using K = PyKinematic;
  using T = PyTask;
  using S = PyStateEstimation;

  virtual ~PyAgent() = default;

  using C = py::object;

  PyAgent(float radius = 0.0f, const py::object &behavior = py::none(),
          const py::object &kinematic = py::none(),
          const py::object &task = py::none(),
          const py::object &estimation = py::none(),
          float control_period = 0.0f, unsigned id = 0)
      : Agent(radius, nullptr, nullptr, nullptr, nullptr, control_period, id) {
    set_kinematic(kinematic);
    set_behavior(behavior);
    set_state_estimation(estimation);
    set_task(task);
  }

  static py::object make(float radius = 0.0f,
                         const py::object &behavior = py::none(),
                         const py::object &kinematic = py::none(),
                         const py::object &task = py::none(),
                         const py::object &estimation = py::none(),
                         float control_period = 0.0f, unsigned id = 0) {
    auto a = std::make_shared<PyAgent>(radius, behavior, kinematic, task,
                                       estimation, control_period, id);
#if 0
    auto a = std::make_shared<PyAgent>(radius, nullptr, nullptr, nullptr,
                                       nullptr, control_period, id);
    a->set_kinematic(kinematic);
    a->set_behavior(behavior);
    a->set_state_estimation(estimation);
    a->set_task(task);
#endif
    return py::cast(a);
  }

  GeometricState *get_geometric_state() const override {
    try {
      return py_behavior.cast<GeometricState *>();
    } catch (const py::cast_error &e) {
      return nullptr;
    }
  }

  void set_kinematic(const py::object &obj) {
    py_kinematic = obj;
    Agent::set_kinematic(obj.cast<std::shared_ptr<Kinematic>>());
  }

  void set_behavior(const py::object &obj) {
    py_behavior = obj;
    Agent::set_behavior(obj.cast<std::shared_ptr<Behavior>>());
  }

  void set_state_estimation(const py::object &obj) {
    py_state_estimation = obj;
    Agent::set_state_estimation(obj.cast<std::shared_ptr<StateEstimation>>());
  }

  void set_task(const py::object &obj) {
    py_task = obj;
    Agent::set_task(obj.cast<std::shared_ptr<Task>>());
  }

 private:
  py::object py_kinematic;
  py::object py_behavior;
  py::object py_state_estimation;
  py::object py_task;
};

struct PyWorld : public World {
  /* Inherit the constructors */
  using World::World;
  using A = PyAgent;

  ~PyWorld() = default;

  std::vector<py::object> py_agents;

  void add_agent(const py::object &value) {
    py_agents.push_back(value);
    std::shared_ptr<Agent> agent = value.cast<std::shared_ptr<Agent>>();
    World::add_agent(agent);
  }
};

struct PyScenario : public Scenario, virtual PyHasRegister<Scenario> {
  /* Inherit the constructors */
  using Scenario::Scenario;
  using Native = Scenario;

  void init_world(World *world) override {
    PYBIND11_OVERRIDE(void, Scenario, init_world, world);
  }

  const Properties &get_properties() const override {
    const std::string t = PyHasRegister<Scenario>::get_type();
    if (type_properties().count(t)) {
      return type_properties().at(t);
    }
    return Native::get_properties();
  };
};

struct PyExperiment : public Experiment {
  /* Inherit the constructors */
  using Experiment::Experiment;

  ~PyExperiment() = default;

  // virtual ~PyExperiment() = default;

  py::object py_scenario;

  std::shared_ptr<World> make_world() override {
    return std::make_shared<PyWorld>();
  }

  std::string dump() override { return YAML::dump<PyExperiment>(this); }

  void set_scenario(const py::object &value) {
    py_scenario = value;
    scenario = value.cast<std::shared_ptr<Scenario>>();
  }
};

namespace YAML {

template <>
struct convert<PyAgent> {
  static Node encode(const PyAgent &rhs) {
    return convert<Agent>::encode(static_cast<const Agent &>(rhs));
  }
  static bool decode(const Node &node, PyAgent &rhs) {
    if (convert<Agent>::decode(node, static_cast<Agent &>(rhs))) {
      if (!rhs.nav_behavior && node["navigation_behavior"]) {
        auto value = load_node<PyBehavior>(node["navigation_behavior"]);
        rhs.set_behavior(value);
      }
      if (!rhs.kinematic && node["kinematic"]) {
        auto value = load_node<PyKinematic>(node["kinematic"]);
        rhs.set_kinematic(value);
      }
      if (!rhs.task && node["task"]) {
        auto value = load_node<PyTask>(node["task"]);
        rhs.set_task(value);
      }
      if (!rhs.state_estimation && node["state_estimation"]) {
        auto value = load_node<PyStateEstimation>(node["state_estimation"]);
        rhs.set_state_estimation(value);
      }
      return true;
    }
    return false;
  }
};

template <>
struct convert<std::shared_ptr<PyAgent>> {
  static Node encode(const std::shared_ptr<PyAgent> &rhs) {
    if (rhs) {
      return convert<PyAgent>::encode(*rhs);
    }
    return Node();
  }
  static bool decode(const Node &node, std::shared_ptr<PyAgent> &rhs) {
    rhs = std::make_shared<PyAgent>();
    if (convert<PyAgent>::decode(node, *rhs)) {
      return true;
    }
    rhs = nullptr;
    return false;
  }
};

PyWorld load_world(const Node &node) {
  PyWorld world;
  convert_world<PyAgent>::decode(node, world);
  return world;
}

py::object load_scenario(const Node &node) {
  auto obj = make_type_from_yaml_py<PyScenario>(node);
  if (obj.is_none()) {
    auto ws = std::make_shared<Scenario>();
    obj = py::cast(ws);
  }
  convert_scenario<PyWorld>::decode(node, obj.cast<Scenario &>());
  return obj;
};

std::string dump_scenario(Scenario *sampler) {
  auto node = convert_scenario<PyWorld>::encode(*sampler);
  Emitter out;
  out << node;
  return std::string(out.c_str());
};

// TODO(move to PyExperiment that creates a PyWorld instead of a World)
// Experiment load_experiment(const Node &node) {
//   Experiment experiment;
//   convert_experiment<PyAgent, PyBehavior, PyKinematic, PyTask,
//                      PyStateEstimation, PyWorld>::decode(node, experiment);
//   return experiment;
// };

// std::string dump_experiment(const Experiment *experiment) {
//   if (!experiment) return "";
//   // const auto node = convert_experiment<PyAgent, PyBehavior, PyKinematic,
//   // PyTask,
//   //                    PyStateEstimation,
//   //                    PyWorld>::encode(*experiment);
//   Node node;
//   Emitter out;
//   out << node;
//   return std::string(out.c_str());
// };

template <>
struct convert<PyExperiment> {
  static Node encode(const PyExperiment &rhs) {
    Node node = convert_experiment::encode(rhs);
    if (rhs.scenario) {
      node["world"] = convert_scenario<PyWorld>::encode(*rhs.scenario);
    }
    return node;
  }
  static bool decode(const Node &node, PyExperiment &rhs) {
    if (convert_experiment::decode(node, rhs)) {
      if (node["world"]) {
        rhs.set_scenario(load_scenario(node["world"]));
      }
      return true;
    }
    return false;
  }
};

}  // namespace YAML

PYBIND11_MODULE(_hl_navigation_sim, m) {
  declare_register<StateEstimation>(m, "StateEstimation");
  declare_register<Task>(m, "Task");
  declare_register<Scenario>(m, "Scenario");
  //  declare_register<PScenario>(m, "Scenario");

  py::class_<Entity, std::shared_ptr<Entity>>(m, "Entity")
    .def_readonly("_uid", &Entity::uid);

  py::class_<Wall, Entity, std::shared_ptr<Wall>>(m, "Wall")
    .def_readonly("line", &Wall::line);

  py::class_<Obstacle, Entity, std::shared_ptr<Obstacle>>(m, "Obstacle")
    .def_readonly("disc", &Obstacle::disc);

  py::class_<Agent, Entity, std::shared_ptr<Agent>>(m, "NativeAgent")
      .def_readwrite("id", &Agent::id)
      .def_readwrite("type", &Agent::type)
      .def_readwrite("radius", &Agent::radius)
      .def_readwrite("control_period", &Agent::control_period)
      .def_readwrite("pose", &Agent::pose)
      .def_property(
          "position", [](const Agent *agent) { return agent->pose.position; },
          [](Agent *agent, const Vector2 &value) {
            agent->pose.position = value;
          })
      .def_property(
          "orientation",
          [](const Agent *agent) { return agent->pose.orientation; },
          [](Agent *agent, const float value) {
            agent->pose.orientation = value;
          })
      .def_readwrite("twist", &Agent::twist)
      .def_property(
          "velocity", [](const Agent *agent) { return agent->twist.velocity; },
          [](Agent *agent, const Vector2 &value) {
            agent->twist.velocity = value;
          })
      .def_property(
          "angular_speed",
          [](const Agent *agent) { return agent->twist.angular_speed; },
          [](Agent *agent, const float value) {
            agent->twist.angular_speed = value;
          })
      .def_readwrite("cmd_twist", &Agent::cmd_twist)
      .def_readonly("nav_controller", &Agent::nav_controller)
      // .def_readwrite("task", &Agent::task)
      .def_property(
          "task", [](const Agent *agent) { return agent->task; },
          py::cpp_function(
              [](Agent *agent, const std::shared_ptr<Task> &value) {
                agent->task = value;
              },
              py::keep_alive<1, 2>()))
      // .def_readwrite("state_estimation", &Agent::state_estimation)
      .def_property(
          "state_estimation",
          [](const Agent *agent) { return agent->state_estimation; },
          py::cpp_function(
              [](Agent *agent, const std::shared_ptr<StateEstimation> &value) {
                agent->state_estimation = value;
              },
              py::keep_alive<1, 2>()))
      // .def_readwrite("nav_behavior", &Agent::nav_behavior)
      .def_property(
          "nav_behavior",
          [](const Agent *agent) { return agent->nav_behavior; },
          py::cpp_function(
              [](Agent *agent, const std::shared_ptr<Behavior> &value) {
                agent->nav_behavior = value;
              },
              py::keep_alive<1, 2>()))
      // .def_readwrite("kinematic", &Agent::kinematic)
      .def_property(
          "kinematic", [](const Agent *agent) { return agent->kinematic; },
          py::cpp_function(
              [](Agent *agent, const std::shared_ptr<Kinematic> &value) {
                agent->kinematic = value;
              },
              py::keep_alive<1, 2>()));

  py::class_<PyAgent, Agent, Entity, std::shared_ptr<PyAgent>>(m, "Agent",
                                                       py::dynamic_attr())
      .def(py::init<float, const py::object &,
                    const py::object &,const py::object &,
                    const py::object &, float, unsigned>(),
           py::arg("radius") = 0.0f, py::arg("behavior") = py::none(),
           py::arg("kinematic") = py::none(), py::arg("task") = py::none(),
           py::arg("state_estimation") = py::none(),
           py::arg("control_period") = 0.0f, py::arg("id") = 0)
      #if 0
      .def(py::init<float, std::shared_ptr<Behavior>,
                    std::shared_ptr<Kinematic>, std::shared_ptr<Task>,
                    std::shared_ptr<StateEstimation>, float, unsigned>(),
           py::arg("radius") = 0.0f, py::arg("behavior") = nullptr,
           py::arg("kinematic") = nullptr, py::arg("task") = nullptr,
           py::arg("state_estimation") = nullptr,
           py::arg("control_period") = 0.0f, py::arg("id") = 0)
      #endif
      .def_property("task", &PyAgent::get_task, &PyAgent::set_task)
      .def_property("state_estimation", &PyAgent::get_state_estimation,
                    &PyAgent::set_state_estimation)
      .def_property("nav_behavior", &PyAgent::get_behavior,
                    &PyAgent::set_behavior)
      .def_property("kinematic", &PyAgent::get_kinematic,
                    &PyAgent::set_kinematic)
      .def_property("controller", &PyAgent::get_controller, nullptr,
                    py::return_value_policy::reference);

  py::class_<BoundingBox>(m, "BoundingBox")
      .def(py::init<float, float, float, float>());

  py::class_<World>(m, "NativeWorld")
      .def(py::init<>())
      .def("run", &World::run)
      .def("add_agent", &World::add_agent, py::keep_alive<1, 2>())
      .def("add_obstacle", &World::add_obstacle)
      .def("add_wall", &World::add_wall)
      .def_readwrite("agents", &World::agents)
      .def_readwrite("walls", &World::walls)
      .def_readwrite("obstacles", &World::obstacles)
      .def("get_neighbors", &World::get_neighbors)
      .def_property("time", &World::get_time, nullptr)
      .def("dump",
           [](const World &world) {
             YAML::Emitter out;
             out << YAML::Node(world);
             return std::string(out.c_str());
           })
      .def_static("load", [](const std::string &yaml) {
        YAML::Node node = YAML::Load(yaml);
        return node.as<World>();
      });

  py::class_<PyWorld, World>(m, "World")
      .def(py::init<>())
      .def("add_agent", &PyWorld::add_agent);

  py::class_<StateEstimation, PyStateEstimation, HasRegister<StateEstimation>,
             HasProperties, std::shared_ptr<StateEstimation>>(m,
                                                              "StateEstimation")
      .def(py::init<World *>(), py::arg("world") = nullptr)
      .def_readwrite("world", &StateEstimation::world,
                     py::return_value_policy::reference)
      .def("update", &StateEstimation::update)
      .def_property(
          "type", [](StateEstimation *obj) { return obj->get_type(); }, nullptr)
      .def("prepare", &StateEstimation::prepare);

  py::class_<BoundedStateEstimation, StateEstimation,
             std::shared_ptr<BoundedStateEstimation>>(m,
                                                      "BoundedStateEstimation")
      .def(py::init<World *, float, float>(), py::arg("world") = nullptr,
           py::arg("field_of_view") = 0.0, py::arg("range_of_view") = 0.0)
      .def_property("field_of_view", &BoundedStateEstimation::get_field_of_view,
                    &BoundedStateEstimation::set_field_of_view)
      .def_property("range_of_view", &BoundedStateEstimation::get_range_of_view,
                    &BoundedStateEstimation::set_range_of_view)
      .def("neighbors", &BoundedStateEstimation::neighbors);

  py::class_<Task, PyTask, HasRegister<Task>, HasProperties,
             std::shared_ptr<Task>>(m, "Task")
      .def(py::init<>())
      .def("update", &Task::update)
      .def_property(
          "type", [](Task *obj) { return obj->get_type(); }, nullptr)
      .def("done", &Task::done);

  py::class_<WayPointsTask, Task, std::shared_ptr<WayPointsTask>>(
      m, "WayPointsTask")
      .def(py::init<Waypoints, bool, float>(),
           py::arg("waypoints") = Waypoints{},
           py::arg("loop") = WayPointsTask::default_loop,
           py::arg("tolerance") = WayPointsTask::default_tolerance)
      .def_property("waypoints", &WayPointsTask::get_waypoints,
                    &WayPointsTask::set_waypoints)
      .def_property("tolerance", &WayPointsTask::get_tolerance,
                    &WayPointsTask::set_tolerance)
      .def("add_callback", &WayPointsTask::add_callback)
      .def_property("loop", &WayPointsTask::get_loop, &WayPointsTask::set_loop);

  py::class_<PyExperiment>(m, "Experiment")
      .def(py::init<float, int>(), py::arg("time_step") = 0.1,
           py::arg("steps") = 1000)
      .def_readwrite("time_step", &Experiment::time_step)
      .def_readwrite("steps", &Experiment::steps)
      .def_property(
          "scenario", [](const PyExperiment *exp) { return exp->scenario; },
          [](PyExperiment *exp, const std::shared_ptr<Scenario> &value) {
            exp->scenario = value;
          })
      .def_readonly("world", &Experiment::world)
      .def_readwrite("runs", &Experiment::runs)
      .def_readwrite("save_directory", &Experiment::save_directory)
      .def_readwrite("name", &Experiment::name)
      .def_property("path", &Experiment::get_path, nullptr)
      .def("add_callback", &Experiment::add_callback)
      .def("run_once", &Experiment::run_once)
      .def("run", &Experiment::run)
      .def_property(
          "record_pose",
          [](const PyExperiment *exp) { return exp->trace.record_pose; },
          [](PyExperiment *exp, bool value) { exp->trace.record_pose = value; })
      .def_property(
          "record_twist",
          [](const PyExperiment *exp) { return exp->trace.record_twist; },
          [](PyExperiment *exp, bool value) {
            exp->trace.record_twist = value;
          })
      .def_property(
          "record_cmd",
          [](const PyExperiment *exp) { return exp->trace.record_cmd; },
          [](PyExperiment *exp, bool value) { exp->trace.record_cmd = value; })
      .def_property(
          "record_target",
          [](const PyExperiment *exp) { return exp->trace.record_target; },
          [](PyExperiment *exp, bool value) {
            exp->trace.record_target = value;
          })
      .def("run", &Experiment::run);

  py::class_<Scenario, PyScenario, HasRegister<Scenario>, HasProperties,
             std::shared_ptr<Scenario>>(m, "Scenario")
      .def(py::init<>())
      .def("init_world", &Scenario::init_world)
      // .def("sample", &Scenario::sample)
      // .def("reset", &Scenario::reset)
      // .def_readwrite("groups", &Scenario::groups)
      .def_readwrite("obstacles", &Scenario::obstacles)
      .def_readwrite("walls", &Scenario::walls)
      .def_property(
          "initializers", [](Scenario *ws) { return ws->initializers; },
          nullptr)
      .def("add_init", &Scenario::add_init);

  py::class_<AntipodalScenario, Scenario, std::shared_ptr<AntipodalScenario>>(
      m, "Antipodal")
      .def(py::init<>())
      .def_property("radius", &AntipodalScenario::get_radius,
                    &AntipodalScenario::set_radius)
      .def_property("tolerance", &AntipodalScenario::get_tolerance,
                    &AntipodalScenario::set_tolerance);

  m.def("load_task", &YAML::load_py<PyTask>);
  m.def("load_state_estimation", &YAML::load_py<PyStateEstimation>);
  m.def("load_agent", [](const std::string &yaml) {
    YAML::Node node = YAML::Load(yaml);
    return node.as<PyAgent>();
  });
  m.def("load_world", [](const std::string &yaml) -> PyWorld {
    YAML::Node node = YAML::Load(yaml);
    return YAML::load_world(node);
  });
  m.def("load_scenario", [](const std::string &yaml) {
    YAML::Node node = YAML::Load(yaml);
    return YAML::load_scenario(node);
  });
  m.def("load_experiment", [](const std::string &yaml) {
    YAML::Node node = YAML::Load(yaml);
    return node.as<PyExperiment>();
  });
  m.def("dump", &YAML::dump<Task>);
  m.def("dump", &YAML::dump<StateEstimation>);
  m.def("dump", &YAML::dump<World>);
  m.def("dump", &YAML::dump_scenario);
  // m.def("dump", &YAML::dump<Scenario>);
  m.def("dump", &YAML::dump<Agent>);
  m.def("dump", &YAML::dump<PyAgent>);
  m.def("dump", &YAML::dump<Experiment>);
  m.def("dump", &YAML::dump<PyExperiment>);
}
