/*
  This file contains docstrings for use in the Python bindings.
  Do not edit! They were automatically extracted by pybind11_mkdoc.
 */

#define __EXPAND(x)                                      x
#define __COUNT(_1, _2, _3, _4, _5, _6, _7, COUNT, ...)  COUNT
#define __VA_SIZE(...)                                   __EXPAND(__COUNT(__VA_ARGS__, 7, 6, 5, 4, 3, 2, 1, 0))
#define __CAT1(a, b)                                     a ## b
#define __CAT2(a, b)                                     __CAT1(a, b)
#define __DOC1(n1)                                       __doc_##n1
#define __DOC2(n1, n2)                                   __doc_##n1##_##n2
#define __DOC3(n1, n2, n3)                               __doc_##n1##_##n2##_##n3
#define __DOC4(n1, n2, n3, n4)                           __doc_##n1##_##n2##_##n3##_##n4
#define __DOC5(n1, n2, n3, n4, n5)                       __doc_##n1##_##n2##_##n3##_##n4##_##n5
#define __DOC6(n1, n2, n3, n4, n5, n6)                   __doc_##n1##_##n2##_##n3##_##n4##_##n5##_##n6
#define __DOC7(n1, n2, n3, n4, n5, n6, n7)               __doc_##n1##_##n2##_##n3##_##n4##_##n5##_##n6##_##n7
#define DOC(...)                                         __EXPAND(__EXPAND(__CAT2(__DOC, __VA_SIZE(__VA_ARGS__)))(__VA_ARGS__))

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif


static const char *__doc_YAML_convert = R"doc()doc";

static const char *__doc_YAML_convert_2 = R"doc()doc";

static const char *__doc_YAML_convert_3 = R"doc()doc";

static const char *__doc_YAML_convert_4 = R"doc()doc";

static const char *__doc_YAML_convert_5 = R"doc()doc";

static const char *__doc_YAML_convert_6 = R"doc()doc";

static const char *__doc_YAML_convert_7 = R"doc()doc";

static const char *__doc_YAML_convert_8 = R"doc()doc";

static const char *__doc_YAML_convert_9 = R"doc()doc";

static const char *__doc_YAML_convert_10 = R"doc()doc";

static const char *__doc_YAML_convert_11 = R"doc()doc";

static const char *__doc_YAML_convert_12 = R"doc()doc";

static const char *__doc_YAML_convert_13 = R"doc()doc";

static const char *__doc_YAML_convert_14 = R"doc()doc";

static const char *__doc_YAML_convert_decode = R"doc()doc";

static const char *__doc_YAML_convert_decode_2 = R"doc()doc";

static const char *__doc_YAML_convert_decode_3 = R"doc()doc";

static const char *__doc_YAML_convert_decode_4 = R"doc()doc";

static const char *__doc_YAML_convert_decode_5 = R"doc()doc";

static const char *__doc_YAML_convert_decode_6 = R"doc()doc";

static const char *__doc_YAML_convert_decode_7 = R"doc()doc";

static const char *__doc_YAML_convert_decode_8 = R"doc()doc";

static const char *__doc_YAML_convert_decode_9 = R"doc()doc";

static const char *__doc_YAML_convert_decode_10 = R"doc()doc";

static const char *__doc_YAML_convert_decode_11 = R"doc()doc";

static const char *__doc_YAML_convert_decode_12 = R"doc()doc";

static const char *__doc_YAML_convert_encode = R"doc()doc";

static const char *__doc_YAML_convert_encode_2 = R"doc()doc";

static const char *__doc_YAML_convert_encode_3 = R"doc()doc";

static const char *__doc_YAML_convert_encode_4 = R"doc()doc";

static const char *__doc_YAML_convert_encode_5 = R"doc()doc";

static const char *__doc_YAML_convert_encode_6 = R"doc()doc";

static const char *__doc_YAML_convert_encode_7 = R"doc()doc";

static const char *__doc_YAML_convert_encode_8 = R"doc()doc";

static const char *__doc_YAML_convert_encode_9 = R"doc()doc";

static const char *__doc_YAML_convert_encode_10 = R"doc()doc";

static const char *__doc_YAML_convert_encode_11 = R"doc()doc";

static const char *__doc_YAML_convert_encode_12 = R"doc()doc";

static const char *__doc_YAML_convert_encode_13 = R"doc()doc";

static const char *__doc_YAML_convert_encode_14 = R"doc()doc";

static const char *__doc_YAML_convert_experiment = R"doc()doc";

static const char *__doc_YAML_convert_experiment_decode = R"doc()doc";

static const char *__doc_YAML_convert_experiment_encode = R"doc()doc";

static const char *__doc_YAML_convert_scenario = R"doc()doc";

static const char *__doc_YAML_convert_scenario_decode = R"doc()doc";

static const char *__doc_YAML_convert_scenario_encode = R"doc()doc";

static const char *__doc_YAML_convert_world = R"doc()doc";

static const char *__doc_YAML_convert_world_decode = R"doc()doc";

static const char *__doc_YAML_convert_world_encode = R"doc()doc";

static const char *__doc_YAML_decode_sr = R"doc()doc";

static const char *__doc_YAML_encode_sr = R"doc()doc";

static const char *__doc_YAML_property_sampler = R"doc()doc";

static const char *__doc_YAML_read_sampler = R"doc()doc";

static const char *__doc_hl_navigation_sim_Agent =
R"doc(This class describes an agent.

The agent navigates in the environment using a task, a state
estimation, a kinematic and a behavior, and a controller.

Agents have a circular shape which should match the shape of their
navigation :py:class:`Behavior`.

The agent main API are :py:meth:`update` and :py:attr:`actuate` which are called
during :py:meth:`World.update` to first compute the agent control command
and then to actuate it.

The role of task and state estimation is to provide goals and
environment state (perception) to the behavior in :py:meth:`update`, while
actuation of the behavior output, is taken and actuated this class
when calling :py:attr:`actuate`.

Agents also have a public identifies :py:attr:`id` that is accessible by the
other agents' state estimation and may be passed to their behavior as
:py:attr:`Neighbor.id`. This identifier may not be unique
(e.g., may be used to identifies *groups* of agents).

Agents runs their update at the rate set by :py:attr:`control_period`. That
is, if the world is updated at a faster rate, the agent does not
update its control each time :py:meth:`update` is called.)doc";

static const char *__doc_hl_navigation_sim_Agent_2 = R"doc()doc";

static const char *__doc_hl_navigation_sim_Agent_3 = R"doc()doc";

static const char *__doc_hl_navigation_sim_AgentSampler =
R"doc(Implement an agent sample.

Defines the same fields as :py:class:`Agent` but as sampler of the respective
type.

Template parameter ``W``:
    The world type that the agents belong too. Used to generalize from
    C++ to Python.)doc";

static const char *__doc_hl_navigation_sim_AgentSampler_AgentSampler = R"doc(Constructs a new instance.)doc";

static const char *__doc_hl_navigation_sim_AgentSampler_add_to_world = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_AgentSampler_behavior = R"doc()doc";

static const char *__doc_hl_navigation_sim_AgentSampler_control_period = R"doc()doc";

static const char *__doc_hl_navigation_sim_AgentSampler_id = R"doc()doc";

static const char *__doc_hl_navigation_sim_AgentSampler_kinematics = R"doc()doc";

static const char *__doc_hl_navigation_sim_AgentSampler_name = R"doc()doc";

static const char *__doc_hl_navigation_sim_AgentSampler_number = R"doc()doc";

static const char *__doc_hl_navigation_sim_AgentSampler_orientation = R"doc()doc";

static const char *__doc_hl_navigation_sim_AgentSampler_position = R"doc()doc";

static const char *__doc_hl_navigation_sim_AgentSampler_radius = R"doc()doc";

static const char *__doc_hl_navigation_sim_AgentSampler_reset = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_AgentSampler_s = R"doc()doc";

static const char *__doc_hl_navigation_sim_AgentSampler_state_estimation = R"doc()doc";

static const char *__doc_hl_navigation_sim_AgentSampler_task = R"doc()doc";

static const char *__doc_hl_navigation_sim_AgentSampler_type = R"doc()doc";

static const char *__doc_hl_navigation_sim_Agent_Agent =
R"doc(Constructs a new instance.

:param radius:
    The radius of the agent

:param behavior:
    The behavior

:param kinematics:
    The kinematics

:param task:
    The task

:param estimation:
    The estimation

:param control_period:
    The control period

:param id:
    The public identifier)doc";

static const char *__doc_hl_navigation_sim_Agent_actuate =
R"doc(Actuate the current agent control command.

:param dt:
    The time step)doc";

static const char *__doc_hl_navigation_sim_Agent_as_neighbor =
R"doc(Returns a neighbor representation of the agent with the same shape,
position and id.

:return:
    The neighbor representation)doc";

static const char *__doc_hl_navigation_sim_Agent_behavior = R"doc()doc";

static const char *__doc_hl_navigation_sim_Agent_cmd_twist = R"doc(The last control command)doc";

static const char *__doc_hl_navigation_sim_Agent_collision_correction = R"doc()doc";

static const char *__doc_hl_navigation_sim_Agent_control_deadline = R"doc()doc";

static const char *__doc_hl_navigation_sim_Agent_control_period = R"doc(The control period)doc";

static const char *__doc_hl_navigation_sim_Agent_controller = R"doc()doc";

static const char *__doc_hl_navigation_sim_Agent_get_behavior =
R"doc(Gets the navigation behavior.

:return:
    The navigation behavior.)doc";

static const char *__doc_hl_navigation_sim_Agent_get_controller =
R"doc(Gets the navigation controller.

:return:
    The controller.)doc";

static const char *__doc_hl_navigation_sim_Agent_get_geometric_state =
R"doc(Gets the geometric state if any, i.e., if the behavior is a subclass
of :py:class:`GeometricState`

@private

:return:
    The geometric state or ``nullptr`` if the behavior is not a
    subclass of :py:class:`GeometricState`)doc";

static const char *__doc_hl_navigation_sim_Agent_get_kinematics =
R"doc(Gets the kinematics.

:return:
    The kinematics.)doc";

static const char *__doc_hl_navigation_sim_Agent_get_state_estimation =
R"doc(Gets the state estimation.

:return:
    The state estimation.)doc";

static const char *__doc_hl_navigation_sim_Agent_get_task =
R"doc(Gets the task.

:return:
    The task.)doc";

static const char *__doc_hl_navigation_sim_Agent_id = R"doc(The agent public identifier)doc";

static const char *__doc_hl_navigation_sim_Agent_idle =
R"doc(Returns whether the task is done and the control is idle.

:return:
    False if it has an active task or if the control is running)doc";

static const char *__doc_hl_navigation_sim_Agent_kinematics = R"doc()doc";

static const char *__doc_hl_navigation_sim_Agent_make =
R"doc(Factory method to build an agent

:param radius:
    The radius of the agent

:param behavior:
    The behavior

:param kinematics:
    The kinematics

:param task:
    The task

:param estimation:
    The estimation

:param control_period:
    The control period

:param id:
    The public identifier

:return:
    A new agent)doc";

static const char *__doc_hl_navigation_sim_Agent_pose = R"doc(The current pose)doc";

static const char *__doc_hl_navigation_sim_Agent_radius = R"doc(The agent radius)doc";

static const char *__doc_hl_navigation_sim_Agent_set_behavior =
R"doc(Sets the navigation behavior.

Automatically set the behavior radius and kinematics to match the
agent.

:param value:
    The desired value)doc";

static const char *__doc_hl_navigation_sim_Agent_set_kinematics =
R"doc(Sets the kinematics.

:param value:
    The desired value)doc";

static const char *__doc_hl_navigation_sim_Agent_set_state_estimation =
R"doc(Sets the state estimation.

:param value:
    The desired value)doc";

static const char *__doc_hl_navigation_sim_Agent_set_task =
R"doc(Sets the task.

:param value:
    The desired value)doc";

static const char *__doc_hl_navigation_sim_Agent_state_estimation = R"doc()doc";

static const char *__doc_hl_navigation_sim_Agent_tags =
R"doc(A set of tags used to label the agent.

Tags are mainly used to add meta-information about an agent, for
instance by the :py:class:`Group` that generated
it (if any), to simplify analysis.)doc";

static const char *__doc_hl_navigation_sim_Agent_task = R"doc()doc";

static const char *__doc_hl_navigation_sim_Agent_twist = R"doc(The current twist)doc";

static const char *__doc_hl_navigation_sim_Agent_type =
R"doc(The type of the agent.

The agent type should not used by the neighbors state estimation. It
is mainly used internally to draw the agents in the UI.)doc";

static const char *__doc_hl_navigation_sim_Agent_update =
R"doc(Tick the agent for a time step.

:param dt:
    The time step

:param time:
    The current time

:param world:
    The that the agent is part of)doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario =
R"doc(A scenario that place the agents around a circle at regular intervals
and task them to reach the opposite ("antipode") side.)doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_AntipodalScenario = R"doc(Constructs a new instance.)doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_get_orientation_noise =
R"doc(Gets the orientation.

:return:
    The orientation noise.)doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_get_position_noise =
R"doc(Gets the position_noise.

:return:
    The position_noise.)doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_get_properties = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_get_radius =
R"doc(Gets the circle radius.

:return:
    The radius.)doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_get_shuffle =
R"doc(Gets whether it should shuffle the agents before initializing them.

:return:
    True if it shuffles the agents.)doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_get_tolerance =
R"doc(Gets the goal tolerance.

:return:
    The tolerance.)doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_get_type = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_init_world = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_orientation_noise = R"doc()doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_position_noise = R"doc()doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_radius = R"doc()doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_set_orientation_noise =
R"doc(Sets the position_noise.

:param value:
    The desired value)doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_set_position_noise =
R"doc(Sets the position noise.

:param value:
    The desired value)doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_set_radius =
R"doc(Sets the circle radius.

:param value:
    The desired value)doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_set_shuffle =
R"doc(Sets whether it should shuffle the agents.

:param value:
    The desired value)doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_set_tolerance =
R"doc(Sets the goal tolerance.

:param value:
    The desired value)doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_shuffle = R"doc()doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_tolerance = R"doc()doc";

static const char *__doc_hl_navigation_sim_BehaviorSampler =
R"doc(Samples :py:class:`Behavior`

Template parameter ``T``:
    The type of the behavior root class. Used to generalize from C++
    to Python.

Defines the same fields as :py:class:`Behavior` but as sampler
of the respective type.)doc";

static const char *__doc_hl_navigation_sim_BehaviorSampler_BehaviorSampler =
R"doc(Constructs a new instance.

:param type:
    The registered type name)doc";

static const char *__doc_hl_navigation_sim_BehaviorSampler_heading = R"doc()doc";

static const char *__doc_hl_navigation_sim_BehaviorSampler_horizon = R"doc()doc";

static const char *__doc_hl_navigation_sim_BehaviorSampler_optimal_angular_speed = R"doc()doc";

static const char *__doc_hl_navigation_sim_BehaviorSampler_optimal_speed = R"doc()doc";

static const char *__doc_hl_navigation_sim_BehaviorSampler_reset = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_BehaviorSampler_rotation_tau = R"doc()doc";

static const char *__doc_hl_navigation_sim_BehaviorSampler_s = R"doc()doc";

static const char *__doc_hl_navigation_sim_BehaviorSampler_safety_margin = R"doc()doc";

static const char *__doc_hl_navigation_sim_BoundedStateEstimation =
R"doc(Perfect state estimation within a range from the agent.

*Properties*: range_of_view (float))doc";

static const char *__doc_hl_navigation_sim_BoundedStateEstimation_BoundedStateEstimation =
R"doc(Constructs a new instance.

:param range_of_view_:
    The range of view)doc";

static const char *__doc_hl_navigation_sim_BoundedStateEstimation_get_properties = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_BoundedStateEstimation_get_range_of_view =
R"doc(Gets the range of view.

:return:
    The range of view.)doc";

static const char *__doc_hl_navigation_sim_BoundedStateEstimation_get_type = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_BoundedStateEstimation_neighbors_of_agent =
R"doc(Gets the neighbors

:param agent:
    The agent

:return:
    { description_of_the_return_value })doc";

static const char *__doc_hl_navigation_sim_BoundedStateEstimation_prepare = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_BoundedStateEstimation_range_of_view = R"doc()doc";

static const char *__doc_hl_navigation_sim_BoundedStateEstimation_set_range_of_view =
R"doc(Sets the range of view.

:param value:
    The new value)doc";

static const char *__doc_hl_navigation_sim_BoundedStateEstimation_update = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_ChoiceSampler =
R"doc(An inexhaustible generator that randomly pick sequences from a
collection of values.

Template parameter ``T``:
    The sampled type)doc";

static const char *__doc_hl_navigation_sim_ChoiceSampler_ChoiceSampler =
R"doc(Construct an instance

:param values:
    The values to be sampled randomly sequence)doc";

static const char *__doc_hl_navigation_sim_ChoiceSampler_dist = R"doc()doc";

static const char *__doc_hl_navigation_sim_ChoiceSampler_done = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_ChoiceSampler_s = R"doc()doc";

static const char *__doc_hl_navigation_sim_ChoiceSampler_values = R"doc()doc";

static const char *__doc_hl_navigation_sim_CollisionsScenario = R"doc()doc";

static const char *__doc_hl_navigation_sim_CollisionsScenario_CollisionsScenario = R"doc()doc";

static const char *__doc_hl_navigation_sim_CollisionsScenario_behavior_name = R"doc()doc";

static const char *__doc_hl_navigation_sim_CollisionsScenario_control_period = R"doc()doc";

static const char *__doc_hl_navigation_sim_CollisionsScenario_get_properties = R"doc()doc";

static const char *__doc_hl_navigation_sim_CollisionsScenario_get_type = R"doc()doc";

static const char *__doc_hl_navigation_sim_CollisionsScenario_init_world = R"doc()doc";

static const char *__doc_hl_navigation_sim_ConstantSampler =
R"doc(An inexhaustible generator that always returns the same value.

Template parameter ``T``:
    The sampled type)doc";

static const char *__doc_hl_navigation_sim_ConstantSampler_ConstantSampler =
R"doc(Construct an instance

:param value:
    The constant value)doc";

static const char *__doc_hl_navigation_sim_ConstantSampler_s = R"doc()doc";

static const char *__doc_hl_navigation_sim_ConstantSampler_value = R"doc()doc";

static const char *__doc_hl_navigation_sim_CorridorScenario = R"doc()doc";

static const char *__doc_hl_navigation_sim_CorridorScenario_CorridorScenario = R"doc()doc";

static const char *__doc_hl_navigation_sim_CorridorScenario_add_safety_to_agent_margin = R"doc()doc";

static const char *__doc_hl_navigation_sim_CorridorScenario_agent_margin = R"doc()doc";

static const char *__doc_hl_navigation_sim_CorridorScenario_get_add_safety_to_agent_margin = R"doc()doc";

static const char *__doc_hl_navigation_sim_CorridorScenario_get_agent_margin = R"doc()doc";

static const char *__doc_hl_navigation_sim_CorridorScenario_get_length = R"doc()doc";

static const char *__doc_hl_navigation_sim_CorridorScenario_get_properties = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_CorridorScenario_get_type = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_CorridorScenario_get_width = R"doc()doc";

static const char *__doc_hl_navigation_sim_CorridorScenario_init_world = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_CorridorScenario_length = R"doc()doc";

static const char *__doc_hl_navigation_sim_CorridorScenario_set_add_safety_to_agent_margin = R"doc()doc";

static const char *__doc_hl_navigation_sim_CorridorScenario_set_agent_margin = R"doc()doc";

static const char *__doc_hl_navigation_sim_CorridorScenario_set_length = R"doc()doc";

static const char *__doc_hl_navigation_sim_CorridorScenario_set_width = R"doc()doc";

static const char *__doc_hl_navigation_sim_CorridorScenario_width = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_2 = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_CrossScenario = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_CrossScenario_2 = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_add_safety_to_agent_margin = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_agent_margin = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_behavior_name = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_control_period = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_get_add_safety_to_agent_margin = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_get_agent_margin = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_get_properties = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_CrossScenario_get_properties_2 = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_get_side = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_get_target_margin = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_get_tolerance = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_get_type = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_CrossScenario_get_type_2 = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_init_world = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_CrossScenario_init_world_2 = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_margin = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_number = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_radius = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_set_add_safety_to_agent_margin = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_set_agent_margin = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_set_side = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_set_target_margin = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_set_tolerance = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_side = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_target_margin = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_tolerance = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossTorusScenario = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossTorusScenario_CrossTorusScenario = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossTorusScenario_add_safety_to_agent_margin = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossTorusScenario_agent_margin = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossTorusScenario_get_add_safety_to_agent_margin = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossTorusScenario_get_agent_margin = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossTorusScenario_get_properties = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_CrossTorusScenario_get_side = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossTorusScenario_get_type = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_CrossTorusScenario_init_world = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_CrossTorusScenario_set_add_safety_to_agent_margin = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossTorusScenario_set_agent_margin = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossTorusScenario_set_side = R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossTorusScenario_side = R"doc()doc";

static const char *__doc_hl_navigation_sim_Entity =
R"doc(Super-class that adds a unique ID to world entities.

This unique ID should not be fed to navigation behaviors, but only
used internally by the simulation, for instance, to identify entities
in a UI.)doc";

static const char *__doc_hl_navigation_sim_Entity_Entity = R"doc(Constructs a new instance.)doc";

static const char *__doc_hl_navigation_sim_Entity_Entity_2 =
R"doc(Constructs a new instance.

:param id:
    The identifier)doc";

static const char *__doc_hl_navigation_sim_Entity_uid = R"doc(Unique identifier)doc";

static const char *__doc_hl_navigation_sim_Experiment =
R"doc(An experiment supervise the execution of a simulation.

When performing one run using :py:meth:`run_once`

1. it initializes a world from its scenario by calling :py:meth:`Scenario.init_world` and setup the dataset.

2. in run the simulation, step by step, collecting and storing data
   about the agents in the dataset.)doc";

static const char *__doc_hl_navigation_sim_Experiment_2 =
R"doc(An experiment supervise the execution of a simulation.

When performing one run using :py:meth:`run_once`

1. it initializes a world from its scenario by calling :py:meth:`Scenario.init_world` and setup the dataset.

2. in run the simulation, step by step, collecting and storing data
   about the agents in the dataset.)doc";

static const char *__doc_hl_navigation_sim_Experiment_Experiment =
R"doc(Constructs a new instance.

:param time_step:
    The simulation time step

:param steps:
    The number of simulation steps)doc";

static const char *__doc_hl_navigation_sim_Experiment_add_callback =
R"doc(Adds a callback to be executed after each simulation step.

:param value:
    The callback)doc";

static const char *__doc_hl_navigation_sim_Experiment_callbacks = R"doc()doc";

static const char *__doc_hl_navigation_sim_Experiment_dump = R"doc()doc";

static const char *__doc_hl_navigation_sim_Experiment_file = R"doc()doc";

static const char *__doc_hl_navigation_sim_Experiment_get_path =
R"doc(Gets the database path.

:return:
    The path.)doc";

static const char *__doc_hl_navigation_sim_Experiment_get_world = R"doc()doc";

static const char *__doc_hl_navigation_sim_Experiment_init_dataset = R"doc()doc";

static const char *__doc_hl_navigation_sim_Experiment_init_dataset_run = R"doc()doc";

static const char *__doc_hl_navigation_sim_Experiment_init_run = R"doc()doc";

static const char *__doc_hl_navigation_sim_Experiment_initialized = R"doc()doc";

static const char *__doc_hl_navigation_sim_Experiment_make_world = R"doc()doc";

static const char *__doc_hl_navigation_sim_Experiment_name = R"doc(The name of the experiment)doc";

static const char *__doc_hl_navigation_sim_Experiment_run = R"doc(Perform all runs)doc";

static const char *__doc_hl_navigation_sim_Experiment_run_group = R"doc()doc";

static const char *__doc_hl_navigation_sim_Experiment_run_index = R"doc()doc";

static const char *__doc_hl_navigation_sim_Experiment_run_once =
R"doc(Perform a single run

:param seed:
    The index (and random seed) of the run)doc";

static const char *__doc_hl_navigation_sim_Experiment_runs = R"doc(Number of runs to perform)doc";

static const char *__doc_hl_navigation_sim_Experiment_save_directory = R"doc(Where to save the results)doc";

static const char *__doc_hl_navigation_sim_Experiment_scenario = R"doc(The scenario)doc";

static const char *__doc_hl_navigation_sim_Experiment_step = R"doc()doc";

static const char *__doc_hl_navigation_sim_Experiment_steps = R"doc(Number of steps per run)doc";

static const char *__doc_hl_navigation_sim_Experiment_terminate_when_all_idle = R"doc()doc";

static const char *__doc_hl_navigation_sim_Experiment_time_step = R"doc(Simulation time step)doc";

static const char *__doc_hl_navigation_sim_Experiment_trace = R"doc(The trace)doc";

static const char *__doc_hl_navigation_sim_Experiment_world = R"doc()doc";

static const char *__doc_hl_navigation_sim_Ghost = R"doc(Ghost agents used in worlds that have a lattice.)doc";

static const char *__doc_hl_navigation_sim_Ghost_Ghost = R"doc()doc";

static const char *__doc_hl_navigation_sim_GridSampler =
R"doc(RegularSamplerly sample from a grid of points.

If wrap is not set to :py:attr:`Wrap.terminate`, the generator is
inexhaustible, else it will be exhausted after looping once through
all values once.)doc";

static const char *__doc_hl_navigation_sim_GridSampler_GridSampler =
R"doc(Construct an instance

:param from:
    One corner of the covered area

:param to:
    The opposite corner of the covered area

:param numbers:
    The size of the grid, i.e., the number of points along the x- and
    y-axis.

:param wrap:
    How it should wrap at end of the covered area)doc";

static const char *__doc_hl_navigation_sim_GridSampler_done = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_GridSampler_from = R"doc()doc";

static const char *__doc_hl_navigation_sim_GridSampler_numbers = R"doc()doc";

static const char *__doc_hl_navigation_sim_GridSampler_s = R"doc()doc";

static const char *__doc_hl_navigation_sim_GridSampler_step = R"doc()doc";

static const char *__doc_hl_navigation_sim_GridSampler_to = R"doc()doc";

static const char *__doc_hl_navigation_sim_GridSampler_wrap = R"doc()doc";

static const char *__doc_hl_navigation_sim_KinematicsSampler =
R"doc(Samples :py:class:`Kinematics`

Template parameter ``T``:
    The type of the behavior root class. Used to generalize from C++
    to Python.

Defines the same fields as :py:class:`Kinematics` but as
sampler of the respective type.)doc";

static const char *__doc_hl_navigation_sim_KinematicsSampler_KinematicsSampler =
R"doc(Constructs a new instance.

:param type:
    The registered type name)doc";

static const char *__doc_hl_navigation_sim_KinematicsSampler_max_angular_speed = R"doc()doc";

static const char *__doc_hl_navigation_sim_KinematicsSampler_max_speed = R"doc()doc";

static const char *__doc_hl_navigation_sim_KinematicsSampler_reset = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_KinematicsSampler_s = R"doc()doc";

static const char *__doc_hl_navigation_sim_NormalSampler =
R"doc(Sample randomly from a uniform distribution.

Values are optionally clamped when min and/or max are provided.

Only defined if T is a number.

Template parameter ``T``:
    The sampled type)doc";

static const char *__doc_hl_navigation_sim_NormalSampler_NormalSampler =
R"doc(Construct an instance

:param mean:
    The mean

:param std_dev:
    The standard deviation

:param min:
    The minimum value

:param max:
    The maximum value)doc";

static const char *__doc_hl_navigation_sim_NormalSampler_dist = R"doc()doc";

static const char *__doc_hl_navigation_sim_NormalSampler_max = R"doc()doc";

static const char *__doc_hl_navigation_sim_NormalSampler_mean = R"doc()doc";

static const char *__doc_hl_navigation_sim_NormalSampler_min = R"doc()doc";

static const char *__doc_hl_navigation_sim_NormalSampler_s = R"doc()doc";

static const char *__doc_hl_navigation_sim_NormalSampler_std_dev = R"doc()doc";

static const char *__doc_hl_navigation_sim_Obstacle = R"doc(A static obstacle with circular shape)doc";

static const char *__doc_hl_navigation_sim_Obstacle_Obstacle =
R"doc(Constructs a new instance.

:param position:
    The position of the circle

:param radius:
    The radius of the circle)doc";

static const char *__doc_hl_navigation_sim_Obstacle_Obstacle_2 = R"doc(Constructs a new instance.)doc";

static const char *__doc_hl_navigation_sim_Obstacle_Obstacle_3 =
R"doc(Constructs a new instance.

:param disc:
    A disc)doc";

static const char *__doc_hl_navigation_sim_Obstacle_disc = R"doc(The disc.)doc";

static const char *__doc_hl_navigation_sim_Obstacle_operator_Disc = R"doc(Disc conversion operator.)doc";

static const char *__doc_hl_navigation_sim_PropertySampler =
R"doc(This class wraps generic :py:class:`Sampler`<T> to generate values of type
:py:class:`Field`.)doc";

static const char *__doc_hl_navigation_sim_PropertySampler_PropertySampler =
R"doc(Constructs an instance

:param value:
    A sampler

Template parameter ``T``:
    Needs to be one of the type of :py:class:`Field`)doc";

static const char *__doc_hl_navigation_sim_PropertySampler_PropertySampler_2 =
R"doc(Constructs an instance

:param value:
    A sampler

Template parameter ``S``:
    Needs to be one a subclass of Sampler<T>, with T as one of the
    types of :py:class:`Field`)doc";

static const char *__doc_hl_navigation_sim_PropertySampler_PropertySampler_3 =
R"doc(Create a new sampler

:param args:
    The arguments of the constructor of ``S``

Template parameter ``S``:
    Needs to be one a subclass of Sampler<T>, with T as one of the
    types of :py:class:`Field`

Template parameter ``Targs``:
    The type of the arguments of the constructor of ``S``)doc";

static const char *__doc_hl_navigation_sim_PropertySampler_count = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_PropertySampler_done = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_PropertySampler_s = R"doc()doc";

static const char *__doc_hl_navigation_sim_PropertySampler_sampler = R"doc()doc";

static const char *__doc_hl_navigation_sim_RTWorld = R"doc()doc";

static const char *__doc_hl_navigation_sim_RTWorld_RTWorld = R"doc()doc";

static const char *__doc_hl_navigation_sim_RTWorld_add_callback = R"doc()doc";

static const char *__doc_hl_navigation_sim_RTWorld_callbacks = R"doc()doc";

static const char *__doc_hl_navigation_sim_RTWorld_rt_run = R"doc()doc";

static const char *__doc_hl_navigation_sim_RegularSampler =
R"doc(An generator that sample regularly, adding a fixed step to the
previous sample.

Only defined if T is an algebra.

If :py:attr:`wrap` is not set to :py:attr:`Wrap.terminate`, the generator is
inexhaustible, else it will be exhausted after looping once through
all values.

Template parameter ``T``:
    The sampled type)doc";

static const char *__doc_hl_navigation_sim_RegularSampler_RegularSampler =
R"doc({ function_description }

@private

:param from:
    The initial value

:param number:
    The number of samples to draw (``from`` included)

:param wrap:
    How it should wrap at the end of the interval)doc";

static const char *__doc_hl_navigation_sim_RegularSampler_done = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_RegularSampler_from = R"doc()doc";

static const char *__doc_hl_navigation_sim_RegularSampler_make_with_interval =
R"doc(Construct a sampler that will samples ``number`` points between
``from`` and ``to``.

For example, ``number=3``, samples the following points: ``from``,
``(from + to) / 2``, ``to``.

:param from:
    The first value

:param to:
    The target value to be reached after ``number`` samples.

:param number:
    The number of samples to draw.

:param wrap:
    How it should wrap at the end of the interval (i.e., after
    ``number`` samples have been drawn)

:return:
    The sampler.)doc";

static const char *__doc_hl_navigation_sim_RegularSampler_make_with_step =
R"doc(Construct a sampler that will samples points, iteratively adding
``step``.

For example, ``step=1``, samples the following points: ``from``,
``from + 1``, ``from + 2`, ...

:param from:
    The first value

:param step:
    The step

:param number:
    The number of samples to draw

:param wrap:
    How it should wrap at the end of the interval (i.e., after
    ``number`` samples have been drawn))doc";

static const char *__doc_hl_navigation_sim_RegularSampler_number = R"doc()doc";

static const char *__doc_hl_navigation_sim_RegularSampler_s = R"doc()doc";

static const char *__doc_hl_navigation_sim_RegularSampler_step = R"doc()doc";

static const char *__doc_hl_navigation_sim_RegularSampler_to = R"doc()doc";

static const char *__doc_hl_navigation_sim_RegularSampler_wrap = R"doc()doc";

static const char *__doc_hl_navigation_sim_Sampler =
R"doc(Abstract Sampler base class. that allows to sample values of type T
using :py:meth:`sample`.

Template parameter ``T``:
    The sampled type)doc";

static const char *__doc_hl_navigation_sim_SamplerFromRegister =
R"doc(An inexhaustible sampler of objects from a class that has register

It creates objects of the sub-class identified by :py:attr:`type`.

The objects properties are sampled using the properties samplers
stored in :py:attr:`properties`. Property that are not sampled, are assigned
to their default value.

The created objects are wrapped in a container, which by default is a
shared pointer.

Template parameter ``T``:
    The type of the root class)doc";

static const char *__doc_hl_navigation_sim_SamplerFromRegister_SamplerFromRegister =
R"doc(Constructs a new instance.

:param type:
    The registered type name)doc";

static const char *__doc_hl_navigation_sim_SamplerFromRegister_properties =
R"doc(A map of property samplers ``name -> sampler`` used configure the
sampled object.)doc";

static const char *__doc_hl_navigation_sim_SamplerFromRegister_reset = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_SamplerFromRegister_s = R"doc()doc";

static const char *__doc_hl_navigation_sim_SamplerFromRegister_type = R"doc(The registered name of the sub-class to be sampled)doc";

static const char *__doc_hl_navigation_sim_Sampler_Sampler = R"doc()doc";

static const char *__doc_hl_navigation_sim_Sampler_count =
R"doc(Counts the number of sampled values since reset.

:return:
    The number of sampled values)doc";

static const char *__doc_hl_navigation_sim_Sampler_done =
R"doc(Returns whether the generator is exhausted and if not reset, :py:meth:`sample` will raise an error.

:return:
    True if the generator is exhausted.)doc";

static const char *__doc_hl_navigation_sim_Sampler_index = R"doc()doc";

static const char *__doc_hl_navigation_sim_Sampler_reset =
R"doc(Resets the generator.

It also resets the samples count to 0.)doc";

static const char *__doc_hl_navigation_sim_Sampler_s = R"doc()doc";

static const char *__doc_hl_navigation_sim_Sampler_sample =
R"doc(Sample values of type T.

:raise:
    std::runtime_error If the generator is exhausted (i.e., :py:meth:`done`
    returns true)

:return:
    The sampled value.)doc";

static const char *__doc_hl_navigation_sim_Scenario =
R"doc(A scenario describes a distribution of :py:class:`World` that can be sampled
to perform an experiment.)doc";

static const char *__doc_hl_navigation_sim_Scenario_Group = R"doc(A group of agents that can be generated and added to the world.)doc";

static const char *__doc_hl_navigation_sim_Scenario_Group_add_to_world =
R"doc(Generate and add the agents to the world.

:param world:
    The world)doc";

static const char *__doc_hl_navigation_sim_Scenario_Group_reset = R"doc(Resets the agent generator.)doc";

static const char *__doc_hl_navigation_sim_Scenario_Scenario =
R"doc(Constructs a new instance.

:param inits:
    The collection of world initializers to use.)doc";

static const char *__doc_hl_navigation_sim_Scenario_add_init =
R"doc(Adds a world initializer.

:param f:
    The initializer)doc";

static const char *__doc_hl_navigation_sim_Scenario_get_initializers =
R"doc(Gets the world initializers.

:return:
    The initializers.)doc";

static const char *__doc_hl_navigation_sim_Scenario_groups = R"doc(Groups)doc";

static const char *__doc_hl_navigation_sim_Scenario_init_world =
R"doc(Initializes the world.

:param world:
    The world)doc";

static const char *__doc_hl_navigation_sim_Scenario_initializers = R"doc()doc";

static const char *__doc_hl_navigation_sim_Scenario_obstacles = R"doc(Obstacles)doc";

static const char *__doc_hl_navigation_sim_Scenario_walls = R"doc(Walls)doc";

static const char *__doc_hl_navigation_sim_SequenceSampler =
R"doc(An generator that loops through a sequence of values.

If wrap is not set to :py:attr:`Wrap.terminate`, the generator is
inexhaustible, else it will be exhausted after looping once through
all values once.

Template parameter ``T``:
    The sampled type)doc";

static const char *__doc_hl_navigation_sim_SequenceSampler_SequenceSampler =
R"doc(Construct an instance

:param values:
    The values to be sampled in sequence

:param wrap:
    How it should wrap at the end of the sequence)doc";

static const char *__doc_hl_navigation_sim_SequenceSampler_done = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_SequenceSampler_s = R"doc()doc";

static const char *__doc_hl_navigation_sim_SequenceSampler_values = R"doc()doc";

static const char *__doc_hl_navigation_sim_SequenceSampler_wrap = R"doc()doc";

static const char *__doc_hl_navigation_sim_SimpleScenario =
R"doc(Simple scenario that serves as an example.

The scenario add a single agent with a waypoints task, dummy behavior,
and holonomic kinematics.)doc";

static const char *__doc_hl_navigation_sim_SimpleScenario_SimpleScenario = R"doc()doc";

static const char *__doc_hl_navigation_sim_SimpleScenario_get_type = R"doc()doc";

static const char *__doc_hl_navigation_sim_SimpleScenario_init_world = R"doc()doc";

static const char *__doc_hl_navigation_sim_StateEstimation =
R"doc(This class describe a generic state estimation that should update the
environment state used by the agent :py:class:`Behavior`.

As the environment state is specialized by sub-classes of :py:class:`Behavior` like :py:class:`GeometricState`,
concrete sub-classes have to target one or more of them.

In particular, the agent should use a state estimation compatible with
it's state representation.

:py:class:`StateEstimation` holds a pointer to the :py:class:`World` containing the
agent, which it queries to get the relevant entities (located nearby
the agent).)doc";

static const char *__doc_hl_navigation_sim_StateEstimation_StateEstimation = R"doc(Constructs a new instance. @private)doc";

static const char *__doc_hl_navigation_sim_StateEstimation_prepare =
R"doc(Setup the state estimation. Called before starting a simulation.

:param agent:
    The agent owning the state estimation

:param world:
    The that the agent is part of)doc";

static const char *__doc_hl_navigation_sim_StateEstimation_update =
R"doc(Updates the state of a given agent :py:class:`Behavior`

:param agent:
    The agent owning the state estimation

:param world:
    The that the agent is part of)doc";

static const char *__doc_hl_navigation_sim_Task =
R"doc(This class describe the high-level task control that provides
navigation goals.)doc";

static const char *__doc_hl_navigation_sim_Task_Task = R"doc(Constructs a new instance. @private)doc";

static const char *__doc_hl_navigation_sim_Task_add_callback =
R"doc(Adds a callback called to log task events

:param value:
    The desired callback)doc";

static const char *__doc_hl_navigation_sim_Task_callbacks = R"doc()doc";

static const char *__doc_hl_navigation_sim_Task_clear_callbacks = R"doc(Remove any callbacks)doc";

static const char *__doc_hl_navigation_sim_Task_done =
R"doc(Returns whether the task is done.

:return:
    True if the task has finished.)doc";

static const char *__doc_hl_navigation_sim_Task_update =
R"doc(Tick the task, possibly updating the navigation goal of the agent.

:param agent:
    The agent ticking the task.)doc";

static const char *__doc_hl_navigation_sim_Trace =
R"doc(This class helps to collect the trajectories of the agents during an
experiment.)doc";

static const char *__doc_hl_navigation_sim_Trace_Trace = R"doc(Constructs a new instance.)doc";

static const char *__doc_hl_navigation_sim_Trace_cmd_data = R"doc()doc";

static const char *__doc_hl_navigation_sim_Trace_cmd_ptr = R"doc()doc";

static const char *__doc_hl_navigation_sim_Trace_collisions_data = R"doc()doc";

static const char *__doc_hl_navigation_sim_Trace_finalize =
R"doc(Called after the simulation has finished

:param world:
    The world

:param group:
    The dataset group where to store data

:param steps:
    The simulation steps done.)doc";

static const char *__doc_hl_navigation_sim_Trace_index_of_agent = R"doc()doc";

static const char *__doc_hl_navigation_sim_Trace_indices = R"doc()doc";

static const char *__doc_hl_navigation_sim_Trace_init =
R"doc(Init a trace recording

:param world:
    The world

:param group:
    The dataset group where to store data

:param steps:
    The maximal number of steps that will be recorded)doc";

static const char *__doc_hl_navigation_sim_Trace_number = R"doc(The number of agents recorded)doc";

static const char *__doc_hl_navigation_sim_Trace_pose_data = R"doc()doc";

static const char *__doc_hl_navigation_sim_Trace_pose_ptr = R"doc()doc";

static const char *__doc_hl_navigation_sim_Trace_record = R"doc()doc";

static const char *__doc_hl_navigation_sim_Trace_record_cmd = R"doc(Whether it should record the agents control commands)doc";

static const char *__doc_hl_navigation_sim_Trace_record_collisions = R"doc(Whether it should record collisions)doc";

static const char *__doc_hl_navigation_sim_Trace_record_pose = R"doc(Whether it should record the agents poses)doc";

static const char *__doc_hl_navigation_sim_Trace_record_safety_violation = R"doc(Whether it should record safety violations)doc";

static const char *__doc_hl_navigation_sim_Trace_record_target = R"doc(Whether it should record the agents targets)doc";

static const char *__doc_hl_navigation_sim_Trace_record_task_events = R"doc(Whether it should record data from task events)doc";

static const char *__doc_hl_navigation_sim_Trace_record_twist = R"doc(Whether it should record the agents twists)doc";

static const char *__doc_hl_navigation_sim_Trace_safety_violation_data = R"doc()doc";

static const char *__doc_hl_navigation_sim_Trace_safety_violation_ptr = R"doc()doc";

static const char *__doc_hl_navigation_sim_Trace_steps = R"doc(The number of steps recorded)doc";

static const char *__doc_hl_navigation_sim_Trace_target_data = R"doc()doc";

static const char *__doc_hl_navigation_sim_Trace_target_ptr = R"doc()doc";

static const char *__doc_hl_navigation_sim_Trace_task_events = R"doc()doc";

static const char *__doc_hl_navigation_sim_Trace_task_events_data = R"doc()doc";

static const char *__doc_hl_navigation_sim_Trace_twist_data = R"doc()doc";

static const char *__doc_hl_navigation_sim_Trace_twist_ptr = R"doc()doc";

static const char *__doc_hl_navigation_sim_Trace_update =
R"doc({ function_description }

:param world:
    The world

:param step:
    The simulation step index)doc";

static const char *__doc_hl_navigation_sim_UniformSampler =
R"doc(Sample randomly from a uniform distribution.

Only defined if T is a number.

Template parameter ``T``:
    The sampled type)doc";

static const char *__doc_hl_navigation_sim_UniformSampler_UniformSampler = R"doc()doc";

static const char *__doc_hl_navigation_sim_UniformSampler_dist = R"doc()doc";

static const char *__doc_hl_navigation_sim_UniformSampler_max = R"doc()doc";

static const char *__doc_hl_navigation_sim_UniformSampler_min = R"doc()doc";

static const char *__doc_hl_navigation_sim_UniformSampler_s = R"doc()doc";

static const char *__doc_hl_navigation_sim_Wall =
R"doc(A static wall.

Currently, only line segment are valid shapes of walls.)doc";

static const char *__doc_hl_navigation_sim_Wall_Wall =
R"doc(Constructs a new instance.

:param p1:
    The line segment start vertex

:param p2:
    The line segment end vertex)doc";

static const char *__doc_hl_navigation_sim_Wall_Wall_2 = R"doc(Constructs a new instance.)doc";

static const char *__doc_hl_navigation_sim_Wall_Wall_3 =
R"doc(Constructs a new instance.

:param ls:
    A line segment)doc";

static const char *__doc_hl_navigation_sim_Wall_line = R"doc(The line segment)doc";

static const char *__doc_hl_navigation_sim_Wall_operator_LineSegment = R"doc(LineSegment conversion operator.)doc";

static const char *__doc_hl_navigation_sim_WaypointsTask =
R"doc(This class implement a task that makes the agent reach a sequence of
waypoints, calling :py:attr:`Controller.go_to_position` for
the next waypoint after the current has been reached within a
tolerance.

The task notifies when a new waypoint is set by calling a callback.

*Properties*: waypoints (list of :py:class:`Vector2`), loop
(bool), tolerance (float))doc";

static const char *__doc_hl_navigation_sim_WaypointsTask_WaypointsTask =
R"doc(Constructs a new instance.

:param waypoints_:
    The waypoints

:param loop_:
    Whether it should start from begin after reaching the last
    waypoint

:param tolerance_:
    The goal tolerance applied to each waypoint.)doc";

static const char *__doc_hl_navigation_sim_WaypointsTask_done = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_WaypointsTask_get_loop =
R"doc(Gets whether it should start from begin after reaching the last
waypoint.

:return:
    True if it should loop.)doc";

static const char *__doc_hl_navigation_sim_WaypointsTask_get_properties = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_WaypointsTask_get_tolerance =
R"doc(Gets the goal tolerance applied to each waypoint.

:return:
    The tolerance.)doc";

static const char *__doc_hl_navigation_sim_WaypointsTask_get_type = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_WaypointsTask_get_waypoints =
R"doc(Gets the waypoints.

:return:
    The waypoints.)doc";

static const char *__doc_hl_navigation_sim_WaypointsTask_loop = R"doc()doc";

static const char *__doc_hl_navigation_sim_WaypointsTask_running = R"doc()doc";

static const char *__doc_hl_navigation_sim_WaypointsTask_set_loop =
R"doc(Sets whether it should start from begin after reaching the last
waypoint

:param value:
    The desired value)doc";

static const char *__doc_hl_navigation_sim_WaypointsTask_set_tolerance =
R"doc(Sets the goal tolerance applied to each waypoint.

:param value:
    The desired value)doc";

static const char *__doc_hl_navigation_sim_WaypointsTask_set_waypoints =
R"doc(Sets the waypoints.

:param value:
    The desired waypoints)doc";

static const char *__doc_hl_navigation_sim_WaypointsTask_tolerance = R"doc()doc";

static const char *__doc_hl_navigation_sim_WaypointsTask_update = R"doc(@private)doc";

static const char *__doc_hl_navigation_sim_WaypointsTask_waypoint = R"doc()doc";

static const char *__doc_hl_navigation_sim_WaypointsTask_waypoints = R"doc()doc";

static const char *__doc_hl_navigation_sim_World = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_2 = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_3 =
R"doc(This class describes a world.

A world implements the core part of the simulation. It holds a
collection of entities as walls, obstacles, and agents.

After setting up the world entities, users call :py:meth:`update` or :py:meth:`run` to perform one or more simulation steps, where

1. each agent updates its control

2. each agent perform actuate its control command

3. collisions are checked and resolved

4. time is advanced

World simulation uses a simple collision model that does not attempt
to be realistic but should be computationally efficient.

1. first runs a broad-phase using `libgeos STR Trees <https://libgeos.
   org/doxygen/classgeos_1_1index_1_1strtree_1_1STRtree.html>`_ where
   potential collisions are found using rectangular bounding boxes. 2.
   then it runs a narrow-phase using the simple exact geometric shape
   of the obstacles and record pairs of entities that are in
   collision. 3. finally it resolves collisions by colliding moving
   minimally entities away from each other, setting to zero the
   component of their velocities that would attract them together.)doc";

static const char *__doc_hl_navigation_sim_World_World = R"doc(Constructs a new instance.)doc";

static const char *__doc_hl_navigation_sim_World_add_agent =
R"doc(Adds an agent to the world.

:param agent:
    The agent)doc";

static const char *__doc_hl_navigation_sim_World_add_entity = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_add_obstacle =
R"doc(Adds a disc the world as a static obstacle

:param disc:
    The disc)doc";

static const char *__doc_hl_navigation_sim_World_add_obstacle_2 =
R"doc(Adds a static obstacle the world

:param obstacle:
    The obstacle)doc";

static const char *__doc_hl_navigation_sim_World_add_wall =
R"doc(Adds a line to the world as a wall

:param line:
    The line)doc";

static const char *__doc_hl_navigation_sim_World_add_wall_2 =
R"doc(Adds a wall to the world

:param wall:
    The wall)doc";

static const char *__doc_hl_navigation_sim_World_agent_envelops = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_agent_index = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_agents = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_agents_are_idle =
R"doc(Check if all agents are idle (i.e., their tasks are done and their
controller are idle).

:return:
    True if all agents are idle)doc";

static const char *__doc_hl_navigation_sim_World_collisions = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_compute_safety_violation =
R"doc(Calculates the safety violation, i.e. the maximal penetration of a
neighbor or obstacle in the safety margin of the agent.

:param agent:
    The agent

:return:
    The safety violation or 0 if no violation.)doc";

static const char *__doc_hl_navigation_sim_World_entities = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_get_agents =
R"doc(Gets all agents in this world.

:return:
    All agents.)doc";

static const char *__doc_hl_navigation_sim_World_get_agents_in_region =
R"doc(Gets all agents in a bounding box.

:param bb:
    The bounding box specified in world-fixed coordinates

:return:
    All agents that lie in a bounding box.)doc";

static const char *__doc_hl_navigation_sim_World_get_collisions =
R"doc(Gets the colliding pairs computed during the last simulation step.

:return:
    The colliding pair of entities.)doc";

static const char *__doc_hl_navigation_sim_World_get_discs =
R"doc(Gets all disc shaped static obstacles in this world.

:return:
    All obstacles.)doc";

static const char *__doc_hl_navigation_sim_World_get_entity =
R"doc(Find an entity by identifier

:param uid:
    The entity uid

:return:
    The entity or nullptr if not found.)doc";

static const char *__doc_hl_navigation_sim_World_get_lattice = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_get_line_obstacles =
R"doc(Gets all line obstacles in this world.

:return:
    All ine obstacles.)doc";

static const char *__doc_hl_navigation_sim_World_get_line_obstacles_in_region =
R"doc(Gets all walls in a bounding box.

:param bb:
    The bounding box specified in world-fixed

:return:
    All walls that lie in a bounding box)doc";

static const char *__doc_hl_navigation_sim_World_get_neighbors =
R"doc(Gets all neighbor of an agent (ghosts included)

:param agent:
    The agent

:param distance:
    The radius of the neighborhood

:return:
    All neighbor within a circle of radius ``radius`` centered around
    the agent.)doc";

static const char *__doc_hl_navigation_sim_World_get_obstacles =
R"doc(Gets all obstacles in this world.

:return:
    All obstacles.)doc";

static const char *__doc_hl_navigation_sim_World_get_static_obstacles_in_region =
R"doc(Gets all agents in a bounding box.

:param bb:
    The bounding box specified in world-fixed

:return:
    All obstacles that lie in a bounding box)doc";

static const char *__doc_hl_navigation_sim_World_get_step =
R"doc(Gets the simulation step.

:return:
    The simulation step.)doc";

static const char *__doc_hl_navigation_sim_World_get_time =
R"doc(Gets the simulation time.

:return:
    The simulation time.)doc";

static const char *__doc_hl_navigation_sim_World_get_walls =
R"doc(Gets all walls in this world.

:return:
    All walls.)doc";

static const char *__doc_hl_navigation_sim_World_ghost_envelops = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_ghost_index = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_ghosts = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_has_lattice = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_in_collision =
R"doc(Check if two entities are currently in collision

:param e1:
    The first entity

:param e2:
    The second entity

:return:
    True if they are in collision.)doc";

static const char *__doc_hl_navigation_sim_World_lattice = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_lattice_grid = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_obstacles = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_obstacles_index = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_prepare = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_ready = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_record_collision = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_resolve_collision = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_resolve_collision_2 = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_resolve_collision_3 = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_run =
R"doc(Updates the world for one or more time steps

:param steps:
    The number of steps

:param time_step:
    The duration of each time step)doc";

static const char *__doc_hl_navigation_sim_World_set_lattice = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_set_obstacles =
R"doc(Replaces all obstacles.

:param obstacles:
    The new obstacles)doc";

static const char *__doc_hl_navigation_sim_World_set_seed =
R"doc(Sets the random seed

:param seed:
    The random seed)doc";

static const char *__doc_hl_navigation_sim_World_set_walls =
R"doc(Replaces all walls.

:param walls:
    The new walls)doc";

static const char *__doc_hl_navigation_sim_World_space_agents_apart =
R"doc(Move agents so that they do not overlap anymore with themselves or
with any obstacle

:param minimal_distance:
    The minimal distance

:param with_safety_margin:
    Whether the safety margin should be added to the minimal distance

:param max_iterations:
    The maximal number of iterations to perform.)doc";

static const char *__doc_hl_navigation_sim_World_space_agents_apart_once = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_static_envelops = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_step = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_time = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_udpate_agent_collisions = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_update =
R"doc(Updates world for a single time step.

:param time_step:
    The time step)doc";

static const char *__doc_hl_navigation_sim_World_update_agent_ghosts = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_update_agents_strtree = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_update_collisions = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_update_static_strtree = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_walls = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_walls_index = R"doc()doc";

static const char *__doc_hl_navigation_sim_World_wrap_agents_on_lattice = R"doc()doc";

static const char *__doc_hl_navigation_sim_Wrap = R"doc(What should a generator do at the end of a sequence)doc";

static const char *__doc_hl_navigation_sim_Wrap_loop = R"doc(Start from scratch)doc";

static const char *__doc_hl_navigation_sim_Wrap_repeat = R"doc(Repeat the last entry)doc";

static const char *__doc_hl_navigation_sim_Wrap_terminate = R"doc(Terminate)doc";

static const char *__doc_hl_navigation_sim_get = R"doc()doc";

static const char *__doc_hl_navigation_sim_get_ptr = R"doc()doc";

static const char *__doc_hl_navigation_sim_get_random_seed =
R"doc(Gets the random seed.

:return:
    The random seed.)doc";

static const char *__doc_hl_navigation_sim_is_one_of = R"doc()doc";

static const char *__doc_hl_navigation_sim_penetration_inside_disc = R"doc()doc";

static const char *__doc_hl_navigation_sim_penetration_inside_line = R"doc()doc";

static const char *__doc_hl_navigation_sim_penetration_vector_inside_line =
R"doc(Computes the com-penetration of a disc and a line segment

:param line:
    The line

:param center:
    The disc center

:param radius:
    The disc radius

:return:
    The penetration vector, i.e. the shortest shift that would remove
    overlapping, or in case the is not overlap.)doc";

static const char *__doc_hl_navigation_sim_random_generator =
R"doc(The random generator shared by all samplers

:return:
    The random generator)doc";

static const char *__doc_hl_navigation_sim_set_random_seed =
R"doc(Sets the random seed.

:param seed:
    The seed)doc";

static const char *__doc_hl_navigation_sim_wrap_done = R"doc()doc";

static const char *__doc_hl_navigation_sim_wrap_from_string = R"doc()doc";

static const char *__doc_hl_navigation_sim_wrap_index = R"doc()doc";

static const char *__doc_hl_navigation_sim_wrap_to_string = R"doc()doc";

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif


static const char *__doc_hl_navigation_sim_Agent_property_behavior =
R"doc(The navigation behavior.
)doc";

static const char *__doc_hl_navigation_sim_Agent_property_controller =
R"doc(The navigation controller.
)doc";

static const char *__doc_hl_navigation_sim_Agent_property_geometric_state =
R"doc(The geometric state if any, i.e., if the behavior is a subclass
of :py:class:`geometricstate`

@private
)doc";

static const char *__doc_hl_navigation_sim_Agent_property_kinematics =
R"doc(The kinematics.
)doc";

static const char *__doc_hl_navigation_sim_Agent_property_state_estimation =
R"doc(The state estimation.
)doc";

static const char *__doc_hl_navigation_sim_Agent_property_task =
R"doc(The task.
)doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_property_orientation_noise =
R"doc(The orientation.
)doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_property_position_noise =
R"doc(The position_noise.
)doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_property_properties =
R"doc(@privat)doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_property_radius =
R"doc(The circle radius.
)doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_property_shuffle =
R"doc(Whether it should shuffle the agents before initializing them.
)doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_property_tolerance =
R"doc(The goal tolerance.
)doc";

static const char *__doc_hl_navigation_sim_AntipodalScenario_property_type =
R"doc(@privat)doc";

static const char *__doc_hl_navigation_sim_BoundedStateEstimation_property_properties =
R"doc(@privat)doc";

static const char *__doc_hl_navigation_sim_BoundedStateEstimation_property_range_of_view =
R"doc(The range of view.
)doc";

static const char *__doc_hl_navigation_sim_BoundedStateEstimation_property_type =
R"doc(@privat)doc";

static const char *__doc_hl_navigation_sim_CollisionsScenario_property_properties =
R"doc()doc";

static const char *__doc_hl_navigation_sim_CollisionsScenario_property_type =
R"doc()doc";

static const char *__doc_hl_navigation_sim_CorridorScenario_property_add_safety_to_agent_margin =
R"doc()doc";

static const char *__doc_hl_navigation_sim_CorridorScenario_property_agent_margin =
R"doc()doc";

static const char *__doc_hl_navigation_sim_CorridorScenario_property_length =
R"doc()doc";

static const char *__doc_hl_navigation_sim_CorridorScenario_property_properties =
R"doc(@privat)doc";

static const char *__doc_hl_navigation_sim_CorridorScenario_property_type =
R"doc(@privat)doc";

static const char *__doc_hl_navigation_sim_CorridorScenario_property_width =
R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_property_add_safety_to_agent_margin =
R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_property_agent_margin =
R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_property_properties =
R"doc(@privat)doc";

static const char *__doc_hl_navigation_sim_CrossScenario_property_properties_2 =
R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_property_side =
R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_property_target_margin =
R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_property_tolerance =
R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossScenario_property_type =
R"doc(@privat)doc";

static const char *__doc_hl_navigation_sim_CrossScenario_property_type_2 =
R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossTorusScenario_property_add_safety_to_agent_margin =
R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossTorusScenario_property_agent_margin =
R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossTorusScenario_property_properties =
R"doc(@privat)doc";

static const char *__doc_hl_navigation_sim_CrossTorusScenario_property_side =
R"doc()doc";

static const char *__doc_hl_navigation_sim_CrossTorusScenario_property_type =
R"doc(@privat)doc";

static const char *__doc_hl_navigation_sim_Experiment_property_path =
R"doc(The database path.
)doc";

static const char *__doc_hl_navigation_sim_Experiment_property_world =
R"doc()doc";

static const char *__doc_hl_navigation_sim_Scenario_property_initializers =
R"doc(The world initializers.
)doc";

static const char *__doc_hl_navigation_sim_SimpleScenario_property_type =
R"doc()doc";

static const char *__doc_hl_navigation_sim_WaypointsTask_property_loop =
R"doc(Whether it should start from begin after reaching the last
waypoint.
)doc";

static const char *__doc_hl_navigation_sim_WaypointsTask_property_properties =
R"doc(@privat)doc";

static const char *__doc_hl_navigation_sim_WaypointsTask_property_tolerance =
R"doc(The goal tolerance applied to each waypoint.
)doc";

static const char *__doc_hl_navigation_sim_WaypointsTask_property_type =
R"doc(@privat)doc";

static const char *__doc_hl_navigation_sim_WaypointsTask_property_waypoints =
R"doc(The waypoints.
)doc";

static const char *__doc_hl_navigation_sim_World_property_agents =
R"doc(All agents in this world.
)doc";

static const char *__doc_hl_navigation_sim_World_property_agents_in_region =
R"doc(All agents in a bounding box.

:param bb:
    the bounding box specified in world-fixed coordinates
)doc";

static const char *__doc_hl_navigation_sim_World_property_collisions =
R"doc(The colliding pairs computed during the last simulation step.
)doc";

static const char *__doc_hl_navigation_sim_World_property_discs =
R"doc(All disc shaped static obstacles in this world.
)doc";

static const char *__doc_hl_navigation_sim_World_property_entity =
R"doc(Find an entity by identifier

:param uid:
    The entity uid
)doc";

static const char *__doc_hl_navigation_sim_World_property_lattice =
R"doc()doc";

static const char *__doc_hl_navigation_sim_World_property_line_obstacles =
R"doc(All line obstacles in this world.
)doc";

static const char *__doc_hl_navigation_sim_World_property_line_obstacles_in_region =
R"doc(All walls in a bounding box.

:param bb:
    the bounding box specified in world-fixed
)doc";

static const char *__doc_hl_navigation_sim_World_property_neighbors =
R"doc(All neighbor of an agent (ghosts included)

:param agent:
    the agent

:param distance:
    the radius of the neighborhood
)doc";

static const char *__doc_hl_navigation_sim_World_property_obstacles =
R"doc(All obstacles in this world.
)doc";

static const char *__doc_hl_navigation_sim_World_property_static_obstacles_in_region =
R"doc(All agents in a bounding box.

:param bb:
    the bounding box specified in world-fixed
)doc";

static const char *__doc_hl_navigation_sim_World_property_step =
R"doc(The simulation step.
)doc";

static const char *__doc_hl_navigation_sim_World_property_time =
R"doc(The simulation time.
)doc";

static const char *__doc_hl_navigation_sim_World_property_walls =
R"doc(All walls in this world.
)doc";

static const char *__doc_hl_navigation_sim_property_ptr =
R"doc()doc";

static const char *__doc_hl_navigation_sim_property_random_seed =
R"doc(The random seed.
)doc";
