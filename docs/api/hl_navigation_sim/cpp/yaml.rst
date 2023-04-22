.. _sim cpp yaml:
====
YAML
====

Similar as for `the core library <core cpp yaml>`_, we provide import/export 
from/to YAML through specializations of :py:class:`struct YAML::convert<T>` in ``hl_navigation_sim/yaml``.

In particular, we specialize the conversion from/to the following additional classes:

- :cpp:class:`hl_navigation_sim::Task`
- :cpp:class:`hl_navigation_sim::StateEstimation`
- :cpp:class:`hl_navigation_sim::Agent`
- :cpp:class:`hl_navigation_sim::Obstacle`
- :cpp:class:`hl_navigation_sim::Wall`
- :cpp:class:`hl_navigation_sim::World`
- :cpp:struct:`hl_navigation_sim::Sampler`
- :cpp:class:`hl_navigation_sim::BehaviorSampler`
- :cpp:class:`hl_navigation_sim::KinematicsSampler`
- :cpp:type:`hl_navigation_sim::StateEstimationSampler`
- :cpp:type:`hl_navigation_sim::TaskSampler`
- :cpp:class:`hl_navigation_sim::AgentSampler`
- :cpp:class:`hl_navigation_sim::Scenario`
- :cpp:class:`hl_navigation_sim::Experiment`


