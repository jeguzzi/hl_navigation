.. _sim cpp yaml:
====
YAML
====

Similar as for `the core library <core cpp yaml>`_, we provide import/export 
from/to YAML through specializations of :py:class:`struct YAML::convert<T>` in ``hl_navigation_sim/yaml``.

In particular, we specialize the conversion from/to the following additional classes:

- :cpp:class:`hl_navigation::sim::Task`
- :cpp:class:`hl_navigation::sim::StateEstimation`
- :cpp:class:`hl_navigation::sim::Agent`
- :cpp:class:`hl_navigation::sim::Obstacle`
- :cpp:class:`hl_navigation::sim::Wall`
- :cpp:class:`hl_navigation::sim::World`
- :cpp:struct:`hl_navigation::sim::Sampler`
- :cpp:class:`hl_navigation::sim::BehaviorSampler`
- :cpp:class:`hl_navigation::sim::KinematicsSampler`
- :cpp:type:`hl_navigation::sim::StateEstimationSampler`
- :cpp:type:`hl_navigation::sim::TaskSampler`
- :cpp:class:`hl_navigation::sim::AgentSampler`
- :cpp:class:`hl_navigation::sim::Scenario`
- :cpp:class:`hl_navigation::sim::Experiment`


