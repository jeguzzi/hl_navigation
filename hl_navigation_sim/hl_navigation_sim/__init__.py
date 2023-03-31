import pkg_resources
from hl_navigation import load_py_plugins as _load_py_plugins
from hl_navigation import register_property

from ._hl_navigation_sim import (Agent, BoundedStateEstimation, BoundingBox,
                                 Entity, Experiment, Wall, Obstacle)
from ._hl_navigation_sim import Scenario as _Scenario
from ._hl_navigation_sim import StateEstimation as _StateEstimation
from ._hl_navigation_sim import Task as _Task
from ._hl_navigation_sim import (WayPointsTask, World, dump, load_agent,
                                 load_experiment, load_scenario,
                                 load_state_estimation, load_task, load_world)


class Scenario(_Scenario):

    def __init_subclass__(cls, /, **kwargs):
        name = kwargs.pop('name', '')
        super().__init_subclass__(**kwargs)
        if name:
            _Scenario.register_type(name, cls)
            cls._type = name
            for k, v in vars(cls).items():
                if isinstance(v, property) and hasattr(v.fget,
                                                       "__default_value__"):
                    return_type = v.fget.__annotations__['return']
                    default_value = return_type(v.fget.__default_value__)
                    desc = v.fget.__desc__
                    _Scenario.add_property(name, k, v, default_value, desc)

    def __init__(self):
        _Scenario.__init__(self)


class StateEstimation(_StateEstimation):

    def __init_subclass__(cls, /, **kwargs):
        name = kwargs.pop('name', '')
        super().__init_subclass__(**kwargs)
        if name:
            _StateEstimation.register_type(name, cls)
            cls._type = name
            for k, v in vars(cls).items():
                if isinstance(v, property) and hasattr(v.fget,
                                                       "__default_value__"):
                    return_type = v.fget.__annotations__['return']
                    default_value = return_type(v.fget.__default_value__)
                    desc = v.fget.__desc__
                    _StateEstimation.add_property(name, k, v, default_value,
                                                  desc)

    def __init__(self, *args, **kwargs):
        _StateEstimation.__init__(self, *args, **kwargs)


class Task(_Task):

    def __init_subclass__(cls, /, **kwargs):
        name = kwargs.pop('name', '')
        super().__init_subclass__(**kwargs)
        if name:
            _Task.register_type(name, cls)
            cls._type = name
            for k, v in vars(cls).items():
                if isinstance(v, property) and hasattr(v.fget,
                                                       "__default_value__"):
                    return_type = v.fget.__annotations__['return']
                    default_value = return_type(v.fget.__default_value__)
                    desc = v.fget.__desc__
                    _Task.add_property(name, k, v, v, default_value, desc)

    def __init__(self):
        _Task.__init__(self)


def load_py_plugins():
    _load_py_plugins()
    for name in ('hl_tasks', 'hl_state_estimations', 'hl_scenarios'):
        for entry_point in pkg_resources.iter_entry_points(name):
            entry_point.load()


__all__ = [
    'Entity', 'Obstacle', 'Wall', 'World', 'Agent', 'BoundedStateEstimation', 'WayPointsTask',
    'Experiment', 'Scenario', 'StateEstimation', 'Task', 'BoundingBox', 'dump'
    'load_agent', 'load_state_estimation', 'load_task', 'load_world',
    'load_scenario', 'load_experiment', 'load_py_plugins'
]
