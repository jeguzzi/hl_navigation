from typing import Any
import pkg_resources

from ._hl_navigation import Action, ActionState
from ._hl_navigation import Behavior as _Behavior
from ._hl_navigation import (BehaviorHeading, BehaviorMode,
                             CachedCollisionComputation, CollisionComputation,
                             Controller, Disc, GeometricState)
from ._hl_navigation import Kinematic as _Kinematic
from ._hl_navigation import (SocialMargin, LineSegment, Neighbor, Pose2, Twist2, dump,
                             load_behavior, load_kinematic, load_plugins)


def register_property(value: Any, description: str = ""):

    def g(f):
        f.__default_value__ = value
        f.__desc__ = description
        return property(f)

    return g


class Behavior(_Behavior):

    def __init_subclass__(cls, /, **kwargs):
        name = kwargs.pop('name', '')
        super().__init_subclass__(**kwargs)
        if name:
            _Behavior.register_type(name, cls)
            cls._type = name
            for k, v in vars(cls).items():
                if isinstance(v, property) and hasattr(v.fget,
                                                       "__default_value__"):
                    return_type = v.fget.__annotations__['return']
                    default_value = return_type(v.fget.__default_value__)
                    desc = v.fget.__desc__
                    _Behavior.add_property(name, k, v, default_value, desc)

    def __init__(self, kinematic=None, radius=0.0):
        _Behavior.__init__(self, kinematic, radius)


class Kinematic(_Kinematic):

    def __init_subclass__(cls, /, **kwargs):
        name = kwargs.pop('name', '')
        super().__init_subclass__(**kwargs)
        if name:
            _Kinematic.register_type(name, cls)
            cls._type = name
            for k, v in vars(cls).items():
                if isinstance(v, property) and hasattr(v.fget,
                                                       "__default_value__"):
                    return_type = v.fget.__annotations__['return']
                    default_value = return_type(v.fget.__default_value__)
                    desc = v.fget.__desc__
                    _Kinematic.add_property(name, k, v, default_value, desc)

    def __init__(self, max_speed=0.0, max_angular_speeed=0.0):
        _Kinematic.__init__(self, max_speed, max_angular_speeed)


from . import behaviors
from . import kinematics


def load_py_plugins():
    for name in ('hl_behaviors', 'hl_kinematics'):
        for entry_point in pkg_resources.iter_entry_points(name):
            entry_point.load()


__all__ = [
    'Behavior', 'BehaviorHeading', 'BehaviorMode', 'Pose2', 'Twist2', 'Disc',
    'Neighbor', 'LineSegment', 'Kinematic', 'Action', 'ActionState',
    'Controller', 'CollisionComputation'
    'CachedCollisionComputation'
    'GeometricState', 'dump', 'load_behavior', 'load_kinematic',
    'load_plugins', 'load_py_plugins'
]
