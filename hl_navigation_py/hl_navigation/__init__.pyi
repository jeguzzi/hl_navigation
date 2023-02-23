from __future__ import annotations
import typing

import numpy

__all__ = [
    "Action",
    "ActionState",
    "Behavior",
    "BehaviorHeading",
    "BehaviorMode",
    "Controller",
    "Disc",
    "DummyBehavior",
    "Forward",
    "FourWheeled",
    "HLBehavior",
    "HRVOBehavior",
    "Holonomic",
    "Kinematic",
    "LineSegment",
    "ORCABehavior",
    "Pose2",
    "Twist2",
    "TwoWheeled",
    "Wheeled"
]

def behavior_with_name(arg0: str, arg1: Kinematic, arg2: float) -> Behavior:
    pass

_Shape = typing.Tuple[int, ...]

Vector2 = typing.Union[numpy.ndarray[numpy.float32, _Shape[2, 1]], typing.Tuple[float, float]]

class Action():
    def abort(self) -> None: ...
    @property
    def done(self) -> bool:
        """
        :type: bool
        """
    @property
    def done_cb(self) -> typing.Optional[typing.Callable[[ActionState], None]]:
        """
        :type: typing.Optional[typing.Callable[[ActionState], None]]
        """
    @done_cb.setter
    def done_cb(self, arg0: typing.Optional[typing.Callable[[ActionState], None]]) -> None:
        pass
    @property
    def running(self) -> bool:
        """
        :type: bool
        """
    @property
    def running_cb(self) -> typing.Optional[typing.Callable[[float], None]]:
        """
        :type: typing.Optional[typing.Callable[[float], None]]
        """
    @running_cb.setter
    def running_cb(self, arg0: typing.Optional[typing.Callable[[float], None]]) -> None:
        pass
    @property
    def state(self) -> ActionState:
        """
        :type: ActionState
        """
    pass
class ActionState():
    """
    Members:

      idle

      running

      failure

      success
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    __members__: dict # value = {'idle': <ActionState.idle: 0>, 'running': <ActionState.running: 1>, 'failure': <ActionState.failure: 2>, 'success': <ActionState.success: 3>}
    failure: ActionState # value = <ActionState.failure: 2>
    idle: ActionState # value = <ActionState.idle: 0>
    running: ActionState # value = <ActionState.running: 1>
    success: ActionState # value = <ActionState.success: 3>
    pass
class Behavior():
    @typing.overload
    def actuate(self, arg0: Twist2, arg1: float) -> None: ...
    @typing.overload
    def actuate(self, arg0: float) -> None: ...
    def cmd_twist(self, time_step: float, relative: bool, mode: BehaviorMode = BehaviorMode.move, set_as_actuated: bool = True) -> Twist2: ...
    def get_actuated_twist(self, relative: bool = False) -> Twist2: ...
    def get_twist(self, relative: bool = False) -> Twist2: ...
    def get_velocity(self, relative: bool = False) -> Vector2: ...
    def set_velocity(self, velocity: Vector2, relative: bool = False) -> None: ...
    def to_frame(self, arg0: Twist2, arg1: bool) -> Twist2: ...
    def twist_from_wheel_speeds(self, arg0: typing.List[float]) -> Twist2: ...
    def wheel_speeds_from_twist(self, arg0: Twist2) -> typing.List[float]: ...
    @property
    def actuated_twist(self) -> Twist2:
        """
        :type: Twist2
        """
    @actuated_twist.setter
    def actuated_twist(self, arg1: Twist2) -> None:
        pass
    @property
    def actuated_wheel_speeds(self) -> typing.List[float]:
        """
        :type: typing.List[float]
        """
    @property
    def angular_speed(self) -> float:
        """
        :type: float
        """
    @angular_speed.setter
    def angular_speed(self, arg1: float) -> None:
        pass
    @property
    def heading_behavior(self) -> BehaviorHeading:
        """
        :type: BehaviorHeading
        """
    @heading_behavior.setter
    def heading_behavior(self, arg1: BehaviorHeading) -> None:
        pass
    @property
    def horizon(self) -> float:
        """
        :type: float
        """
    @horizon.setter
    def horizon(self, arg1: float) -> None:
        pass
    @property
    def kinematic(self) -> Kinematic:
        """
        :type: Kinematic
        """
    @property
    def line_obstacles(self) -> typing.List[LineSegment]:
        """
        :type: typing.List[LineSegment]
        """
    @line_obstacles.setter
    def line_obstacles(self, arg1: typing.List[LineSegment]) -> None:
        pass
    @property
    def max_angular_speed(self) -> float:
        """
        :type: float
        """
    @max_angular_speed.setter
    def max_angular_speed(self, arg1: float) -> None:
        pass
    @property
    def max_speed(self) -> float:
        """
        :type: float
        """
    @max_speed.setter
    def max_speed(self, arg1: float) -> None:
        pass
    @property
    def neighbors(self) -> typing.List[Disc]:
        """
        :type: typing.List[Disc]
        """
    @neighbors.setter
    def neighbors(self, arg1: typing.List[Disc]) -> None:
        pass
    @property
    def optimal_angular_speed(self) -> float:
        """
        :type: float
        """
    @optimal_angular_speed.setter
    def optimal_angular_speed(self, arg1: float) -> None:
        pass
    @property
    def optimal_speed(self) -> float:
        """
        :type: float
        """
    @optimal_speed.setter
    def optimal_speed(self, arg1: float) -> None:
        pass
    @property
    def orientation(self) -> float:
        """
        :type: float
        """
    @orientation.setter
    def orientation(self, arg1: float) -> None:
        pass
    @property
    def pose(self) -> Pose2:
        """
        :type: Pose2
        """
    @pose.setter
    def pose(self, arg1: Pose2) -> None:
        pass
    @property
    def position(self) -> Vector2:
        """
        :type: Vector2
        """
    @position.setter
    def position(self, arg1: Vector2) -> None:
        pass
    @property
    def radius(self) -> float:
        """
        :type: float
        """
    @radius.setter
    def radius(self, arg1: float) -> None:
        pass
    @property
    def rotation_tau(self) -> float:
        """
        :type: float
        """
    @rotation_tau.setter
    def rotation_tau(self, arg1: float) -> None:
        pass
    @property
    def safety_margin(self) -> float:
        """
        :type: float
        """
    @safety_margin.setter
    def safety_margin(self, arg1: float) -> None:
        pass
    @property
    def static_obstacles(self) -> typing.List[Disc]:
        """
        :type: typing.List[Disc]
        """
    @static_obstacles.setter
    def static_obstacles(self, arg1: typing.List[Disc]) -> None:
        pass
    @property
    def target_angular_speed(self) -> float:
        """
        :type: float
        """
    @target_angular_speed.setter
    def target_angular_speed(self, arg1: float) -> None:
        pass
    @property
    def target_orientation(self) -> float:
        """
        :type: float
        """
    @target_orientation.setter
    def target_orientation(self, arg1: float) -> None:
        pass
    @property
    def target_position(self) -> Vector2:
        """
        :type: Vector2
        """
    @target_position.setter
    def target_position(self, arg1: Vector2) -> None:
        pass
    @property
    def target_velocity(self) -> Vector2:
        """
        :type: Vector2
        """
    @target_velocity.setter
    def target_velocity(self, arg1: Vector2) -> None:
        pass
    @property
    def twist(self) -> Twist2:
        """
        :type: Twist2
        """
    @twist.setter
    def twist(self, arg1: Twist2) -> None:
        pass
    @property
    def velocity(self) -> Vector2:
        """
        :type: Vector2
        """
    @velocity.setter
    def velocity(self, arg1: Vector2) -> None:
        pass
    @property
    def wheel_speeds(self) -> typing.List[float]:
        """
        :type: typing.List[float]
        """
    @wheel_speeds.setter
    def wheel_speeds(self, arg1: typing.List[float]) -> None:
        pass
    pass
class BehaviorHeading():
    """
    Members:

      idle

      target_point

      target_angle

      velocity
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    __members__: dict # value = {'idle': <BehaviorHeading.idle: 0>, 'target_point': <BehaviorHeading.target_point: 1>, 'target_angle': <BehaviorHeading.target_angle: 2>, 'velocity': <BehaviorHeading.velocity: 4>}
    idle: BehaviorHeading # value = <BehaviorHeading.idle: 0>
    target_angle: BehaviorHeading # value = <BehaviorHeading.target_angle: 2>
    target_point: BehaviorHeading # value = <BehaviorHeading.target_point: 1>
    velocity: BehaviorHeading # value = <BehaviorHeading.velocity: 4>
    pass
class BehaviorMode():
    """
    Members:

      move

      turn

      stop

      follow
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    __members__: dict # value = {'move': <BehaviorMode.move: 0>, 'turn': <BehaviorMode.turn: 2>, 'stop': <BehaviorMode.stop: 3>, 'follow': <BehaviorMode.follow: 1>}
    follow: BehaviorMode # value = <BehaviorMode.follow: 1>
    move: BehaviorMode # value = <BehaviorMode.move: 0>
    stop: BehaviorMode # value = <BehaviorMode.stop: 3>
    turn: BehaviorMode # value = <BehaviorMode.turn: 2>
    pass
class Controller():
    def __init__(self, behavior: Behavior = None, compute_relative_twist: float = True, set_twist_as_automatically_actuated: float = True) -> None: ...
    def follow_point(self, arg0: Vector2) -> Action: ...
    def follow_pose(self, arg0: Pose2) -> Action: ...
    def follow_twist(self, arg0: Twist2) -> Action: ...
    def follow_velocity(self, arg0: Vector2) -> Action: ...
    def go_to_pose(self, arg0: Pose2, arg1: float, arg2: float) -> Action: ...
    def go_to_position(self, arg0: Vector2, arg1: float) -> Action: ...
    def set_cmd_cb(self, arg0: typing.Callable[[Twist2], None]) -> None: ...
    def update(self, arg0: float) -> Twist2: ...
    @property
    def behavior(self) -> Behavior:
        """
        :type: Behavior
        """
    @behavior.setter
    def behavior(self, arg1: Behavior) -> None:
        pass
    @property
    def idle(self) -> bool:
        """
        :type: bool
        """
    @property
    def speed_tolerance(self) -> float:
        """
        :type: float
        """
    @speed_tolerance.setter
    def speed_tolerance(self, arg1: float) -> None:
        pass
    @property
    def state(self) -> ActionState:
        """
        :type: ActionState
        """
    pass
class Disc():
    def __init__(self, position: Vector2, radius: float, social_margin: float = 0.0, velocity: Vector2 = array([0., 0.], dtype=float32)) -> None: ...
    @property
    def position(self) -> Vector2:
        """
        :type: Vector2
        """
    @property
    def radius(self) -> float:
        """
        :type: float
        """
    @property
    def social_margin(self) -> float:
        """
        :type: float
        """
    @property
    def velocity(self) -> Vector2:
        """
        :type: Vector2
        """
    pass
class DummyBehavior(Behavior):
    def __init__(self, kinematic: Kinematic, radius: float) -> None: ...
    pass
class Kinematic():
    def feasible(self, arg0: Twist2) -> Twist2: ...
    @property
    def dof(self) -> int:
        """
        :type: int
        """
    @property
    def is_wheeled(self) -> bool:
        """
        :type: bool
        """
    @property
    def max_angular_speed(self) -> float:
        """
        :type: float
        """
    @max_angular_speed.setter
    def max_angular_speed(self, arg1: float) -> None:
        pass
    @property
    def max_speed(self) -> float:
        """
        :type: float
        """
    @max_speed.setter
    def max_speed(self, arg1: float) -> None:
        pass
    pass
class Wheeled(Kinematic):
    def twist(self, arg0: typing.List[float]) -> Twist2: ...
    def wheel_speeds(self, arg0: typing.List[float]) -> Twist2: ...
    @property
    def axis(self) -> float:
        """
        :type: float
        """
    pass
class HLBehavior(Behavior):
    def __init__(self, kinematic: Kinematic, radius: float) -> None: ...
    @property
    def angular_resolution(self) -> float:
        """
        :type: float
        """
    @property
    def aperture(self) -> float:
        """
        :type: float
        """
    @aperture.setter
    def aperture(self, arg1: float) -> None:
        pass
    @property
    def collision_distances(self) -> typing.List[typing.Tuple[float, float]]:
        """
        :type: typing.List[float]
        """
    @property
    def eta(self) -> float:
        """
        :type: float
        """
    @eta.setter
    def eta(self, arg1: float) -> None:
        pass
    @property
    def resolution(self) -> int:
        """
        :type: int
        """
    @resolution.setter
    def resolution(self, arg1: int) -> None:
        pass
    @property
    def tau(self) -> float:
        """
        :type: float
        """
    @tau.setter
    def tau(self, arg1: float) -> None:
        pass
    pass
class HRVOBehavior(Behavior):
    def __init__(self, kinematic: Kinematic, radius: float) -> None: ...
    pass
class Holonomic(Kinematic):
    def __init__(self, max_speed: float, max_angular_speed: float) -> None: ...
    pass
class Forward(Kinematic):
    def __init__(self, max_speed: float, max_angular_speed: float) -> None: ...
    pass
class LineSegment():
    def __init__(self, p1: Vector2, p2: Vector2) -> None: ...
    @property
    def e1(self) -> Vector2:
        """
        :type: Vector2
        """
    @property
    def e2(self) -> Vector2:
        """
        :type: Vector2
        """
    @property
    def length(self) -> float:
        """
        :type: float
        """
    @property
    def p1(self) -> Vector2:
        """
        :type: Vector2
        """
    @property
    def p2(self) -> Vector2:
        """
        :type: Vector2
        """
    pass
class ORCABehavior(Behavior):
    def __init__(self, kinematic: Kinematic, radius: float) -> None: ...
    @property
    def is_using_effective_center(self) -> bool:
        """
        :type: bool
        """
    @is_using_effective_center.setter
    def is_using_effective_center(self, arg1: bool) -> None:
        pass
    @property
    def time_horizon(self) -> float:
        """
        :type: float
        """
    @time_horizon.setter
    def time_horizon(self, arg1: float) -> None:
        pass
    pass
class Pose2():
    def __init__(self, position: Vector2, orientation: float = 0.0) -> None: ...
    def __repr__(self) -> str: ...
    def integrate(self, arg0: Twist2, arg1: float) -> Pose2: ...
    def rotate(self, arg0: float) -> Pose2: ...
    @property
    def orientation(self) -> float:
        """
        :type: float
        """
    @orientation.setter
    def orientation(self, arg0: float) -> None:
        pass
    @property
    def position(self) -> Vector2:
        """
        :type: Vector2
        """
    @position.setter
    def position(self, arg0: Vector2) -> None:
        pass
    pass
class Twist2():
    def __init__(self, velocity: Vector2, angular_speed: float = 0.0, relative: bool = False) -> None: ...
    def __repr__(self) -> str: ...
    def rotate(self, arg0: float) -> Twist2: ...
    @property
    def angular_speed(self) -> float:
        """
        :type: float
        """
    @angular_speed.setter
    def angular_speed(self, arg0: float) -> None:
        pass
    @property
    def relative(self) -> bool:
        """
        :type: bool
        """
    @relative.setter
    def relative(self, arg0: bool) -> None:
        pass
    @property
    def velocity(self) -> Vector2:
        """
        :type: Vector2
        """
    @velocity.setter
    def velocity(self, arg0: Vector2) -> None:
        pass
    pass
class TwoWheeled(Wheeled, Kinematic):
    def __init__(self, max_speed: float, axis: float) -> None: ...
    pass
class FourWheeled(Wheeled, Kinematic):
    def __init__(self, max_speed: float, axis: float) -> None: ...
    pass
def behavior_names() -> typing.List[str]:
    pass
def behavior_with_name(arg0: str, arg1: Kinematic, arg2: float) -> Behavior:
    pass