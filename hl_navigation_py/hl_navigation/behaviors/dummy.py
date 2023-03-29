from typing import Optional

import numpy as np

from hl_navigation import Behavior, Kinematic, register_property


class PyDummyBehavior(Behavior, name="PyDummy"):

    """
    Dummy Behavior that ignore obstacles instead of avoiding them.
    Equivalent to the C++ class hl_navigation::DummyBehavior.
    Implemented to demonstrate that sub-classing Behavior works in Python
    """

    # Not needed ... defined to have a more complete template
    def __init__(self, kinematic: Optional[Kinematic] = None, radius: float = 0.0):
        Behavior.__init__(self, kinematic, radius)
        self._tired = False

    @register_property(True, "Am I dummy?")
    def dummy(self) -> bool:
        return True;

    @register_property(False, "Am I tired?")
    def tired(self) -> bool:
        return self._tired;

    @tired.setter
    def tired(self, value: bool) -> None:
        self._tired = value;

    def compute_desired_velocity(self, time_step: float) -> np.ndarray:
        delta = self.target_pose.position - self.pose.position
        return self.optimal_speed * delta / np.linalg.norm(delta)
