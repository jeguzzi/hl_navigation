"""
    Helbing, Dirk, and Peter Molnar. 
    "Social force model for pedestrian dynamics." 
    Physical review E 51.5 (1995): 4282.

    Missing (not specified in the paper):
    - attractive forces
    - fluctuation
    - cit. "potentials with a hard core that would be more realistic"
    - gradient in case of divergence
"""


from typing import Tuple, Callable, Optional

import numpy as np

from hl_navigation import (
    Behavior,
    GeometricState,
    Kinematic,
    Neighbor,
    Disc,
    LineSegment,
)
# from hl_navigation._hl_navigation import Behavior, GeometricState


# Jerome: what about the radius and safety margin?

Vector2 = np.ndarray


# value and gradient
# in the paper, called b
def neighbor_distance(
    position: Vector2, neighbor: Neighbor, step_duration: float
) -> Tuple[float, Vector2]:
    delta = position - neighbor.position
    step = neighbor.velocity * step_duration
    n1 = np.linalg.norm(delta, axis=-1)
    n2 = np.linalg.norm(delta - step, axis=-1)
    n3 = np.linalg.norm(step)
    # Can this be negative?
    b = 0.5 * np.sqrt((n1 + n2) ** 2 - n3**2)
    if b:
        grad = (
            0.5
            / b
            * (
                2 * delta
                + 2 * (delta - step)
                + 0.5 * n1 / n2 * (delta - step)
                + 0.5 * n2 / n1 * delta
            )
        )
    else:
        # TODO
        grad = np.zeros(2)
    return b, grad


def disc_distance(position: Vector2, disc: Disc) -> Tuple[float, Vector2]:
    delta = position - disc.position
    distance = np.linalg.norm(delta)
    if distance:
        grad = delta / distance
    else:
        grad = np.zeros(2)
    return distance, grad


def segment_distance(position: Vector2, line: LineSegment) -> Tuple[float, Vector2]:
    delta = position - line.p1
    x = delta.dot(line.e1)
    if x < 0:
        distance = np.linalg.norm(delta)
        grad = delta / distance
    elif x > line.length:
        delta = position - line.p2
        distance = np.linalg.norm(delta)
        grad = delta / distance
    else:
        y = delta.dot(line.e2)
        distance = abs(delta.dot(line.e2))
        grad = line.e2 if y > 0 else -line.e2
    return distance, grad


class Potential:
    """A monotonic decreasing function of the distance"""

    def __call__(self, x: float) -> Tuple[float, Vector2]:
        "Return the value and gradient of the potential"
        ...


class ExponentialPotential(Potential):
    """
    V(x) = a exp(-x/r)
    """

    def __init__(self, a: float, r: float):
        self.a = a
        self.r = r

    def __call__(self, x: float) -> Tuple[float, Vector2]:
        v = self.a * np.exp(-x / self.r)
        return v, -v / self.r


# TODO cit "Potentials with a hard core would be more realistic"


class SocialForceBehavior(Behavior, GeometricState, name="SocialForce"):
    def __init__(
        self,
        kinematic: Optional[Kinematic] = None,
        radius: float = 0.0,
        tau: float = 0.5,
        step_duration: float = 1.0,
        phi: float = 1.75,
        c: float = 0.5,
        v: Potential = ExponentialPotential(2.1, 0.3),
        u: Potential = ExponentialPotential(10, 0.2),
    ):
        Behavior.__init__(self, kinematic, radius)
        GeometricState.__init__(self)
        self.tau = tau
        self.step_duration = step_duration
        self.cos_phi = np.cos(phi)
        self.c = c
        self.v = v
        self.u = u

    def potential(self) -> Callable[[Vector2], float]:
        ps = (
            [self.neighbor_potential(neighbor) for neighbor in self.neighbors]
            + [self.obstacle_potential(obstacle) for obstacle in self.static_obstacles]
            + [self.segment_potential(line) for line in self.line_obstacles]
        )

        def f(position: Vector2) -> float:
            return sum(p(position) for p in ps)

        return f

    def neighbor_potential(self, neighbor: Neighbor) -> Callable[[Vector2], float]:
        def f(position: Vector2) -> float:
            value, _ = neighbor_distance(position, neighbor, self.step_duration)
            return self.v(value)[0]

        return f

    def neighbor_repulsion_force(self, neighbor: Neighbor) -> Vector2:
        value, grad = neighbor_distance(self.position, neighbor, self.step_duration)
        _, p_grad = self.v(value)
        return -grad * p_grad

    def obstacle_potential(self, obstacle: Neighbor) -> Callable[[Vector2], float]:
        def f(position: Vector2) -> float:
            value, _ = disc_distance(position, obstacle)
            return self.u(value)[0]

        return f

    def obstacle_repulsion_force(self, obstacle: Disc) -> Vector2:
        value, grad = disc_distance(self.position, obstacle)
        _, p_grad = self.u(value)
        return -grad * p_grad

    def segment_potential(self, line: LineSegment) -> Callable[[Vector2], float]:
        def f(position: Vector2) -> float:
            value, _ = segment_distance(position, line)
            return self.u(value)[0]

        return f

    def segment_repulsion_force(self, line: LineSegment) -> Vector2:
        value, grad = segment_distance(self.position, line)
        _, p_grad = self.u(value)
        return -grad * p_grad

    def in_sight(self, force: Vector2, e: Vector2) -> bool:
        return np.all(np.dot(e, force) > self.cos_phi * np.linalg.norm(force))

    def weight(self, force: Vector2, e: Vector2) -> float:
        return 1.0 if self.in_sight(force, e) else self.c

    def weighted(self, force: Vector2, e: Vector2, sign: int) -> float:
        return self.weight(sign * force, e) * force

    # TODO attractive effects
    # TODO fluctuations
    def compute_desired_velocity(self, dt: float) -> Vector2:
        # Target is in general an area,
        # then target_position is the closed point in that area
        e = self.target_position - self.position
        e /= np.linalg.norm(e)
        target_velocity = e * self.optimal_speed
        # acceleration towards desired velocity
        force = (target_velocity - self.velocity) / self.tau
        # repulsion from neighbors
        force += sum(
            self.weighted(self.neighbor_repulsion_force(neighbor), e, -1)
            for neighbor in self.neighbors
        )
        force += sum(
            self.weighted(self.obstacle_repulsion_force(obstacle), e, -1)
            for obstacle in self.static_obstacles
        )
        force += sum(
            self.weighted(self.segment_repulsion_force(line), e, -1)
            for line in self.line_obstacles
        )
        desired_velocity = self.actuated_twist.velocity + dt * force
        # no need to clamp norm ... this will be done the superclass
        # are we sure?
        return desired_velocity
