import argparse
import sys
from typing import List
import math
import itertools
import pyenki
import hl_navigation


class Thymio(pyenki.Thymio2):

    def __init__(self, behavior_name: str = "HL",
                 obstacles: List[pyenki.CircularObject] = []):
        super().__init__(use_aseba_units=False)
        self.agent = hl_navigation.agent_with_name(
            behavior_name, hl_navigation.AgentType.TWO_WHEELED, 0.08, 0.01 * self.wheel_axis)
        if not self.agent:
            print(f"No behavior with name {behavior_name}")
            sys.exit(1)
        self.agent.safety_margin = 0.02
        self.agent.max_speed = 0.01 * self.max_wheel_speed
        self.agent.optimal_speed = 0.1
        self.agent.optimal_angular_speed = self.agent.max_angular_speed
        self.agent.horizon = 1.0
        self.controller = hl_navigation.Controller()
        self.controller.agent = self.agent
        self.controller.distance_tolerance = 0.6
        self.controller.speed_tolerance = 0.01

        os = []
        for obstacle in obstacles:
            x, y = obstacle.position
            p = hl_navigation.CVector2(x * 0.01, y * 0.01)
            o = hl_navigation.Disc(position=p, radius=0.01 * obstacle.radius)
            os.append(o)
        self.agent.set_static_obstacles(os)

    def controlStep(self, dt: float) -> None:
        x, y = self.position
        self.agent.angle = self.angle
        self.agent.position = (x * 0.01, y * 0.01)
        x, y = self.velocity
        v = (x * 0.01, y * 0.01)
        self.agent.velocity = v
        # self.agent.set_wheel_speeds([0.01 * self.motor_left_speed, 0.01 * self.motor_right_speed])
        # self.agent.velocity = hl_navigation.CVector2(*self.agent.velocity_from_wheel_speed(
        #     0.01 * self.motor_left_speed, 0.01 * self.motor_right_speed))
        ns = []
        for thymio in self.thymios:
            x, y = thymio.position
            p = (x * 0.01, y * 0.01)
            x, y = thymio.velocity
            v = (x * 0.01, y * 0.01)
            ns.append(hl_navigation.Disc(p, 0.08, 0.05, v))
        self.agent.set_neighbors(ns)
        self.controller.update(dt)
        # print('D', self.agent.desiredVelocity)
        # TODO(Jerome): sono gia' dei memeber (leftSpeed,...)
        left_speed, right_speed = self.agent.target_wheel_speeds
        # print(left_speed, right_speed, self.agent.desiredLinearSpeed, self.agent.desiredAngularSpeed.value())
        self.motor_left_target = 100 * left_speed
        self.motor_right_target = 100 * right_speed
        # print(self.motor_left_speed, self.motor_right_speed, self.controller.state, self.agent.velocity)
        if self.controller.state == hl_navigation.ControllerState.IDLE:
            color, target = next(self.targets)
            self.controller.set_target_point(*target)
            self.set_led_top(*color)


def main(behavior_name: str = "HL") -> None:
    world = pyenki.World()
    r = (1.0, 0.0, 0.0)
    y = (1.0, 1.0, 0.0)
    g = (0.0, 1.0, 0.0)
    b = (0.0, 0.0, 1.0)
    ps = ((0.5, 0),)
    obstacles = []
    for p in ps[:0]:
        cylinder = pyenki.CircularObject(10.0, 10.0, -1, pyenki.Color(0.8, 0.3, 0))
        cylinder.position = tuple([c * 100 for c in p])
        world.add_object(cylinder)
        obstacles.append(cylinder)
    targets = [(r, (2.0, 0.0, 0.0)), (g, (-2.0, 0.0, 0.0)),
               (y, (0.0, 2.0, 0.0)), (b, (0.0, -2.0, 0.0))]
    for color, pose in targets:
        cylinder = pyenki.CircularObject(20.0, 1.0, -1, pyenki.Color(*color))
        cylinder.position = tuple([c * 100 for c in pose[:2]])
        world.add_object(cylinder)
    thymios = set()
    for i in range(30):
        thymio = Thymio(behavior_name=behavior_name, obstacles=obstacles)
        world.add_object(thymio)
        thymio.position = (i * 20 - 150, 0.0)
        thymio.angle = 0.0
        if i % 2 == 0:
            thymio.targets = itertools.cycle(targets[:2])
        else:
            thymio.targets = itertools.cycle(targets[2:])
        thymios.add(thymio)

    for thymio in thymios:
        thymio.thymios = thymios - {thymio}
    world.run_in_viewer(cam_position=(0, 0), cam_altitude=70.0, cam_yaw=0.0,
                        cam_pitch=-math.pi / 2, walls_height=10,
                        orthographic=False, realtime_factor=4.0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--behavior', help='', type=str, default="HL")
    arg = parser.parse_args()
    main(arg.behavior)
