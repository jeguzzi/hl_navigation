import argparse
import time

import hl_navigation
import numpy as np

# TODO(Jerome): add obstacle


def go_to(controller: hl_navigation.Controller, target: np.ndarray) -> None:
    action = controller.go_to_position(target, 0.2)

    def done_cb(state):
        if state == hl_navigation.ActionState.success:
            go_to(controller, -target)
    action.done_cb = done_cb


def run(behavior_name: str = "HL") -> None:
    dt = 0.02
    controllers = []
    agents = []
    target = np.array((1.0, 0.0))
    obstacles = [hl_navigation.Disc((0.0, 0.0), 0.1)]
    for p in ((0.5, 0.0), (-0.5, 0.5)):
        behavior = hl_navigation.behavior_with_name(
            behavior_name, hl_navigation.TwoWheeled(0.166, 0.094), 0.08)
        behavior.horizon = 1.0
        behavior.safety_margin = 0.02
        behavior.optimal_speed = 0.12
        try:
            behavior.static_obstacles = obstacles
        except AttributeError:
            pass
        controller = hl_navigation.Controller(behavior)
        controller.speed_tolerance = 0.01
        behavior.position = p

        controllers.append(controller)
        agents.append(behavior)
        go_to(controller, target)
    print('Start simulating 1 minute at 50 ticks per second')
    a = time.time()
    for _ in range(50 * 60):
        for controller in controllers:
            this = controller.behavior
            try:
                controller.behavior.neighbors = [
                    hl_navigation.Neighbor(
                            agent.position, agent.radius, agent.velocity, 0)
                    for agent in agents if agent != this]
            except AttributeError:
                pass
        for controller in controllers:
            cmd = controller.update(dt)
            controller.behavior.actuate(cmd, dt)
            # if controller.idle:
            #     go_to(controller, -controller.behavior.target_position)

    print(f'Done simulating in {1000 * (time.time() - a):.1f} ms')


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--behavior', help='', type=str, default="HL")
    arg = parser.parse_args()
    run(arg.behavior)  