import argparse
import sys
import hl_navigation
import numpy as np
from matplotlib import pyplot as plt  # noqa: I100
from typing import cast, List


def main(behavior_name: str = "HL", duration: float = 0.0) -> None:
    behaviors = [hl_navigation.behavior_with_name(
        behavior_name, hl_navigation.TwoWheeled(0.3, 0.1), 0.1) for _ in range(2)]
    if None in behaviors:
        print(f"No behavior with name {behavior_name}")
        sys.exit(1)
    dt = 0.1
    controllers = []
    for i, behavior in enumerate(behaviors):
        behavior.horizon = 2.0
        behavior.velocity = (0.0, 0.0)
        behavior.safety_margin = 0.1
        behavior.heading_behavior = hl_navigation.BehaviorHeading.idle
        try:
            cast(hl_navigation.HLBehavior, behavior).tau= 0.125
        except AttributeError:
            pass
        # try:
        #     behavior.time_horizon = 1.0
        # except AttributeError:
        #     pass
        controller = hl_navigation.Controller(behavior)
        controller.speed_tolerance = 0.05
        if i:
            behavior.position = (0.5, 0.0)
            controller.go_to_position((2.0, 0.0), 0.4)
        else:
            behavior.position = (1.5, 0.0)
            behavior.orientation = np.pi
            # not working for 0.0 -> they just advance straight
            controller.go_to_position((0.0, -0.05), 0.4)
        controllers.append(controller)
    t = 0.0

    traces: List[List[List[float]]] = [[] for _ in controllers]
    while (any(controller.state != hl_navigation.ActionState.idle
               for controller in controllers) and (duration <= 0 or t < duration)):
        for controller in controllers:
            behavior = controller.behavior
            ns = []
            for a in behaviors:
                if a != behavior:
                    ns.append(hl_navigation.Disc(a.position, a.radius, 0.0, a.velocity))
            behavior.neighbors = ns

        for trace, controller in zip(traces, controllers):
            cmd = controller.update(dt)
            controller.behavior.actuate(cmd, dt)
            pose = controller.behavior.pose
            trace.append([pose.position[0], pose.position[1], pose.orientation])
        t += dt
    print(f'All have arrived after {t:.1f} s')
    a_traces = np.array(traces)
    for trace in a_traces:
        x, y, _ = a_traces[:].T
        plt.plot(x, y, '.-')
        plt.axis('equal')
    plt.savefig(f'trace_{behavior_name}.pdf')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--behavior', help='', type=str, default="HL")
    parser.add_argument('--duration', help='', type=float, default=0.0)
    arg = parser.parse_args()
    main(arg.behavior, arg.duration)
