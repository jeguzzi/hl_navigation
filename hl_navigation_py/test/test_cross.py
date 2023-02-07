import argparse
import sys
import hl_navigation
import numpy as np
from matplotlib import pyplot as plt


def main(behavior_name: str = "HL", duration: float = 0.0) -> None:
    behaviors = [hl_navigation.behavior_with_name(
        behavior_name, hl_navigation.AgentType.HOLONOMIC, 0.1, 0.2)
        for _ in range(2)]
    if None in behaviors:
        print(f"No behavior with name {behavior_name}")
        sys.exit(1)
    dt = 0.02
    controllers = []
    for i, behavior in enumerate(behaviors):
        behavior.heading_behavior = hl_navigation.HeadingBehavior.IDLE
        behavior.max_speed = 1.0
        behavior.optimal_speed = 1.0
        behavior.max_angular_speed = 1.0
        behavior.optimal_angular_speed = 1.0
        behavior.horizon = 2.0
        behavior.velocity = (0.0, 0.0)
        behavior.safety_margin = 0.00
        behavior.set_tau(0.25)
        # try:
        #     behavior.time_horizon = 1.0
        # except AttributeError:
        #     pass
        controller = hl_navigation.Controller()
        controller.behavior = behavior
        controller.distance_tolerance = 0.1
        controller.speed_tolerance = 0.05
        if i:
            behavior.position = (0.5, 0.0)
            controller.set_target_point(2.0, 0.0, 0.0)
        else:
            behavior.position = (1.5, 0.0)
            behavior.angle = np.pi
            # not working for 0.0 -> they just advance straight
            controller.set_target_point(0.0, 0.02, 0.0)
        controllers.append(controller)
    t = 0.0

    traces = [[] for _ in controllers]

    while (any(controller.state != hl_navigation.ControllerState.IDLE
               for controller in controllers) and (duration <= 0 or t < duration)):
        for controller in controllers:
            behavior = controller.behavior
            ns = []
            for a in behaviors:
                if a != behavior:
                    ns.append(hl_navigation.Disc(np.array(a.position), a.radius, 0.0,
                                                 np.array(a.velocity)))
            behavior.set_neighbors(ns)

        for trace, controller in zip(traces, controllers):
            controller.update(dt)
            behavior = controller.behavior
            # print(behavior.target_velocity)
            v = (behavior.velocity + behavior.target_velocity) / 2
            behavior.velocity = behavior.target_velocity
            behavior.position = behavior.position + v * dt
            behavior.angle += behavior.target_twist.angular * dt
            trace.append([behavior.position[0], behavior.position[1], behavior.angle])
        t += dt
    print(f'All have arrived after {t:.1f} s')
    traces = np.array(traces)
    for trace in traces:
        x, y, _ = trace[:].T
        plt.plot(x, y, '-')
        plt.axis('equal')
    plt.savefig(f'trace_{behavior_name}.pdf')
    plt.figure()
    for trace in traces:
        _, _, z = trace[:].T
        plt.plot(z, '.-')
        plt.axis('equal')
    plt.savefig(f'a_{behavior_name}.pdf')



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--behavior', help='', type=str, default="HL")
    parser.add_argument('--duration', help='', type=float, default=0.0)
    arg = parser.parse_args()
    main(arg.behavior, arg.duration)
