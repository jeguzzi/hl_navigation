import argparse
import sys
import hl_navigation
import numpy as np
from matplotlib import pyplot as plt


def main(behavior_name: str = "HL", duration: float = 0.0) -> None:
    agents = [hl_navigation.agent_with_name(
        behavior_name, hl_navigation.AgentType.HOLONOMIC, 0.1, 0.2)
        for _ in range(2)]
    if None in agents:
        print(f"No behavior with name {behavior_name}")
        sys.exit(1)
    dt = 0.02
    controllers = []
    for i, agent in enumerate(agents):
        agent.heading_behavior = hl_navigation.HeadingBehavior.IDLE
        agent.max_speed = 1.0
        agent.optimal_speed = 1.0
        agent.max_angular_speed = 1.0
        agent.optimal_angular_speed = 1.0
        agent.horizon = 2.0
        agent.velocity = (0.0, 0.0)
        agent.safety_margin = 0.00
        agent.set_tau(0.25)
        # try:
        #     agent.time_horizon = 1.0
        # except AttributeError:
        #     pass
        controller = hl_navigation.Controller()
        controller.agent = agent
        controller.distance_tolerance = 0.1
        controller.speed_tolerance = 0.05
        if i:
            agent.position = (0.5, 0.0)
            controller.set_target_point(2.0, 0.0, 0.0)
        else:
            agent.position = (1.5, 0.0)
            agent.angle = np.pi
            # not working for 0.0 -> they just advance straight
            controller.set_target_point(0.0, 0.02, 0.0)
        controllers.append(controller)
    t = 0.0

    traces = [[] for _ in controllers]

    while (any(controller.state != hl_navigation.ControllerState.IDLE
               for controller in controllers) and (duration <= 0 or t < duration)):
        for controller in controllers:
            agent = controller.agent
            ns = []
            for a in agents:
                if a != agent:
                    ns.append(hl_navigation.Disc(np.array(a.position), a.radius, 0.0,
                                                 np.array(a.velocity)))
            agent.set_neighbors(ns)

        for trace, controller in zip(traces, controllers):
            controller.update(dt)
            agent = controller.agent
            # print(agent.target_velocity)
            v = (agent.velocity + agent.target_velocity) / 2
            agent.velocity = agent.target_velocity
            agent.position = agent.position + v * dt
            agent.angle += agent.target_twist.angular * dt
            trace.append([agent.position[0], agent.position[1], agent.angle])
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
