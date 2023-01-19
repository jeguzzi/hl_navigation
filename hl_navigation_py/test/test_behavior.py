import argparse
import sys
import hl_navigation
import numpy as np
from matplotlib import pyplot as plt


def main(behavior_name: str = "HL", duration: float = 1.0) -> None:
    agent = hl_navigation.agent_with_name(
        behavior_name, hl_navigation.AgentType.TWO_WHEELED, 0.1, 0.1)
    if not agent:
        print(f"No behavior with name {behavior_name}")
        sys.exit(1)
    dt = 0.1
    agent.max_speed = 1.0
    agent.optimal_speed = 1.0
    agent.max_angular_speed = 1.0
    agent.horizon = 1.0

    # Start in 0, 0
    agent.position = (0.0, 0.05)
    agent.velocity = (0.0, 0.0)
    agent.angle = 0.0
    # Go to 1, 0
    agent.target_position = (3.0, 0.0)
    agent.set_static_obstacles(
        [hl_navigation.Disc(position=(1.5, 0.0), radius=0.5)])

    # This should be in init
    poses = []
    t = 0.0
    while t < duration:
        t += dt
        agent.update(dt)
        agent.velocity = agent.target_velocity
        agent.position = agent.position + agent.velocity * dt
        agent.angle += agent.target_twist.angular * dt
        pose = [agent.position[0], agent.position[1], agent.angle]
        poses.append(pose)
    print(f'Arrived at {agent.position}')
    # print(f'poses = {poses}')
    trace = np.array(poses)
    x, y, _ = trace[:].T
    plt.plot(x, y, 'o--')
    plt.axis('equal')
    plt.savefig('trace.png')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--behavior', help='', type=str, default="HL")
    parser.add_argument('--duration', help='', type=float, default=1.0)
    arg = parser.parse_args()
    main(arg.behavior, arg.duration)
