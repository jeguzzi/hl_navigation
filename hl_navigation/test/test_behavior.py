import argparse
import sys

sys.path.append('/Users/Jerome/Dev/ROS/ros2_ws/build/hl_navigation')

import hl_navigation_py


def main(behavior_name: str = "HL") -> None:
    agent = hl_navigation_py.agent_with_name(
        behavior_name, hl_navigation_py.AgentType.HOLONOMIC, 0.1, 0.0)
    if not agent:
        print(f"No behavior with name {behavior_name}")
        sys.exit(1)
    dt = 0.1
    agent.max_speed = 1.0
    agent.optimal_speed = 1.0
    agent.max_angular_speed = 1.0
    agent.horizon = 1.0
    # Start in 0, 0
    agent.position = hl_navigation_py.CVector2(0.0, 0.05)
    agent.velocity = hl_navigation_py.CVector2(0.0, 0.0)
    # Go to 1, 0
    agent.target_position = hl_navigation_py.CVector2(3.0, 0.0)
    agent.set_static_obstacles(
        [hl_navigation_py.Disc(position=hl_navigation_py.CVector2(1.5, 0.0), radius=0.25)])
    # This should be in init
    xs = []
    ys = []
    for _ in range(30):
        agent.update(dt)
        agent.velocity = agent.target_velocity
        agent.position += agent.velocity * dt
        xs.append(agent.position.x)
        ys.append(agent.position.y)
    print(f'Arrived at {agent.position}')
    print(f'xs = {xs}')
    print(f'ys = {ys}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--behavior', help='', type=str, default="HL")
    arg = parser.parse_args()
    main(arg.behavior)
