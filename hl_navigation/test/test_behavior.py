import argparse
import sys

sys.path.append('/Users/Jerome/Dev/ROS/ros2_ws/build/hl_navigation')

import hl_navigation_py


def main(behavior_name: str = "HL") -> None:
    agent = hl_navigation_py.agent_with_name(behavior_name)
    if not agent:
        print(f"No behavior with name {behavior_name}")
        sys.exit(1)
    dt = 0.1
    agent.type = hl_navigation_py.AgentType.HOLONOMIC
    agent.radius = 0.1
    agent.setMaxSpeed(1.0)
    agent.setOptimalSpeed(1.0)
    agent.setMaxAngularSpeed(1.0)
    agent.setHorizon(1.0)
    # Start in 0, 0
    agent.position = hl_navigation_py.CVector2(0.0, 0.0)
    agent.velocity = hl_navigation_py.CVector2(0.0, 0.0)
    # Go to 1, 0
    agent.targetPosition = hl_navigation_py.CVector2(1.0, 0.0)
    # This should be in init
    xs = []
    ys = []
    for _ in range(20):
        agent.clearObstacles()
        agent.addObstacleAtPoint(hl_navigation_py.CVector2(0.5, 0.0), 0.0, 0.0)
        agent.updateDesiredVelocity()
        agent.updateVelocity(dt)
        agent.velocity = agent.desiredVelocity
        agent.position += agent.velocity * dt
        xs.append(agent.position.get_x())
        ys.append(agent.position.get_y())
    print(f'Arrived at {agent.position}')
    print(f'xs = {xs}')
    print(f'ys = {ys}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--behavior', help='', type=str, default="HL")
    arg = parser.parse_args()
    main(arg.behavior)
