import argparse
import sys
import hl_navigation


def main(behavior_name: str = "HL") -> None:
    agent = hl_navigation.agent_with_name(
        behavior_name, hl_navigation.AgentType.HOLONOMIC, 0.1, 0.2)
    if not agent:
        print(f"No behavior with name {behavior_name}")
        sys.exit(1)
    dt = 0.1
    agent.max_speed = 1.0
    agent.optimal_speed = 1.0
    agent.max_angular_speed = 1.0
    agent.optimal_angular_speed = 1.0
    agent.horizon = 1.0
    # Start in 0, 0
    agent.position = hl_navigation.CVector2(0.0, 0.05)
    agent.velocity = hl_navigation.CVector2(0.0, 0.0)
    # agent.set_static_obstacles(
    #     [hl_navigation.Disc(position=hl_navigation.CVector2(0.5, 0.0), radius=0.1)])
    controller = hl_navigation.Controller()
    controller.agent = agent
    controller.set_target_point(0.0, 1.0, 0.0)
    controller.distance_tolerance = 0.2
    controller.speed_tolerance = 0.05
    t = 0.0
    while controller.state != hl_navigation.ControllerState.IDLE and t < 2:
        controller.update(dt)
        agent.velocity = agent.target_velocity
        agent.position += agent.velocity * dt
        agent.angle += hl_navigation.CRadians(agent.target_twist.angular * dt)
        print(agent.position, agent.angle.value(), agent.desired_velocity, agent.target_twist)
        t += dt
    print(f'Arrived at {agent.position} after {t:.1f} s')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--behavior', help='', type=str, default="HL")
    arg = parser.parse_args()
    main(arg.behavior)
