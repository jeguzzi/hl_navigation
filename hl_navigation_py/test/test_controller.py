import argparse
import sys
import hl_navigation


def main(behavior_name: str = "HL") -> None:
    behavior = hl_navigation.behavior_with_name(
        behavior_name, hl_navigation.AgentType.HOLONOMIC, 0.1, 0.2)
    if not behavior:
        print(f"No behavior with name {behavior_name}")
        sys.exit(1)
    dt = 0.1
    behavior.max_speed = 1.0
    behavior.optimal_speed = 1.0
    behavior.max_angular_speed = 1.0
    behavior.optimal_angular_speed = 1.0
    behavior.horizon = 1.0
    try:
        behavior.set_time_horizon(1.0)
    except AttributeError as e:
        print(e)
        pass
    print(behavior)
    # Start in 0, 0
    behavior.position = (0.0, 0.05)
    behavior.velocity = (0.0, 0.0)
    behavior.angle = 0.0
    # behavior.set_static_obstacles(
    #     [hl_navigation.Disc(position=hl_navigation.Vector2(0.5, 0.0), radius=0.1)])
    controller = hl_navigation.Controller()
    controller.behavior = behavior
    controller.set_target_point(0.0, 1.0, 0.0)
    controller.distance_tolerance = 0.2
    controller.speed_tolerance = 0.05
    t = 0.0
    while controller.state != hl_navigation.ControllerState.IDLE and t < 2:
        controller.update(dt)
        behavior.velocity = behavior.target_velocity
        behavior.position = behavior.position + behavior.velocity * dt
        behavior.angle += behavior.target_twist.angular * dt
        print(behavior.position, behavior.angle, behavior.desired_velocity, behavior.target_twist)
        t += dt
    print(f'Arrived at {behavior.position} after {t:.1f} s')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--behavior', help='', type=str, default="HL")
    arg = parser.parse_args()
    main(arg.behavior)
