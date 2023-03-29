import argparse  # noqa: D100
import sys

import hl_navigation as nav
import hl_navigation.kinematics


def main(behavior_name: str = 'HL') -> None:
    """Test obstacle avoidance behavior for a few iterations

    Args:
        behavior_name (str, optional): The name of the behavior
    """
    behavior = nav.Behavior.make_type(behavior_name)
    if not behavior:
        print(f'No behavior with name {behavior_name}')
        sys.exit(1)
    print(f'Use behavior {behavior_name}')
    behavior.kinematic = nav.kinematics.Holonomic(1.0, 1.0)
    behavior.radius = 0.1
    dt = 0.1
    behavior.horizon = 5.0
    behavior.position = (0.0, 0.05)
    behavior.target_position = (10.0, 0.0)
    behavior.static_obstacles = [nav.Disc(position=(1.5, 0.0), radius=0.5)]
    for _ in range(30):
        cmd = behavior.cmd_twist(dt, True)
        behavior.actuate(cmd, dt)
    pose = behavior.pose
    twist = behavior.twist
    print(f'End loop @ ({pose.position[0]:.3f}, {pose.position[1]:.3f})'
          f'({twist.velocity[0]:.3f}, {twist.velocity[1]:.3f})')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--behavior', help='', type=str, default='HL')
    arg = parser.parse_args()
    main(arg.behavior)
