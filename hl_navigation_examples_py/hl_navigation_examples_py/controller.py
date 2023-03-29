import argparse
import sys

import hl_navigation as nav
import hl_navigation.kinematics


def done_cb(state: nav.ActionState) -> None:
    print('Arrived' if state == nav.ActionState.success else 'Failed')


def running_cb(t: float) -> None:
    print(f"In progress ... expected to last at least {t:.2f} s")


def main(behavior_name: str = "HL") -> None:
    """Test the 2D control to reach a point

    Args:
        behavior_name (str, optional): behavior name
    """
    behavior = nav.Behavior.make_type(behavior_name)
    if not behavior:
        print(f'No behavior with name {behavior_name}')
        sys.exit(1)
    print(f'Use behavior {behavior_name}')
    behavior.kinematic = nav.kinematics.TwoWheeled(1.0, 0.1)
    behavior.radius = 0.1
    dt = 0.1
    behavior.horizon = 1.0
    behavior.position = (0.0, 0.00)
    controller = nav.Controller(behavior)
    controller.speed_tolerance = 0.05
    controller.set_cmd_cb(lambda cmd: behavior.actuate(cmd, dt))
    action = controller.go_to_position((0.0, 1.0), 0.2)
    action.done_cb = done_cb
    action.running_cb = running_cb
    t = 0.0
    while not action.done:
        cmd = controller.update(dt)
        t += dt
    print(f'Arrived at {behavior.position} after {t:.1f} s')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--behavior', help='', type=str, default="HL")
    arg = parser.parse_args()
    main(arg.behavior)
