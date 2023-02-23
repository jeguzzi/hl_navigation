import argparse
import sys
import hl_navigation


def main(behavior_name: str = "HL") -> None:
    """Test the 2D control to reach a point
    
    Args:
        behavior_name (str, optional): behavior name
    """
    behavior = hl_navigation.behavior_with_name(
        behavior_name, hl_navigation.TwoWheeled(1.0, 0.1), 0.1)
    if not behavior:
        print(f"No behavior with name {behavior_name}")
        sys.exit(1)
    print(f'Use behavior {behavior_name} - {behavior.__class__.__name__}')
    dt = 0.1
    behavior.horizon = 1.0
    behavior.position = (0.0, 0.00)
    controller = hl_navigation.Controller(behavior)
    controller.speed_tolerance = 0.05
    controller.set_cmd_cb(lambda cmd: behavior.actuate(cmd, dt))
    action = controller.go_to_position((0.0, 1.0), 0.2)
    action.done_cb = lambda state: print('Arrived' if state == hl_navigation.ActionState.success else 'Failed')
    action.running_cb = lambda t: print(f"In progress ... expected to last at least {t:.2f} s")
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
