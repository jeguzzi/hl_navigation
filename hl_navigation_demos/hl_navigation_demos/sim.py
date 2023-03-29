import argparse
import time

import hl_navigation as nav
import hl_navigation_sim as sim


class ThymioDemo(sim.Scenario, name="PyThymioDemo"):  # type: ignore[call-arg]

    def __init__(self, behavior_type: str = "HL"):
        super().__init__()
        self._behavior_type = behavior_type

    def init_world(self, world: sim.World) -> None:
        targets = [(1.0, 0.0), (-1.0, 0.0)]
        for i in range(2):
            task = sim.WayPointsTask(targets, True, 0.2)
            se = sim.BoundedStateEstimation(world, 1.0, 1.0)
            kinematic = nav.kinematics.TwoWheeled(0.166, 0.094)
            behavior = nav.Behavior.make_type(self.behavior_type)
            agent = sim.Agent(0.08, behavior, kinematic, task, se, 0.02)
            agent.nav_behavior.optimal_speed = 0.12
            agent.nav_behavior.horizon = 1.0
            agent.nav_behavior.safety_margin = 0.02
            agent.nav_controller.speed_tolerance = 0.01
            agent.pose = nav.Pose2((-0.5 if i else 0.5, 0.5), 0.0)
            world.add_agent(agent)
        world.add_obstacle(nav.Disc((0.0, 0.0), 0.1))

    @sim.register_property("HL", "Behavior name")
    def behavior_type(self) -> str:
        return self._behavior_type

    @behavior_type.setter  # type: ignore[no-redef]
    def behavior_type(self, value: str) -> None:
        self._behavior_type = value


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--behavior", help="", type=str, default="HL")
    arg = parser.parse_args()
    demo = sim.Experiment(0.02, 50 * 60)
    demo.scenario = ThymioDemo(arg.behavior)
    demo.save_directory = "."
    demo.record_pose = True
    demo.name = "PyThymioDemo";
    print("Start simulating 1 minute at 50 ticks per second")
    begin = time.time()
    demo.run()
    end = time.time()
    ms = (end - begin) * 1e3
    print(f"Done simulating in {ms:.1f} ms")