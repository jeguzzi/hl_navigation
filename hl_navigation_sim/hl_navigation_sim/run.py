import argparse

import hl_navigation_sim as sim


def main() -> None:
    # nav.load_plugins()
    sim.load_py_plugins()
    parser = argparse.ArgumentParser()
    parser.add_argument('yaml', help='yaml string', type=str, default="")
    parser.add_argument('--input', help='yaml file', type=str, default="")
    arg = parser.parse_args()
    if arg.input:
        with open(arg.input, 'r') as f:
            yaml = f.read()
    else:
        yaml = arg.yaml
    experiment = sim.load_experiment(yaml)
    if experiment:
        # print(sim.dump(experiment))
        experiment.run()
    else:
        print("could not load yaml")
