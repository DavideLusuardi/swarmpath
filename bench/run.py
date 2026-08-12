"""Run solvers on the first k agents of some scenarios, for several k, and write a CSV.

    python bench/run.py --map data/maps/random-32-32-10.map \
        --scen data/scen/random-32-32-10-random-{1..5}.scen \
        --algos independent,eecbs:w=1,eecbs:w=1.2,eecbs:w=1.2:bypass=0 --agents 10:50:10

An algorithm is a solver name followed by its parameters, ``name:key=value:key=value``.
"""

import argparse
import csv

from swarmpath.grid import load_map
from swarmpath.instance import load_scen
from swarmpath.solution import makespan, soc, validate
from swarmpath.solvers import SOLVERS

COLUMNS = ["map", "scen", "algo", "agents", "solved", "valid", "soc", "makespan", "runtime", "conflicts", "lb", "expanded"]


def agent_counts(spec: str) -> list[int]:
    """ "10:50:10" -> [10, 20, 30, 40, 50] """
    first, last, step = [int(v) for v in spec.split(":")]
    return list(range(first, last + 1, step))


def parse_algo(spec: str) -> tuple[str, dict]:
    """ "eecbs:w=1.2:bypass=0" -> ("eecbs", {"w": 1.2, "bypass": 0.0}) """
    name, *settings = spec.split(":")
    params = {}
    for setting in settings:
        key, value = setting.split("=")
        params[key] = float(value)
    return name, params


def run(map_path: str, scen_paths: list[str], algos: list[str], counts: list[int], time_limit: float, out: str) -> None:
    grid = load_map(map_path)
    with open(out, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=COLUMNS)
        writer.writeheader()
        for spec in algos:
            name, params = parse_algo(spec)
            for scen_path in scen_paths:
                for k in counts:
                    instance = load_scen(scen_path, grid, k)
                    result = SOLVERS[name](instance, time_limit, **params)
                    found = result.paths is not None
                    row = {
                        "map": grid.name,
                        "scen": instance.name,
                        "algo": spec,
                        "agents": k,
                        "solved": result.solved,
                        "valid": found and not validate(instance, result.paths),
                        "soc": soc(result.paths) if found else "",
                        "makespan": makespan(result.paths) if found else "",
                        "runtime": round(result.runtime, 4),
                        "conflicts": result.stats.get("conflicts", ""),
                        "lb": result.stats.get("lb", ""),
                        "expanded": result.stats.get("expanded", ""),
                    }
                    writer.writerow(row)
                    f.flush()
                    print(row)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--map", required=True)
    parser.add_argument("--scen", required=True, nargs="+", help="one or more .scen files")
    parser.add_argument("--algos", default="independent", help="comma-separated, e.g. independent,eecbs:w=1.2")
    parser.add_argument("--agents", default="10:50:10", help="first:last:step")
    parser.add_argument("--time-limit", type=float, default=30.0)
    parser.add_argument("--out", default="results.csv")
    args = parser.parse_args()
    run(args.map, args.scen, args.algos.split(","), agent_counts(args.agents), args.time_limit, args.out)


if __name__ == "__main__":
    main()
