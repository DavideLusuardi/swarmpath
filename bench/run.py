"""Run solvers on the first k agents of one scenario, for several k, and write a CSV.

    python bench/run.py --map data/maps/random-32-32-10.map \
        --scen data/scen/random-32-32-10-random-1.scen --algos independent --agents 10:50:10
"""

import argparse
import csv

from swarmpath.grid import load_map
from swarmpath.instance import load_scen
from swarmpath.solution import makespan, soc, validate
from swarmpath.solvers import SOLVERS

COLUMNS = ["map", "scen", "algo", "agents", "solved", "valid", "soc", "makespan", "runtime", "conflicts"]


def agent_counts(spec: str) -> list[int]:
    """ "10:50:10" -> [10, 20, 30, 40, 50] """
    first, last, step = [int(v) for v in spec.split(":")]
    return list(range(first, last + 1, step))


def run(map_path: str, scen_path: str, algos: list[str], counts: list[int], time_limit: float, out: str) -> None:
    grid = load_map(map_path)
    with open(out, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=COLUMNS)
        writer.writeheader()
        for algo in algos:
            for k in counts:
                instance = load_scen(scen_path, grid, k)
                result = SOLVERS[algo](instance, time_limit)
                found = result.paths is not None
                row = {
                    "map": grid.name,
                    "scen": instance.name,
                    "algo": algo,
                    "agents": k,
                    "solved": result.solved,
                    "valid": found and not validate(instance, result.paths),
                    "soc": soc(result.paths) if found else "",
                    "makespan": makespan(result.paths) if found else "",
                    "runtime": round(result.runtime, 4),
                    "conflicts": result.stats.get("conflicts", ""),
                }
                writer.writerow(row)
                print(row)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--map", required=True)
    parser.add_argument("--scen", required=True)
    parser.add_argument("--algos", default="independent", help="comma-separated solver names")
    parser.add_argument("--agents", default="10:50:10", help="first:last:step")
    parser.add_argument("--time-limit", type=float, default=30.0)
    parser.add_argument("--out", default="results.csv")
    args = parser.parse_args()
    run(args.map, args.scen, args.algos.split(","), agent_counts(args.agents), args.time_limit, args.out)


if __name__ == "__main__":
    main()
