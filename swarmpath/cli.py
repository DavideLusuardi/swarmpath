"""Command line: ``python -m swarmpath solve --map M --scen S -k 20 --algo independent --show``."""

import argparse

from swarmpath.conflicts import find_conflicts
from swarmpath.grid import load_map
from swarmpath.instance import load_scen, random_instance
from swarmpath.solution import makespan, soc, validate
from swarmpath.solvers import SOLVERS


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="swarmpath")
    commands = parser.add_subparsers(dest="command", required=True)
    solve = commands.add_parser("solve", help="solve one MAPF instance")
    solve.add_argument("--map", required=True, help="MovingAI .map file")
    source = solve.add_mutually_exclusive_group(required=True)
    source.add_argument("--scen", help="MovingAI .scen file (its first k agents are used)")
    source.add_argument("--random", action="store_true", help="random starts and goals")
    solve.add_argument("-k", "--agents", type=int, required=True, help="number of agents")
    solve.add_argument("--seed", type=int, default=0, help="seed for --random")
    solve.add_argument("--algo", default="independent", choices=sorted(SOLVERS))
    solve.add_argument("--time-limit", type=float, default=60.0, help="seconds")
    solve.add_argument("--show", action="store_true", help="animate the solution in a window")
    solve.add_argument("--save", help="save the animation as a GIF")
    args = parser.parse_args(argv)

    grid = load_map(args.map)
    if args.scen:
        instance = load_scen(args.scen, grid, args.agents)
    else:
        instance = random_instance(grid, args.agents, args.seed)
    result = SOLVERS[args.algo](instance, args.time_limit)

    print(f"instance  {instance.name} ({grid.width}x{grid.height}, {instance.num_agents} agents)")
    print(f"algo      {result.algo}")
    print(f"solved    {result.solved}")
    print(f"runtime   {result.runtime:.3f}s")
    print(f"stats     {result.stats}")
    if result.paths is None:
        return 1
    print(f"SOC       {soc(result.paths)}")
    print(f"makespan  {makespan(result.paths)}")
    conflicts = find_conflicts(result.paths)
    print(f"conflicts {len(conflicts)}")
    for c in conflicts[:10]:
        where = c.u if c.kind == "vertex" else f"{c.u}<->{c.v}"
        print(f"  t={c.t:<4} {c.kind:<6} agents {c.a1},{c.a2} at {where}")
    if len(conflicts) > 10:
        print(f"  ... and {len(conflicts) - 10} more")
    print(f"valid     {not validate(instance, result.paths)}")

    if args.show or args.save:
        from swarmpath.viz import animate  # imported here: matplotlib is slow to load

        animate(instance, result.paths, conflicts, show=args.show, save=args.save, title=result.algo)
    return 0
