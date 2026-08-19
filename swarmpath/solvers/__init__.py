"""All solvers, by name. Every solver is ``solve(instance, time_limit, **params) -> Result``."""

from swarmpath.solvers import eecbs, independent, lns2, pp

SOLVERS = {
    "independent": independent.solve,
    "eecbs": eecbs.solve,
    "pp": pp.solve,
    "lns2": lns2.solve,
}
