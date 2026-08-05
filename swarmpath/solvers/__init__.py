"""All solvers, by name. Every solver is ``solve(instance, time_limit, **params) -> Result``."""

from swarmpath.solvers import independent

SOLVERS = {
    "independent": independent.solve,
}
