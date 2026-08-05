from swarmpath.conflicts import Conflict, find_conflicts

A, B, C, D, E = [(x, 0) for x in range(5)]  # five cells in a row


def test_no_conflicts_on_disjoint_paths():
    assert find_conflicts([[A, B], [D, E]]) == []


def test_vertex_conflict():
    assert find_conflicts([[A, B, C], [C, B, A]]) == [Conflict("vertex", 0, 1, 1, B)]


def test_swap_conflict():
    assert find_conflicts([[A, B], [B, A]]) == [Conflict("edge", 0, 1, 1, A, B)]


def test_agent_parked_at_goal_still_occupies_it():
    # agent 0 reaches C at t=1 and stays; agent 1 passes through C at t=2
    assert find_conflicts([[B, C], [A, B, C, D]]) == [Conflict("vertex", 0, 1, 2, C)]


def test_following_is_not_a_conflict():
    assert find_conflicts([[B, C, D], [A, B, C]]) == []


def test_three_agents_in_one_cell_give_three_pairs():
    assert len(find_conflicts([[A, D], [B, D], [C, D]])) == 3
