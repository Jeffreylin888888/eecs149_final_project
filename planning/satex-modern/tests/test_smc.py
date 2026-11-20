import numpy as np
import pytest
import z3

from satex.smc import GuardedRows, LinearOracle, Rows, solve_smc


def make_oracle(groups, bounds=((-10, 10),), hard=None):
    guarded = []
    for guard, coefficient, rhs in groups:
        rows = Rows()
        rows.add({0: coefficient}, rhs)
        guarded.append(GuardedRows(guard, rows, str(guard)))
    return LinearOracle(bounds, hard or Rows(), guarded)


@pytest.mark.parametrize("strategy", ["deletion", "prefix", "full"])
def test_learns_conflict_and_returns_alternative(strategy):
    a, b, c = z3.Bools("a b c")
    solver = z3.Solver()
    solver.add(a, z3.Or(b, c))
    oracle = make_oracle([(a, 1, 0), (b, -1, -1), (c, 1, -2)])
    result = solve_smc(solver, oracle, strategy=strategy)
    assert result.status == "sat"
    assert result.values[0] <= -2 + 1e-6
    assert z3.is_true(result.model.eval(c))


@pytest.mark.parametrize("strategy", ["deletion", "prefix", "full"])
def test_infeasible_guarded_combination(strategy):
    a, b = z3.Bools("a b")
    solver = z3.Solver()
    solver.add(a, b)
    oracle = make_oracle([(a, 1, 0), (b, -1, -1)])
    result = solve_smc(solver, oracle, strategy=strategy)
    assert result.status == "unsat"
    assert result.conflicts


def test_disabled_guard_is_not_negated_convex_constraint():
    a = z3.Bool("a")
    solver = z3.Solver()
    solver.add(z3.Not(a))
    oracle = make_oracle([(a, 1, -2)], bounds=((0, 0),))
    result = solve_smc(solver, oracle)
    assert result.status == "sat"
    assert result.values[0] == 0


def test_base_infeasibility_and_negative_variable_bounds():
    hard = Rows()
    hard.add({0: 1}, -2)
    oracle = make_oracle([], bounds=((-5, -3),), hard=hard)
    assert solve_smc(z3.Solver(), oracle).status == "sat"
    oracle = make_oracle([], bounds=((0, 1),), hard=hard)
    assert solve_smc(z3.Solver(), oracle).status == "unsat"


def test_unbounded_objective_is_unknown_not_unsat():
    oracle = LinearOracle([(None, None)], Rows(), [], np.array([1.0]))
    assert solve_smc(z3.Solver(), oracle).status == "unknown"


def test_conflict_budget_preserves_sound_subset():
    guards = z3.Bools("a b c d")
    oracle = make_oracle([(guards[0], 1, 0), (guards[1], -1, -1), (guards[2], 1, 10), (guards[3], -1, 10)])
    for budget in [0, 1, 3, 100]:
        core = oracle.conflict([0, 1, 2, 3], "deletion", None, budget)
        assert oracle.solve(core).status == "unsat"
    assert len(oracle.conflict([0, 1, 2, 3], "deletion", None, 100)) == 2


def test_timeout_does_not_become_unsat():
    a = z3.Bool("a")
    oracle = make_oracle([(a, 1, -2)])
    assert solve_smc(z3.Solver(), oracle, time_limit=1e-12).status == "unknown"
