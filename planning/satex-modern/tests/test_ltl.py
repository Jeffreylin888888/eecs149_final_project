import itertools
import json

import pytest
import z3

from satex.ltl import (
    FALSE,
    TRUE,
    Always,
    And,
    Atom,
    Count,
    Eventually,
    Formula,
    Implies,
    Next,
    Not,
    Or,
    Release,
    Until,
    atoms,
    encode,
    evaluate,
)

A = Atom("r", "A")
B = Atom("r", "B")


def symbolic_value(formula, trace, loop_start=None):
    def atom_at(robot, region, t):
        active = trace[t].get(robot, ())
        value = active == region if isinstance(active, str) else region in active
        return z3.BoolVal(value)

    value = z3.simplify(encode(formula, len(trace), atom_at, loop_start))
    assert z3.is_true(value) or z3.is_false(value)
    return z3.is_true(value)


def assert_value(formula, trace, expected, loop_start=None):
    assert evaluate(formula, trace, loop_start) is expected
    assert symbolic_value(formula, trace, loop_start) is expected


def test_strong_next_and_finite_boundary_duals():
    trace = [{"r": "A"}]
    assert_value(Next(TRUE), trace, False)
    assert_value(Not(Next(FALSE)), trace, True)
    assert_value(Always(A), trace, True)
    assert_value(Eventually(A), trace, True)
    assert_value(Until(FALSE, A), trace, True)
    assert_value(Until(A, B), trace, False)
    assert_value(Release(B, A), trace, True)
    assert_value(Release(A, B), trace, False)
    assert_value(Always(Next(TRUE)), trace, False)
    assert_value(Next(TRUE), trace, True, loop_start=0)


def test_until_requires_right_witness_before_left_failure():
    assert_value(Until(A, B), [{"r": "A"}, {"r": "A"}, {"r": "B"}], True)
    assert_value(Until(A, B), [{"r": "A"}, {}, {"r": "B"}], False)
    assert_value(Until(A, B), [{"r": "B"}, {}], True)


def test_lasso_eventuality_is_least_fixed_point():
    trace = [{"r": "A"}, {"r": "A"}]
    assert_value(Eventually(B), trace, False, loop_start=0)
    assert_value(Until(A, B), trace, False, loop_start=1)
    assert_value(Always(A), trace, True, loop_start=0)
    assert_value(Release(B, A), trace, True, loop_start=1)
    assert_value(Always(Eventually(B)), trace, False, loop_start=0)


def test_lasso_distinguishes_prefix_and_cycle():
    prefix_only = [{"r": "A"}, {"r": "B"}]
    assert_value(Eventually(A), prefix_only, True, loop_start=1)
    assert_value(Always(Eventually(A)), prefix_only, False, loop_start=1)
    assert_value(Eventually(Always(B)), prefix_only, True, loop_start=1)
    assert_value(Eventually(Always(A)), prefix_only, False, loop_start=1)
    alternating = [{"r": "A"}, {"r": "B"}]
    assert_value(Always(Eventually(A)), alternating, True, loop_start=0)
    assert_value(Eventually(Always(A)), alternating, False, loop_start=0)
    assert_value(Next(Next(A)), alternating, True, loop_start=0)
    assert_value(Always(Implies(A, Next(B))), alternating, True, loop_start=0)


def test_until_can_wrap_to_a_cycle_witness():
    trace = [{"r": "B"}, {"r": "A"}, {"r": "A"}]
    assert_value(Next(Until(A, B)), trace, True, loop_start=0)
    assert_value(Next(Until(A, B)), trace, False, loop_start=1)
    assert_value(Next(Until(A, B)), trace, False)


def test_release_is_greatest_fixed_point_with_correct_witness():
    # Until release is witnessed by A, B must hold, including the A state.
    assert_value(Release(A, B), [{"r": {"B"}}, {"r": {"A", "B"}}, {}], True)
    assert_value(Release(A, B), [{"r": {"B"}}, {"r": {"A"}}, {}], False)
    assert_value(Release(A, B), [{"r": {"B"}}, {}], False, loop_start=0)
    assert_value(Release(A, B), [{"r": {"B"}}], True, loop_start=0)


def test_count_bounds_empty_sets_and_overlapping_regions():
    trace = [{"a": {"goal", "workspace"}, "b": "goal", "c": "other"}]
    assert_value(Count("goal", ["a", "b", "c"], minimum=2, maximum=2), trace, True)
    assert_value(Count("goal", ["a", "b", "c"], minimum=3), trace, False)
    assert_value(Count("goal", ["a", "b", "c"], minimum=0, maximum=1), trace, False)
    assert_value(Count("goal", [], minimum=0, maximum=0), trace, True)
    assert_value(Count("goal", []), trace, False)
    assert_value(Count("goal", ["missing"]), trace, False)
    assert atoms(Count("goal", ["a", "b"])) == {("a", "goal"), ("b", "goal")}


def test_json_round_trip_and_aliases():
    formula = Always(Implies(Count("goal", ["r", "s"], minimum=1, maximum=2), Eventually(A & ~B))) | Next(
        FALSE
    )
    restored = Formula.from_dict(json.loads(json.dumps(formula.to_dict())))
    assert restored == formula
    assert hash(restored) == hash(formula)
    assert atoms(formula) == {("r", "goal"), ("s", "goal"), ("r", "A"), ("r", "B")}
    assert Formula.from_dict({"op": "G", "args": [{"op": "true"}]}) == Always(TRUE)
    assert_value(And(), [{}], True)
    assert_value(Or(), [{}], False)
    with pytest.raises(TypeError, match="truth value"):
        bool(formula)


@pytest.mark.parametrize(
    "horizon,loop_start", [(0, None), (-1, None), (2, -1), (2, 2), (2, True), (True, None)]
)
def test_invalid_horizon_or_loop(horizon, loop_start):
    with pytest.raises(ValueError):
        encode(TRUE, horizon, lambda *_: z3.BoolVal(True), loop_start)


def test_invalid_inputs_are_rejected():
    with pytest.raises(ValueError):
        evaluate(TRUE, [])
    with pytest.raises(ValueError):
        evaluate(TRUE, [{}], loop_start=1)
    with pytest.raises(TypeError):
        evaluate(TRUE, ["not a state"])
    for construction in [
        lambda: Atom("", "A"),
        lambda: Count("A", ["r", "r"]),
        lambda: Count("A", "robot"),
        lambda: Count("A", ["r"], minimum=-1),
        lambda: Count("A", ["r"], minimum=2, maximum=1),
        lambda: Formula("unknown"),
        lambda: Formula("until", (A,)),
        lambda: Formula.from_dict({"op": "true", "typo": 1}),
    ]:
        with pytest.raises(ValueError):
            construction()


def test_symbolic_satisfiability_does_not_allow_false_eventualities():
    regions = [z3.Bool(f"p_{t}") for t in range(4)]
    solver = z3.Solver()
    solver.add(encode(Always(Eventually(A)), 4, lambda *_args: regions[_args[2]], loop_start=2))
    solver.add(regions[0], regions[1], z3.Not(regions[2]), z3.Not(regions[3]))
    assert solver.check() == z3.unsat
    solver = z3.Solver()
    solver.add(encode(Always(Eventually(A)), 4, lambda *_args: regions[_args[2]], loop_start=2))
    solver.add(regions[2])
    assert solver.check() == z3.sat


def test_symbolic_and_independent_evaluator_agree_exhaustively():
    formulas = [
        A,
        B,
        ~A,
        A & B,
        A | B,
        Implies(A, B),
        Next(A),
        Next(Next(B)),
        Until(A, B),
        Release(A, B),
        Eventually(A),
        Always(A),
        Always(Eventually(A)),
        Eventually(Always(A)),
        Until(Next(A), Always(B)),
        Release(Eventually(A), Next(B)),
        Always(Implies(A, Next(Eventually(B)))),
        Until(Release(A, B), Until(B, A)),
        Not(Until(Not(A), Not(B))),
        Count("A", ["r"], minimum=0, maximum=0),
    ]
    # All independent valuations of A and B for up to three unique states,
    # tested as finite traces and with every possible lasso entry point.
    for n in range(1, 4):
        for bits in itertools.product([False, True], repeat=2 * n):
            trace = [{"r": {name for j, name in enumerate(("A", "B")) if bits[2 * t + j]}} for t in range(n)]
            for loop_start in [None, *range(n)]:
                for formula in formulas:
                    assert symbolic_value(formula, trace, loop_start) == evaluate(
                        formula, trace, loop_start
                    ), (formula, trace, loop_start)


def test_horizon_thirty_nested_temporal_formula():
    trace = [{"r": "A" if t % 2 == 0 else "B"} for t in range(30)]
    formula = Always(Eventually(A)) & Always(Eventually(B)) & Always(Until(A | B, Next(A)))
    assert_value(formula, trace, True, loop_start=0)
