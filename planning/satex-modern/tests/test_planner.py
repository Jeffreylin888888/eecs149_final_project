"""End-to-end feasibility and adversarial independent-validation regressions."""

from copy import deepcopy

import numpy as np
import pytest
import z3

from satex.ltl import TRUE, Always, And, Atom, Eventually, Next
from satex.models import Dynamics, Problem, Region, Robot, Workspace, integrator_chain
from satex.planner import Planner, PlanResult
from satex.smc import GuardedRows, LinearOracle, LPResult, Rows, SMCResult, solve_smc
from satex.validation import validate_plan


def line_problem(specification=None):
    workspace = Workspace(
        (
            Region.box("start", [0], [0.2]),
            Region.box("middle", [0.2], [1.8]),
            Region.box("goal", [1.8], [2.2]),
        ),
        {"start": ("middle",), "middle": ("start", "goal"), "goal": ("middle",)},
    )
    robot = Robot("r", integrator_chain(1, 1, input_bound=1, state_bound=3), [0])
    return Problem(
        workspace,
        (robot,),
        Eventually(Atom("r", "goal")) if specification is None else specification,
        separation=0,
    )


@pytest.mark.parametrize("backend", ["smc", "smt"])
def test_terminal_state_requires_acceleration_and_braking(backend):
    dynamics = integrator_chain(1, 2, input_bound=1, state_bound=3)
    robot = Robot("r", dynamics, [0, 0], terminal_state=(1, 0))
    problem = Problem(Workspace((Region.box("free", [-1], [3]),), {}), (robot,), TRUE)
    assert Planner(problem).solve(2, backend=backend).status == "unsat"
    result = Planner(problem).solve(3, backend=backend)
    assert result.status == "sat"
    np.testing.assert_allclose(result.states["r"][-1], [1, 0], atol=1e-7)
    np.testing.assert_allclose(result.controls["r"][:, 0], [1, -1], atol=1e-7)
    # Make a dynamically valid, bounded trajectory that fails only the endpoint.
    result.controls["r"][-1] = 0
    result.states["r"][-1] = dynamics.A @ result.states["r"][-2]
    report = validate_plan(problem, result)
    assert report["max_dynamics_residual"] < 1e-7
    assert not report["valid"]
    assert any("terminal state" in error for error in report["errors"])


def cycle_problem():
    workspace = Workspace(
        (Region.box("left", [0], [0]), Region.box("right", [1], [1])),
        {"left": ("right",), "right": ("left",)},
    )
    robot = Robot("r", integrator_chain(1, 1, input_bound=1, state_bound=2), [0])
    return Problem(
        workspace,
        (robot,),
        And(Always(Eventually(Atom("r", "left"))), Always(Eventually(Atom("r", "right")))),
        separation=0,
    )


@pytest.mark.parametrize("backend", ["smc", "smt"])
def test_finite_final_region_and_all_transition_residuals(backend):
    problem = line_problem(Next(Next(Atom("r", "goal"))))
    result = Planner(problem).solve(3, backend=backend)
    assert result.status == "sat", result.stats
    assert result.states["r"].shape == (3, 1)
    assert result.controls["r"].shape == (2, 1)
    assert result.regions["r"][-1] == "goal"
    assert problem.workspace.region_map["goal"].contains(result.states["r"][-1])
    np.testing.assert_allclose(
        result.states["r"][1:] - result.states["r"][:-1], result.controls["r"], atol=1e-7
    )
    assert validate_plan(problem, result)["valid"]


def test_search_grows_horizon_and_unsat_is_bounded():
    planner = Planner(line_problem())
    bounded = planner.search(1, 2)
    assert bounded.status == "unsat"
    assert bounded.stats["searched_horizons"] == [1, 2]
    assert [attempt["status"] for attempt in bounded.stats["attempts"]] == ["unsat", "unsat"]
    result = planner.search(1, 4)
    assert result.status == "sat"
    assert result.horizon == 3
    assert [attempt["status"] for attempt in result.stats["attempts"]] == ["unsat", "unsat", "sat"]


@pytest.mark.parametrize("backend", ["smc", "smt"])
def test_lasso_has_real_closing_transition_and_periodic_ltl(backend):
    problem = cycle_problem()
    result = Planner(problem).solve(2, mode="lasso", backend=backend)
    assert result.status == "sat", result.stats
    assert result.loop_start == 0
    assert result.states["r"].shape == (3, 1)
    assert result.controls["r"].shape == (2, 1)
    np.testing.assert_allclose(result.states["r"][:, 0], [0, 1, 0], atol=1e-7)
    np.testing.assert_allclose(result.controls["r"][:, 0], [1, -1], atol=1e-7)
    report = validate_plan(problem, result)
    assert report["valid"] and report["ltl_satisfied"]
    assert report["max_loop_residual"] < 1e-7


@pytest.mark.parametrize("backend", ["smc", "smt"])
def test_full_state_lasso_cannot_close_only_position(backend):
    # Position stays at zero, while an unactuated-for-geometry state increases.
    # Position-only periodicity would incorrectly accept every horizon.
    d = Dynamics(np.eye(2), [[0], [1]], (0,), [-1, -10], [1, 10], [1], [1])
    problem = Problem(
        Workspace((Region.box("cell", [-1], [1]),), {}), (Robot("r", d, [0, 0]),), TRUE, separation=0
    )
    finite = Planner(problem).solve(3, backend=backend)
    assert finite.status == "sat"
    np.testing.assert_allclose(finite.states["r"][:, 1], [0, 1, 2])
    assert Planner(problem).solve(3, mode="lasso", backend=backend).status == "unsat"
    forged = PlanResult(
        "sat",
        1,
        "lasso",
        loop_start=0,
        states={"r": np.array([[0.0, 0.0], [0.0, 1.0]])},
        controls={"r": np.array([[1.0]])},
        regions={"r": ["cell"]},
    )
    report = validate_plan(problem, forged)
    assert not report["valid"]
    assert report["max_dynamics_residual"] == 0
    assert report["max_loop_residual"] == 1
    assert any("full-state loop" in error for error in report["errors"])


@pytest.mark.parametrize("backend", ["smc", "smt"])
def test_initial_collision_is_impossible(backend):
    d = integrator_chain(1, 1, input_bound=1, state_bound=2)
    problem = Problem(
        Workspace((Region.box("cell", [-2], [2]),), {}),
        (Robot("a", d, [0]), Robot("b", d, [0])),
        TRUE,
        separation=0.2,
    )
    result = Planner(problem).solve(2, backend=backend)
    assert result.status == "unsat"
    assert not result.states


def test_heterogeneous_state_and_input_dimensions_and_position_indices():
    base = line_problem()
    slow = Dynamics(
        [[-0.5, 0, 0], [0, 1, 0], [0, 0, 1]],
        [[0, 0], [0, 1], [0.5, 0]],
        (2,),
        [-3, -4, -3],
        [3, 4, 3],
        [-1, 0],
        [1, 0],
    )
    problem = Problem(
        base.workspace,
        (base.robots[0], Robot("slow", slow, [2, 3, 0])),
        And(Eventually(Atom("r", "goal")), Eventually(Atom("slow", "goal"))),
        separation=0,
    )
    result = Planner(problem).solve(5)
    assert result.status == "sat", result.stats
    assert result.states["r"].shape == (5, 1)
    assert result.states["slow"].shape == (5, 3)
    assert result.controls["slow"].shape == (4, 2)
    np.testing.assert_allclose(result.states["slow"][:, 0], [2, -1, 0.5, -0.25, 0.125])
    np.testing.assert_allclose(result.states["slow"][:, 1], 3)
    for robot in problem.robots:
        x, u, d = result.states[robot.name], result.controls[robot.name], robot.dynamics
        np.testing.assert_allclose(x[1:], x[:-1] @ d.A.T + u @ d.B.T, atol=1e-7)
    assert validate_plan(problem, result)["valid"]


def test_sampled_separation_and_obstacles_have_independent_checks():
    d = integrator_chain(1, 1, input_bound=2, state_bound=3)
    workspace = Workspace((Region.box("cell", [-3], [3]),), {}, (Region.box("obstacle", [-0.2], [0.2]),))
    problem = Problem(workspace, (Robot("a", d, [-1]), Robot("b", d, [1])), TRUE, separation=0.5)
    result = Planner(problem).solve(2)
    assert result.status == "sat", result.stats
    report = validate_plan(problem, result)
    assert report["valid"]
    assert report["min_sampled_separation"] >= 0.5 - 1e-6
    assert "no inter-sample guarantee" in report["safety_scope"]
    # Corrupt both final states while preserving each linear transition.
    result.states["a"][-1, 0] = 0
    result.states["b"][-1, 0] = 0
    result.controls["a"][0, 0] = 1
    result.controls["b"][0, 0] = -1
    report = validate_plan(problem, result)
    assert not report["valid"]
    assert report["max_dynamics_residual"] == 0
    assert any("inside obstacle" in error for error in report["errors"])
    assert any("sampled separation" in error for error in report["errors"])


@pytest.mark.parametrize(
    "corruption,match",
    [
        ("initial", "wrong initial state"),
        ("nan", "non-finite"),
        ("control", "input bound"),
        ("region", "outside selected region"),
        ("shape", "malformed state/control"),
        ("names", "robot names"),
        ("loop", "Finite trajectory"),
        ("boolean_horizon", "Invalid mode or horizon"),
    ],
)
def test_validator_rejects_corrupted_results(corruption, match):
    problem = line_problem()
    result = Planner(problem).solve(3)
    assert result.status == "sat"
    if corruption == "initial":
        result.states["r"][0, 0] += 0.1
    elif corruption == "nan":
        result.states["r"][-1, 0] = np.nan
    elif corruption == "control":
        result.controls["r"][0, 0] = 100
    elif corruption == "region":
        result.regions["r"][-1] = "start"
    elif corruption == "shape":
        result.controls["r"] = np.zeros((3, 1))
    elif corruption == "names":
        result.states["wrong"] = result.states.pop("r")
    elif corruption == "loop":
        result.loop_start = 0
    elif corruption == "boolean_horizon":
        # Use a genuinely one-state result, so shape checks cannot mask a bool horizon.
        problem = line_problem(TRUE)
        result = Planner(problem).solve(1)
        result.horizon = True
    report = validate_plan(problem, result)
    assert not report["valid"]
    assert any(match in error for error in report["errors"]), report


def test_validator_rejects_temporally_false_but_physically_feasible_trace():
    result = PlanResult(
        "sat",
        3,
        "finite",
        states={"r": np.zeros((3, 1))},
        controls={"r": np.zeros((2, 1))},
        regions={"r": ["start"] * 3},
    )
    assert validate_plan(line_problem(TRUE), result)["valid"]
    report = validate_plan(line_problem(), result)
    assert not report["valid"]
    assert "Temporal specification is false on the returned trace" in report["errors"]


def test_unknown_solver_result_never_becomes_unsat_or_a_trajectory(monkeypatch):
    import satex.planner as planner_module

    monkeypatch.setattr(
        planner_module,
        "solve_smc",
        lambda *_args, **_kwargs: SMCResult("unknown", message="injected numerical failure"),
    )
    result = Planner(line_problem()).search(1, 5)
    assert result.status == "unknown"
    assert len(result.stats["attempts"]) == 1
    assert not result.states
    assert "injected numerical failure" in result.stats["message"]


def test_independent_validation_failure_prevents_sat_status(monkeypatch):
    import satex.validation as validation_module

    monkeypatch.setattr(
        validation_module,
        "validate_plan",
        lambda *_args: {"valid": False, "errors": ["injected independent failure"]},
    )
    result = Planner(line_problem()).solve(3)
    assert result.status == "unknown"
    assert "Independent validation rejected" in result.stats["message"]


def test_lp_unknown_does_not_learn_an_infeasible_conflict(monkeypatch):
    guard = z3.Bool("guard_unknown_regression")
    rows = Rows()
    rows.add({0: 1}, 1)
    oracle = LinearOracle([(-1, 1)], Rows(), [GuardedRows(guard, rows)])
    solver = z3.Solver()
    solver.add(guard)

    def fake_solve(active, deadline=None):
        return (
            LPResult("unknown", message="injected time limit") if active else LPResult("sat", np.array([0.0]))
        )

    monkeypatch.setattr(oracle, "solve", fake_solve)
    result = solve_smc(solver, oracle)
    assert result.status == "unknown"
    assert not result.conflicts
    assert solver.check() == z3.sat


def test_plan_result_json_round_trip(tmp_path):
    problem = cycle_problem()
    result = Planner(problem).solve(2, mode="lasso")
    target = tmp_path / "plan.json"
    result.save(target)
    import json

    recovered = PlanResult.from_dict(json.loads(target.read_text()))
    assert validate_plan(problem, recovered)["valid"]
    np.testing.assert_array_equal(recovered.controls["r"], result.controls["r"])


def test_horizon_one_json_round_trip_preserves_empty_control_dimensions():
    d = integrator_chain(2, 2, input_bound=1, state_bound=3)
    problem = Problem(
        Workspace((Region.box("cell", [-3, -3], [3, 3]),), {}),
        (Robot("r", d, [0, 0, 0, 0]),),
        TRUE,
        separation=0,
    )
    result = Planner(problem).solve(1)
    assert result.status == "sat"
    assert result.controls["r"].shape == (0, 2)
    import json

    recovered = PlanResult.from_dict(json.loads(json.dumps(result.to_dict())))
    assert recovered.controls["r"].shape == (0, 2)
    assert validate_plan(problem, recovered)["valid"]


@pytest.mark.parametrize("malformation", ["none_labels", "unhashable_label", "text_array", "ragged_array"])
def test_validator_returns_invalid_for_malformed_external_values(malformation):
    problem = line_problem()
    result = deepcopy(Planner(problem).solve(3))
    if malformation == "none_labels":
        result.regions["r"] = None
    elif malformation == "unhashable_label":
        result.regions["r"][0] = ["start"]
    elif malformation == "text_array":
        result.states["r"] = np.array([["bad"], ["bad"], ["bad"]])
    elif malformation == "ragged_array":
        result.states["r"] = [[0], [1, 2], [2]]
    report = validate_plan(problem, result)
    assert not report["valid"]
    assert report["errors"]


def test_directed_adjacency_applies_to_lasso_closing_edge():
    base = cycle_problem()
    one_way = Workspace(base.workspace.regions, {"left": ("right",)})
    problem = Problem(one_way, base.robots, base.specification, separation=0)
    assert Planner(problem).solve(2, mode="lasso").status == "unsat"
    finite = Problem(one_way, base.robots, Eventually(Atom("r", "right")), separation=0)
    assert Planner(finite).solve(2).status == "sat"
