"""Input validation, geometry, exact discretization, and safe JSON round trips."""

import json
from dataclasses import replace

import numpy as np
import pytest
from scipy.linalg import expm

from satex.ltl import Atom, Count
from satex.models import Problem, Region, Robot, Workspace, integrator_chain
from satex.scenarios import (
    legacy_reach_avoid,
    legacy_workspace,
    multi_robot_crossing,
    multi_robot_patrol,
    reach_avoid,
    team_recurrence,
    three_dimensional,
)


def test_box_membership_includes_boundary_and_respects_tolerance():
    region = Region.box("box", [-2, 1], [3, 4])
    assert region.dimension == 2
    assert region.contains([-2, 4])
    assert region.contains([3 + 5e-8, 2])
    assert not region.contains([3 + 5e-6, 2])
    assert not region.contains([0, 0])
    with pytest.raises(ValueError, match="dimension"):
        region.contains([0])


@pytest.mark.parametrize("lower,upper", [([], []), ([0], [1, 2]), ([2], [1]), ([0], [np.inf])])
def test_box_rejects_invalid_bounds(lower, upper):
    with pytest.raises(ValueError):
        Region.box("bad", lower, upper)


@pytest.mark.parametrize(
    "A,b", [([[1, 2]], [1, 2]), ([1, 2], [1]), ([[np.nan]], [1]), ([[1]], [np.inf]), ([], [])]
)
def test_region_rejects_invalid_shapes_and_nonfinite_values(A, b):
    with pytest.raises(ValueError):
        Region("bad", A, b)


def test_region_copies_source_arrays():
    A = np.array([[1.0]])
    region = Region("r", A, [1])
    A[0, 0] = 999
    assert region.A[0, 0] == 1
    with pytest.raises(ValueError):
        region.A[0, 0] = 2


def test_region_rejects_infeasible_sets_and_vacuous_obstacle_facets():
    with pytest.raises(ValueError, match="infeasible"):
        Region("empty", [[1], [-1]], [0, -1])  # x <= 0 and x >= 1.
    with pytest.raises(ValueError, match="nonzero normal"):
        Region("vacuous-face", [[1], [-1], [0]], [1, 1, 0])
    # Unbounded regions remain legitimate convex sets.
    assert Region("halfspace", [[1]], [0]).contains([-100])


@pytest.mark.parametrize("A", [[[True]], [["1"]], [[1 + 2j]]])
def test_region_does_not_silently_coerce_nonnumeric_or_complex_json(A):
    with pytest.raises(ValueError, match="numeric array"):
        Region("bad", A, [1])


def test_workspace_normalizes_self_edges_and_preserves_directed_edges():
    a, b = Region.box("a", [0], [1]), Region.box("b", [1], [2])
    workspace = Workspace((a, b), {"a": ("b", "b")})
    assert workspace.adjacency == {"a": ("a", "b"), "b": ("b",)}
    assert workspace.dimension == 1


def test_workspace_rejects_unknown_names_duplicates_and_dimensions():
    a = Region.box("a", [0], [1])
    b = Region.box("b", [0, 0], [1, 1])
    for regions, adjacency, obstacles in [
        ((a,), {"ghost": ()}, ()),
        ((a,), {"a": ("ghost",)}, ()),
        ((a, a), {}, ()),
        ((a, b), {}, ()),
        ((a,), {}, (a,)),
        ((a,), {}, (b,)),
    ]:
        with pytest.raises(ValueError):
            Workspace(regions, adjacency, obstacles)


@pytest.mark.parametrize("dimensions,order,dt", [(1, 1, 0.2), (2, 2, 0.7), (3, 3, 1.3), (2, 4, 0.4)])
def test_integrator_discretization_matches_augmented_matrix_exponential(dimensions, order, dt):
    dynamics = integrator_chain(dimensions, order, dt)
    continuous_A = np.diag(np.ones(order - 1), k=1)
    continuous_B = np.zeros((order, 1))
    continuous_B[-1] = 1
    augmented = np.block([[continuous_A, continuous_B], [np.zeros((1, order + 1))]])
    exact = expm(augmented * dt)
    np.testing.assert_allclose(dynamics.A, np.kron(np.eye(dimensions), exact[:order, :order]))
    np.testing.assert_allclose(dynamics.B, np.kron(np.eye(dimensions), exact[:order, order:]))
    assert dynamics.position_indices == tuple(range(0, dimensions * order, order))


@pytest.mark.parametrize(
    "options",
    [
        {"dimensions": 0},
        {"order": 0},
        {"order": True},
        {"dt": 0},
        {"dt": np.nan},
        {"input_bound": -1},
        {"state_bound": np.inf},
    ],
)
def test_integrator_rejects_invalid_arguments(options):
    with pytest.raises(ValueError):
        integrator_chain(**options)


def test_dynamics_rejects_invalid_shapes_indices_and_bounds():
    dynamics = integrator_chain()
    for changes in [
        {"A": np.ones((4, 3))},
        {"B": np.ones((3, 2))},
        {"position_indices": (0, 0)},
        {"position_indices": (0, 5)},
        {"position_indices": (0.0, 2.0)},
        {"state_lower": [-1]},
        {"input_upper": [-2, -2]},
        {"input_lower": [np.nan, 0]},
    ]:
        with pytest.raises(ValueError):
            replace(dynamics, **changes)


def test_problem_rejects_unknown_temporal_references_including_count():
    problem = reach_avoid()
    for formula in [
        Atom("ghost", "goal"),
        Atom("r1", "ghost"),
        Count("ghost", ("r1",), minimum=1),
        Count("goal", ("ghost",), minimum=1),
        Count("ghost", (), minimum=0),
    ]:
        with pytest.raises(ValueError, match="unknown"):
            replace(problem, specification=formula)


def test_problem_rejects_initial_positions_outside_workspace_or_in_obstacle():
    robot = Robot("r1", integrator_chain(), [0, 0, 0, 0])
    workspace = Workspace(
        (Region.box("free", [-1, -1], [1, 1]),), {}, (Region.box("blocked", [-0.1, -0.1], [0.1, 0.1]),)
    )
    with pytest.raises(ValueError, match="obstacle"):
        Problem(workspace, (robot,), Atom("r1", "free"))
    with pytest.raises(ValueError, match="outside the free workspace"):
        replace(reach_avoid(), robots=(replace(robot, initial_state=[50, 0, 50, 0]),))


def test_initial_obstacle_boundary_contact_matches_planner_semantics():
    workspace = Workspace(
        (Region.box("free", [-1, -1], [1, 1]),), {}, (Region.box("blocked", [0, 0], [0.5, 0.5]),)
    )
    robot = Robot("r1", integrator_chain(), [0, 0, 0.25, 0])
    problem = Problem(workspace, (robot,), Atom("r1", "free"))
    assert problem.robots[0].initial_position.tolist() == [0, 0.25]


def test_problem_rejects_bad_robot_configuration_and_separation():
    problem = reach_avoid()
    for changes in [
        {"robots": ()},
        {"robots": problem.robots * 2},
        {"separation": -1},
        {"separation": np.nan},
        {"robots": (Robot("r1", integrator_chain(3), [0] * 6),)},
    ]:
        with pytest.raises(ValueError):
            replace(problem, **changes)
    with pytest.raises(ValueError, match="bounds"):
        Robot("r1", integrator_chain(), [101, 0, 0, 0])


def test_terminal_state_mask_round_trip_and_copy():
    mask = [None, 0, None, 0]
    robot = Robot("r1", integrator_chain(), [1, 0, 1, 0], mask)
    mask[1] = 9
    assert robot.terminal_state == (None, 0.0, None, 0.0)
    assert Robot.from_dict(robot.to_dict()).terminal_state == robot.terminal_state
    unconstrained = Robot("r1", integrator_chain(), [1, 0, 1, 0])
    assert "terminal_state" not in unconstrained.to_dict()
    assert Robot.from_dict(unconstrained.to_dict()).terminal_state is None


@pytest.mark.parametrize(
    "mask",
    [
        [],
        [None] * 3,
        [None, 101, None, 0],
        [None, np.nan, None, 0],
        [None, np.inf, None, 0],
        [None, True, None, 0],
        [None, "0", None, 0],
        [None, 1j, None, 0],
    ],
)
def test_terminal_state_rejects_invalid_mask_values_and_bounds(mask):
    with pytest.raises(ValueError, match="terminal state"):
        Robot("r1", integrator_chain(), [1, 0, 1, 0], mask)


@pytest.mark.parametrize(
    "factory",
    [
        reach_avoid,
        multi_robot_patrol,
        three_dimensional,
        legacy_reach_avoid,
        multi_robot_crossing,
        team_recurrence,
    ],
)
def test_scenario_json_round_trip(factory, tmp_path):
    original = factory()
    path = tmp_path / "scenario.json"
    original.save(path)
    recovered = Problem.load(path)
    assert recovered.to_dict() == original.to_dict()
    assert json.loads(path.read_text())["schema_version"] == 1


def test_json_rejects_unsupported_schema_and_nonfinite_state():
    data = reach_avoid().to_dict()
    data["schema_version"] = 999
    with pytest.raises(ValueError, match="schema"):
        Problem.from_dict(data)
    data["schema_version"] = 1
    data["robots"][0]["initial_state"][0] = float("nan")
    with pytest.raises(ValueError, match="finite"):
        Problem.from_dict(data)


def test_json_validates_structure_and_prevents_alias_mutation():
    data = reach_avoid().to_dict()
    original = Problem.from_dict(data)
    data["robots"][0]["initial_state"][0] = 99
    data["workspace"]["adjacency"]["start"].append("goal")
    assert original.robots[0].initial_state[0] == 1
    assert "goal" not in original.workspace.adjacency["start"]
    for invalid in [
        [],
        {"workspace": {}},
        {**original.to_dict(), "typo": 1},
        {**original.to_dict(), "robots": "r1"},
        {**original.to_dict(), "schema_version": True},
        {**original.to_dict(), "separation": "0.5"},
    ]:
        with pytest.raises(ValueError):
            Problem.from_dict(invalid)


def test_legacy_workspace_matches_source_geometry_and_free_adjacency():
    workspace = legacy_workspace()
    assert len(workspace.regions) == 33
    assert len(workspace.obstacles) == 12
    assert {region.name for region in workspace.obstacles} == {
        f"cell_{x}_{y}_{z}" for x in (2, 4) for y in (1, 2, 3) for z in (1, 3)
    }
    assert set(workspace.adjacency["cell_1_1_1"]) == {"cell_1_1_1", "cell_1_2_1", "cell_1_1_2"}
    assert workspace.region_map["cell_5_3_3"].contains([5, 3, 3])
    assert workspace.region_map["cell_1_1_1"].contains([0, 0, 0])


def test_legacy_workspace_retains_original_x_obstacle_cutoff():
    workspace = legacy_workspace(33, 2, 2)
    assert "cell_30_1_1" in {region.name for region in workspace.obstacles}
    assert "cell_32_1_1" in workspace.region_map


def test_legacy_example_rejects_an_obstructed_goal():
    with pytest.raises(ValueError, match="far corner is an obstacle"):
        legacy_reach_avoid(x_ticks=5)
