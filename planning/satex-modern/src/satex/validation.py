"""Independent numerical/temporal validation, without reading solver assertions."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from itertools import combinations

import numpy as np

from .ltl import evaluate


def validate_plan(problem, result, tolerance: float = 1e-6) -> dict:
    """Check the sampled discrete-time contract and full-state periodic closure.

    Region labels are the selected one-hot partition labels. A shared boundary
    may belong to either adjacent cell, consistently with the planner encoding.
    Obstacle interiors are excluded; boundary contact is permitted. This checks
    neither swept-volume safety nor execution robustness on a physical robot.
    """
    if not np.isfinite(tolerance) or tolerance <= 0:
        raise ValueError("Validation tolerance must be finite and positive")
    errors = []
    report = {
        "valid": False,
        "errors": errors,
        "tolerance": tolerance,
        "max_dynamics_residual": 0.0,
        "max_loop_residual": 0.0,
        "min_sampled_separation": None,
        "ltl_satisfied": False,
        "safety_scope": "sampled discrete-time states; no inter-sample guarantee",
    }
    if result.status != "sat":
        errors.append("Result is not a SAT trajectory")
        return report
    if result.mode not in {"finite", "lasso"} or type(result.horizon) is not int or result.horizon < 1:
        errors.append("Invalid mode or horizon")
        return report
    h = result.horizon
    lasso = result.mode == "lasso"
    state_count = h + int(lasso)
    if lasso and (
        isinstance(result.loop_start, bool)
        or not isinstance(result.loop_start, int)
        or not 0 <= result.loop_start < h
    ):
        errors.append("Invalid lasso start")
    if not lasso and result.loop_start is not None:
        errors.append("Finite trajectory must not contain a loop start")
    names = {r.name for r in problem.robots}
    if not all(isinstance(value, Mapping) for value in (result.states, result.controls, result.regions)):
        errors.append("States, controls, and regions must be mappings")
        return report
    if set(result.states) != names or set(result.controls) != names or set(result.regions) != names:
        errors.append("Trajectory robot names do not match problem")
        return report
    region_map = {r.name: r for r in problem.workspace.regions}
    positions = {}
    for robot in problem.robots:
        d = robot.dynamics
        labels = result.regions[robot.name]
        try:
            x = np.asarray(result.states[robot.name], dtype=float)
            u = np.asarray(result.controls[robot.name], dtype=float)
        except (ValueError, TypeError):
            errors.append(f"{robot.name}: state/control arrays must be numeric and rectangular")
            continue
        if x.shape != (state_count, d.state_dimension) or u.shape != (state_count - 1, d.input_dimension):
            errors.append(f"{robot.name}: malformed state/control array")
            continue
        if not np.isfinite(x).all() or not np.isfinite(u).all():
            errors.append(f"{robot.name}: non-finite trajectory")
            continue
        if (
            not isinstance(labels, Sequence)
            or isinstance(labels, (str, bytes))
            or len(labels) != h
            or any(not isinstance(label, str) or label not in region_map for label in labels)
        ):
            errors.append(f"{robot.name}: malformed region labels")
            continue
        if np.max(np.abs(x[0] - robot.initial_state), initial=0) > tolerance:
            errors.append(f"{robot.name}: wrong initial state")
        if robot.terminal_state is not None:
            if any(
                value is not None and abs(x[-1, j] - value) > tolerance
                for j, value in enumerate(robot.terminal_state)
            ):
                errors.append(f"{robot.name}: terminal state constraint violation")
        if np.any(x < d.state_lower - tolerance) or np.any(x > d.state_upper + tolerance):
            errors.append(f"{robot.name}: state bound violation")
        if np.any(u < d.input_lower - tolerance) or np.any(u > d.input_upper + tolerance):
            errors.append(f"{robot.name}: input bound violation")
        residual = float(np.max(np.abs(x[1:] - x[:-1] @ d.A.T - u @ d.B.T), initial=0))
        report["max_dynamics_residual"] = max(report["max_dynamics_residual"], residual)
        if residual > tolerance:
            errors.append(f"{robot.name}: dynamics residual {residual:g}")
        pos = x[:, list(d.position_indices)]
        positions[robot.name] = pos
        for t, label in enumerate(labels):
            if not region_map[label].contains(pos[t], tolerance):
                errors.append(f"{robot.name}@{t}: outside selected region {label}")
            if t and label not in (set(problem.workspace.adjacency[labels[t - 1]]) | {labels[t - 1]}):
                errors.append(f"{robot.name}@{t}: non-adjacent region transition")
        for t, point in enumerate(pos):
            for obstacle in problem.workspace.obstacles:
                if np.all(obstacle.A @ point < obstacle.b - tolerance):
                    errors.append(f"{robot.name}@{t}: inside obstacle {obstacle.name}")
        if lasso and isinstance(result.loop_start, int) and 0 <= result.loop_start < h:
            loop_error = float(np.max(np.abs(x[-1] - x[result.loop_start]), initial=0))
            report["max_loop_residual"] = max(report["max_loop_residual"], loop_error)
            if loop_error > tolerance:
                errors.append(f"{robot.name}: full-state loop does not close")
            if labels[result.loop_start] not in (set(problem.workspace.adjacency[labels[-1]]) | {labels[-1]}):
                errors.append(f"{robot.name}: loop region transition is not adjacent")
    for first, second in combinations(positions, 2):
        distance = np.max(np.abs(positions[first] - positions[second]), axis=1)
        smallest = float(distance.min())
        report["min_sampled_separation"] = (
            smallest
            if report["min_sampled_separation"] is None
            else min(smallest, report["min_sampled_separation"])
        )
        if smallest < problem.separation - tolerance:
            errors.append(f"{first}/{second}: sampled separation {smallest:g} below {problem.separation:g}")
    if not errors:
        trace = [{robot.name: result.regions[robot.name][t] for robot in problem.robots} for t in range(h)]
        report["ltl_satisfied"] = evaluate(problem.specification, trace, result.loop_start)
        if not report["ltl_satisfied"]:
            errors.append("Temporal specification is false on the returned trace")
    report["valid"] = not errors
    return report
