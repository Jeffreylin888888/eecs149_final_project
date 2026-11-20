"""Discrete LTL planning coordinated with continuous affine robot dynamics."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import z3

from .ltl import encode
from .models import Problem
from .smc import GuardedRows, LinearOracle, Rows, solve_monolithic, solve_smc


@dataclass
class PlanResult:
    status: str
    horizon: int
    mode: str
    loop_start: int | None = None
    states: dict[str, np.ndarray] = field(default_factory=dict)
    controls: dict[str, np.ndarray] = field(default_factory=dict)
    regions: dict[str, list[str]] = field(default_factory=dict)
    stats: dict = field(default_factory=dict)
    validation: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "status": self.status,
            "horizon": self.horizon,
            "mode": self.mode,
            "loop_start": self.loop_start,
            "states": {k: v.tolist() for k, v in self.states.items()},
            "controls": {k: v.tolist() for k, v in self.controls.items()},
            "control_dimensions": {k: int(v.shape[1]) for k, v in self.controls.items()},
            "regions": self.regions,
            "stats": self.stats,
            "validation": self.validation,
        }

    def save(self, path: str | Path) -> None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(json.dumps(self.to_dict(), indent=2, allow_nan=False) + "\n")

    @classmethod
    def from_dict(cls, data: dict) -> PlanResult:
        return cls(
            data["status"],
            data["horizon"],
            data["mode"],
            data.get("loop_start"),
            {k: np.asarray(v, dtype=float) for k, v in data.get("states", {}).items()},
            {
                k: (
                    np.asarray(v, dtype=float).reshape(0, data["control_dimensions"][k])
                    if len(v) == 0 and k in data.get("control_dimensions", {})
                    else np.asarray(v, dtype=float)
                )
                for k, v in data.get("controls", {}).items()
            },
            data.get("regions", {}),
            data.get("stats", {}),
            data.get("validation", {}),
        )


class Planner:
    def __init__(self, problem: Problem):
        self.problem = problem

    def _build(self, horizon: int, mode: str):
        problem = self.problem
        total_states = horizon + (mode == "lasso")
        total_inputs = total_states - 1
        solver = z3.Solver()
        solver.set(random_seed=0)
        hard = Rows()
        groups = []
        bounds = []
        objective = []
        state_indices, input_indices = {}, {}

        def variables(shape, lower, upper, cost=0):
            first = len(bounds)
            for _ in range(shape[0]):
                for lo, hi in zip(lower, upper):
                    bounds.append((float(lo), float(hi)))
                    objective.append(float(cost))
            return np.arange(first, len(bounds)).reshape(shape)

        def group(guard, rows, label):
            groups.append(GuardedRows(guard, rows, label))

        region_bool = {}
        for ri, robot in enumerate(problem.robots):
            d = robot.dynamics
            x = variables((total_states, d.state_dimension), d.state_lower, d.state_upper)
            u = variables((total_inputs, d.input_dimension), d.input_lower, d.input_upper)
            effort_max = np.maximum(np.abs(d.input_lower), np.abs(d.input_upper))
            effort = variables((total_inputs, d.input_dimension), np.zeros(d.input_dimension), effort_max, 1)
            state_indices[robot.name], input_indices[robot.name] = x, u
            for j, value in enumerate(robot.initial_state):
                hard.equal({x[0, j]: 1}, value)
            if robot.terminal_state is not None:
                for j, value in enumerate(robot.terminal_state):
                    if value is not None:
                        hard.equal({x[-1, j]: 1}, value)
            for t in range(total_inputs):
                for j in range(d.state_dimension):
                    coeff = {x[t + 1, j]: 1.0}
                    coeff.update({x[t, k]: -d.A[j, k] for k in range(d.state_dimension)})
                    coeff.update({u[t, k]: -d.B[j, k] for k in range(d.input_dimension)})
                    hard.equal(coeff, 0)
                for j in range(d.input_dimension):
                    hard.add({u[t, j]: 1, effort[t, j]: -1}, 0)
                    hard.add({u[t, j]: -1, effort[t, j]: -1}, 0)

            lower = np.array(robot.initial_state, copy=True)
            upper = lower.copy()
            for t in range(horizon):
                choices = []
                for rj, region in enumerate(problem.workspace.regions):
                    b = z3.Bool(f"region_{ri}_{t}_{rj}")
                    region_bool[robot.name, region.name, t] = b
                    choices.append(b)
                    rows = Rows()
                    for a, rhs in zip(region.A, region.b):
                        rows.add({x[t, p]: value for p, value in zip(d.position_indices, a)}, rhs)
                    group(b, rows, f"{robot.name}@{t}:{region.name}")
                    # Sound interval reachability pruning, independent of chosen regions.
                    lo = lower[list(d.position_indices)]
                    hi = upper[list(d.position_indices)]
                    min_value = np.maximum(region.A, 0) @ lo + np.minimum(region.A, 0) @ hi
                    if np.any(min_value > region.b + 1e-9):
                        solver.add(z3.Not(b))
                solver.add(z3.PbEq([(b, 1) for b in choices], 1))
                a_pos, a_neg = np.maximum(d.A, 0), np.minimum(d.A, 0)
                b_pos, b_neg = np.maximum(d.B, 0), np.minimum(d.B, 0)
                lower, upper = (
                    np.maximum(
                        d.state_lower,
                        a_pos @ lower + a_neg @ upper + b_pos @ d.input_lower + b_neg @ d.input_upper,
                    ),
                    np.minimum(
                        d.state_upper,
                        a_pos @ upper + a_neg @ lower + b_pos @ d.input_upper + b_neg @ d.input_lower,
                    ),
                )

        def atom_at(robot, region, t):
            return region_bool[robot, region, t]

        def adjacency(t, next_t):
            clauses = []
            for robot in problem.robots:
                for region in problem.workspace.regions:
                    neighbors = set(problem.workspace.adjacency.get(region.name, ())) | {region.name}
                    clauses.append(
                        z3.Implies(
                            atom_at(robot.name, region.name, t),
                            z3.Or([atom_at(robot.name, n, next_t) for n in neighbors]),
                        )
                    )
            return z3.And(clauses)

        for t in range(horizon - 1):
            solver.add(adjacency(t, t + 1))

        loop_bools = []
        if mode == "finite":
            solver.add(encode(problem.specification, horizon, atom_at, None))
        else:
            for loop in range(horizon):
                selector = z3.Bool(f"loop_{loop}")
                loop_bools.append(selector)
                solver.add(z3.Implies(selector, adjacency(horizon - 1, loop)))
                solver.add(z3.Implies(selector, encode(problem.specification, horizon, atom_at, loop)))
                rows = Rows()
                for robot in problem.robots:
                    x = state_indices[robot.name]
                    for j in range(robot.dynamics.state_dimension):
                        rows.equal({x[horizon, j]: 1, x[loop, j]: -1}, 0)
                group(selector, rows, f"full-state-loop:{loop}")
            solver.add(z3.PbEq([(b, 1) for b in loop_bools], 1))

        # Explicit obstacle exclusion also protects custom inputs whose free cells
        # accidentally overlap obstacles. Obstacle boundaries may be touched.
        for ri, robot in enumerate(problem.robots):
            x = state_indices[robot.name]
            for t in range(horizon):
                for oi, obstacle in enumerate(problem.workspace.obstacles):
                    sides = []
                    for si, (a, rhs) in enumerate(zip(obstacle.A, obstacle.b)):
                        b = z3.Bool(f"obstacle_{ri}_{t}_{oi}_{si}")
                        sides.append(b)
                        rows = Rows()
                        rows.add({x[t, p]: -v for p, v in zip(robot.dynamics.position_indices, a)}, -rhs)
                        group(b, rows, f"obstacle:{robot.name}@{t}:{obstacle.name}:face{si}")
                    solver.add(z3.PbEq([(b, 1) for b in sides], 1))

        if problem.separation > 0:
            for i, first in enumerate(problem.robots):
                for j, second in enumerate(problem.robots[i + 1 :], start=i + 1):
                    xi, xj = state_indices[first.name], state_indices[second.name]
                    for t in range(horizon):
                        sides = []
                        for k, (pi, pj) in enumerate(
                            zip(first.dynamics.position_indices, second.dynamics.position_indices)
                        ):
                            for sign in (-1, 1):
                                b = z3.Bool(f"separation_{i}_{j}_{t}_{k}_{sign}")
                                sides.append(b)
                                rows = Rows()
                                rows.add({xi[t, pi]: -sign, xj[t, pj]: sign}, -problem.separation)
                                group(b, rows, f"separation:{first.name}:{second.name}@{t}:{k}:{sign}")
                        solver.add(z3.PbEq([(b, 1) for b in sides], 1))
        oracle = LinearOracle(bounds, hard, groups, np.asarray(objective))
        return solver, oracle, state_indices, input_indices, region_bool, loop_bools

    def solve(
        self,
        horizon: int,
        *,
        mode: str = "finite",
        backend: str = "smc",
        strategy: str = "deletion",
        time_limit: float = 60.0,
        max_iterations: int = 2000,
        conflict_budget: int = 100,
    ) -> PlanResult:
        if isinstance(horizon, bool) or not isinstance(horizon, int) or horizon < 1:
            raise ValueError("Horizon must be a positive integer number of unique states")
        if mode not in {"finite", "lasso"} or backend not in {"smc", "smt"}:
            raise ValueError("Mode must be finite/lasso and backend must be smc/smt")
        if not np.isfinite(time_limit) or time_limit <= 0:
            raise ValueError("Time limit must be finite and positive")
        if type(max_iterations) is not int or max_iterations < 1:
            raise ValueError("max_iterations must be a positive integer")
        if type(conflict_budget) is not int or conflict_budget < 0:
            raise ValueError("conflict_budget must be a nonnegative integer")
        if strategy not in {"deletion", "prefix", "full"}:
            raise ValueError("Unknown conflict strategy")
        start = time.monotonic()
        solver, oracle, xi, ui, regions, loops = self._build(horizon, mode)
        build_seconds = time.monotonic() - start
        remaining = time_limit - build_seconds
        if remaining <= 0:
            return PlanResult(
                "unknown",
                horizon,
                mode,
                stats={"message": "Time limit reached during encoding", "elapsed": build_seconds},
            )
        raw = (
            solve_smc(
                solver,
                oracle,
                time_limit=remaining,
                max_iterations=max_iterations,
                strategy=strategy,
                conflict_budget=conflict_budget,
            )
            if backend == "smc"
            else solve_monolithic(solver, oracle, time_limit=remaining)
        )
        result = PlanResult(
            raw.status,
            horizon,
            mode,
            stats={
                "backend": backend,
                "strategy": strategy,
                "iterations": raw.iterations,
                "lp_calls": raw.lp_calls,
                "conflicts": len(raw.conflicts),
                "conflict_sizes": [len(c) for c in raw.conflicts],
                "build_seconds": build_seconds,
                "elapsed": time.monotonic() - start,
                "message": raw.message,
                "continuous_variables": oracle.n,
                "guarded_groups": len(oracle.groups),
            },
        )
        if raw.status == "sat":
            result.loop_start = next(
                (
                    loop
                    for loop, b in enumerate(loops)
                    if z3.is_true(raw.model.eval(b, model_completion=True))
                ),
                None,
            )
            result.states = {name: raw.values[index] for name, index in xi.items()}
            result.controls = {name: raw.values[index] for name, index in ui.items()}
            result.regions = {
                robot.name: [
                    next(
                        region.name
                        for region in self.problem.workspace.regions
                        if z3.is_true(
                            raw.model.eval(regions[robot.name, region.name, t], model_completion=True)
                        )
                    )
                    for t in range(horizon)
                ]
                for robot in self.problem.robots
            }
            result.stats["control_effort_l1"] = float(sum(np.abs(u).sum() for u in result.controls.values()))
            from .validation import validate_plan

            result.validation = validate_plan(self.problem, result)
            if not result.validation["valid"]:
                result.status = "unknown"
                result.stats["message"] = "Independent validation rejected solver output"
        return result

    def search(
        self, min_horizon: int = 2, max_horizon: int = 20, *, time_limit: float = 60.0, **kwargs
    ) -> PlanResult:
        if (
            type(min_horizon) is not int
            or type(max_horizon) is not int
            or min_horizon < 1
            or max_horizon < min_horizon
        ):
            raise ValueError("Require 1 <= min_horizon <= max_horizon")
        if not np.isfinite(time_limit) or time_limit <= 0:
            raise ValueError("Time limit must be finite and positive")
        start = time.monotonic()
        attempts = []
        for horizon in range(min_horizon, max_horizon + 1):
            remaining = time_limit - (time.monotonic() - start)
            if remaining <= 0:
                result = PlanResult(
                    "unknown",
                    horizon,
                    kwargs.get("mode", "finite"),
                    stats={"message": "Search time limit reached"},
                )
                break
            result = self.solve(horizon, time_limit=remaining, **kwargs)
            attempts.append({"horizon": horizon, "status": result.status, **result.stats})
            if result.status != "unsat":
                break  # Unknown is never a proof of infeasibility or minimum horizon.
        result.stats["attempts"] = attempts
        result.stats["search_elapsed"] = time.monotonic() - start
        result.stats["searched_horizons"] = [min_horizon, result.horizon]
        return result
