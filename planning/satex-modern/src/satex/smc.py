"""Monotone SAT-modulo-linear-convex-programming with verified conflict learning.

Each Boolean guard implies a conjunction of affine inequalities. False guards
disable their constraints; they never imply the negation of those constraints.
The LP engine is SciPy's HiGHS. Only a confirmed infeasible LP may produce a
learned conflict. Time limits/numerical failures remain UNKNOWN.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Mapping, Sequence

import numpy as np
import z3
from scipy.optimize import linprog
from scipy.sparse import coo_matrix, csr_matrix, vstack


@dataclass
class Rows:
    """Sparse rows representing a conjunction of a @ x <= b inequalities."""

    coefficients: list[dict[int, float]] = field(default_factory=list)
    rhs: list[float] = field(default_factory=list)

    def add(self, coefficients: Mapping[int, float], rhs: float) -> None:
        if not np.isfinite(rhs) or any(not np.isfinite(v) for v in coefficients.values()):
            raise ValueError("Linear coefficients and right-hand sides must be finite")
        self.coefficients.append({int(k): float(v) for k, v in coefficients.items() if v != 0})
        self.rhs.append(float(rhs))

    def equal(self, coefficients: Mapping[int, float], rhs: float) -> None:
        self.add(coefficients, rhs)
        self.add({k: -v for k, v in coefficients.items()}, -rhs)

    def matrix(self, n: int) -> csr_matrix:
        ri, ci, data = [], [], []
        for i, row in enumerate(self.coefficients):
            for j, value in row.items():
                if not 0 <= j < n:
                    raise ValueError("Variable index is out of bounds")
                ri.append(i)
                ci.append(j)
                data.append(value)
        return coo_matrix((data, (ri, ci)), shape=(len(self.rhs), n)).tocsr()


@dataclass
class GuardedRows:
    guard: z3.BoolRef
    rows: Rows
    label: str = ""


@dataclass
class LPResult:
    status: str
    values: np.ndarray | None = None
    objective: float | None = None
    message: str = ""


@dataclass
class SMCResult:
    status: str
    values: np.ndarray | None = None
    model: z3.ModelRef | None = None
    iterations: int = 0
    lp_calls: int = 0
    conflicts: list[list[str]] = field(default_factory=list)
    elapsed: float = 0.0
    message: str = ""


class LinearOracle:
    def __init__(
        self,
        bounds: Sequence[tuple[float | None, float | None]],
        hard: Rows,
        groups: Sequence[GuardedRows],
        objective: np.ndarray | None = None,
    ):
        self.bounds = list(bounds)
        self.n = len(bounds)
        self.hard = hard
        self.groups = list(groups)
        self.objective = np.zeros(self.n) if objective is None else np.asarray(objective, dtype=float)
        if self.objective.shape != (self.n,) or not np.isfinite(self.objective).all():
            raise ValueError("Objective must be a finite vector with one coefficient per variable")
        self.matrices = [hard.matrix(self.n)] + [g.rows.matrix(self.n) for g in groups]
        self.rhs = [np.asarray(hard.rhs)] + [np.asarray(g.rows.rhs) for g in groups]
        self.calls = 0
        self.cache: dict[tuple[int, ...], LPResult] = {}

    def solve(self, active: Sequence[int], deadline: float | None = None) -> LPResult:
        key = tuple(sorted(active))
        if key in self.cache:
            return self.cache[key]
        remaining = None if deadline is None else deadline - time.monotonic()
        if remaining is not None and remaining <= 0:
            return LPResult("unknown", message="Time limit reached")
        indices = [0] + [i + 1 for i in key]
        matrix = vstack([self.matrices[i] for i in indices], format="csr")
        rhs = np.concatenate([self.rhs[i] for i in indices])
        options = {"primal_feasibility_tolerance": 1e-8, "dual_feasibility_tolerance": 1e-8}
        if remaining is not None:
            options["time_limit"] = max(0.001, remaining)
        self.calls += 1
        try:
            res = linprog(
                self.objective,
                A_ub=matrix if len(rhs) else None,
                b_ub=rhs if len(rhs) else None,
                bounds=self.bounds,
                method="highs",
                options=options,
            )
        except (ValueError, RuntimeError) as exc:
            return LPResult("unknown", message=str(exc))
        if res.status == 0:
            # Never accept a nominal optimum whose primal residual exceeds tolerance.
            violation = max(float(np.max(matrix @ res.x - rhs, initial=0.0)), 0.0)
            for val, (lower, upper) in zip(res.x, self.bounds):
                if lower is not None:
                    violation = max(violation, lower - val)
                if upper is not None:
                    violation = max(violation, val - upper)
            result = (
                LPResult("sat", res.x, float(res.fun))
                if violation <= 1e-6
                else LPResult("unknown", message=f"LP residual {violation:g} exceeds tolerance")
            )
        elif res.status == 2:
            result = LPResult("unsat", message=res.message)
        else:
            result = LPResult("unknown", message=res.message)
        if result.status != "unknown":
            self.cache[key] = result
        return result

    def conflict(
        self, active: Sequence[int], strategy: str, deadline: float | None, budget: int = 100
    ) -> list[int]:
        """Return a confirmed infeasible subset; budget limits affect size, not soundness."""
        core = list(active)
        if strategy == "full":
            return core
        if strategy == "prefix":
            lo, hi = 0, len(core)
            while lo < hi:
                mid = (lo + hi) // 2
                result = self.solve(core[:mid], deadline)
                if result.status == "unsat":
                    hi = mid
                elif result.status == "sat":
                    lo = mid + 1
                else:
                    return core
            return core[:hi]
        if strategy != "deletion":
            raise ValueError("Conflict strategy must be deletion, prefix, or full")
        # First remove chunks; then delete single groups. Only UNSAT trials permit removal.
        size = max(1, len(core) // 2)
        while size >= 1 and budget > 0:
            i = 0
            while i < len(core) and budget > 0:
                trial = core[:i] + core[i + size :]
                result = self.solve(trial, deadline)
                budget -= 1
                if result.status == "unsat":
                    core = trial
                elif result.status == "unknown":
                    return core
                else:
                    i += size
            size //= 2
        return core


def solve_smc(
    solver: z3.Solver,
    oracle: LinearOracle,
    *,
    max_iterations: int = 2000,
    time_limit: float = 60.0,
    strategy: str = "deletion",
    conflict_budget: int = 100,
) -> SMCResult:
    if max_iterations < 1 or time_limit <= 0 or conflict_budget < 0:
        raise ValueError("Iteration/time limits must be positive and conflict budget nonnegative")
    if strategy not in {"deletion", "prefix", "full"}:
        raise ValueError("Unknown conflict strategy")
    start = time.monotonic()
    deadline = start + time_limit
    result = SMCResult("unknown")
    base = oracle.solve([], deadline)
    if base.status != "sat":
        result.status, result.message = base.status, "Unconditional dynamics/bounds: " + base.message
    else:
        for iteration in range(1, max_iterations + 1):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                result.message = "Time limit reached"
                break
            solver.set(timeout=max(1, int(1000 * remaining)))
            status = solver.check()
            if status == z3.unsat:
                result.status = "unsat"
                break
            if status == z3.unknown:
                result.message = solver.reason_unknown()
                break
            result.iterations = iteration
            model = solver.model()
            active = [
                i
                for i, g in enumerate(oracle.groups)
                if z3.is_true(model.eval(g.guard, model_completion=True))
            ]
            lp = oracle.solve(active, deadline)
            if lp.status == "sat":
                result.status, result.values, result.model = "sat", lp.values, model
                break
            if lp.status == "unknown":
                result.message = lp.message
                break
            core = oracle.conflict(active, strategy, deadline, conflict_budget)
            result.conflicts.append([oracle.groups[i].label or str(oracle.groups[i].guard) for i in core])
            solver.add(z3.Or([z3.Not(oracle.groups[i].guard) for i in core]))
        else:
            result.message = "Candidate iteration limit reached"
    result.elapsed = time.monotonic() - start
    result.lp_calls = oracle.calls
    return result


def solve_monolithic(solver: z3.Solver, oracle: LinearOracle, *, time_limit: float = 60.0) -> SMCResult:
    """Exact rational SMT baseline for the same bounded feasibility constraints.

    Z3 selects a feasible assignment. HiGHS then minimizes L1 effort for that
    assignment. This is a baseline, not a reproduction of the paper's RRT/EST.
    """
    start = time.monotonic()
    deadline = start + time_limit
    real = [z3.Real(f"continuous_{i}") for i in range(oracle.n)]

    def expr(rows: Rows) -> list[z3.BoolRef]:
        return [
            z3.Sum([z3.RealVal(str(v)) * real[j] for j, v in row.items()]) <= z3.RealVal(str(rhs))
            for row, rhs in zip(rows.coefficients, rows.rhs)
        ]

    solver.add(expr(oracle.hard))
    for x, (lower, upper) in zip(real, oracle.bounds):
        if lower is not None:
            solver.add(x >= z3.RealVal(str(lower)))
        if upper is not None:
            solver.add(x <= z3.RealVal(str(upper)))
    for group in oracle.groups:
        solver.add(z3.Implies(group.guard, z3.And(expr(group.rows))))
    solver.set(timeout=max(1, int(1000 * max(0.001, deadline - time.monotonic()))))
    status = solver.check()
    result = SMCResult("unsat" if status == z3.unsat else "unknown", iterations=1)
    if status == z3.sat:
        model = solver.model()
        active = [
            i for i, g in enumerate(oracle.groups) if z3.is_true(model.eval(g.guard, model_completion=True))
        ]
        lp = oracle.solve(active, deadline)
        result.status, result.values, result.model, result.message = lp.status, lp.values, model, lp.message
    elif status == z3.unknown:
        result.message = solver.reason_unknown()
    result.lp_calls, result.elapsed = oracle.calls, time.monotonic() - start
    return result
