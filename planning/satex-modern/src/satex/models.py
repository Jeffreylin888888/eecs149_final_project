"""Validated, serializable models for discrete-time multi-robot planning.

All geometric sets use ``A @ position <= b``. A robot's state may contain
velocity, acceleration, or other components; ``position_indices`` identifies
the components used for geometry. Bounds are finite to keep numerical
feasibility checks explicit and reproducible.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from math import factorial
from numbers import Real
from pathlib import Path
from typing import Any, Mapping

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.optimize import linprog

from .ltl import Formula, atoms
from .ltl import from_dict as formula_from_dict


def _name(value: str, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{label} must be a nonempty string")
    return value


def _array(value: ArrayLike, dimensions: int, label: str) -> NDArray[np.float64]:
    try:
        raw = np.asarray(value)
        if raw.dtype.kind not in "iuf":
            raise ValueError(f"{label} must contain real numbers, not strings, booleans, or complex values")
        result = np.array(value, dtype=float, copy=True)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label} must be a numeric array") from exc
    if result.ndim != dimensions:
        raise ValueError(f"{label} must have {dimensions} dimensions")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{label} must contain only finite numbers")
    result.setflags(write=False)
    return result


def _nonnegative(value: float, label: str) -> float:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise ValueError(f"{label} must be a finite nonnegative number")
    result = float(value)
    if not np.isfinite(result) or result < 0:
        raise ValueError(f"{label} must be a finite nonnegative number")
    return result


def _sequence(value: Any, label: str) -> tuple:
    if isinstance(value, (str, bytes, Mapping)):
        raise ValueError(f"{label} must be a sequence")
    try:
        return tuple(value)
    except TypeError as exc:
        raise ValueError(f"{label} must be a sequence") from exc


def _object(data: Any, label: str, required: set[str], optional: set[str] = frozenset()) -> Mapping:
    if not isinstance(data, Mapping):
        raise ValueError(f"{label} must be a JSON object")
    missing, extra = required - data.keys(), data.keys() - required - optional
    if missing:
        raise ValueError(f"{label} is missing fields: {sorted(missing)}")
    if extra:
        raise ValueError(f"{label} has unknown fields: {sorted(extra, key=str)}")
    return data


@dataclass(frozen=True, eq=False)
class Region:
    """A named closed convex polyhedron in workspace coordinates."""

    name: str
    A: NDArray[np.float64]
    b: NDArray[np.float64]

    def __post_init__(self) -> None:
        _name(self.name, "region name")
        A = _array(self.A, 2, "region A")
        b = _array(self.b, 1, "region b")
        if A.shape[0] == 0 or A.shape[1] == 0 or b.shape != (A.shape[0],):
            raise ValueError("region requires a nonempty A and one b per row")
        if np.any(np.all(A == 0, axis=1)):
            raise ValueError("region facets must have nonzero normal vectors")
        feasible = linprog(
            np.zeros(A.shape[1]), A_ub=A, b_ub=b, bounds=[(None, None)] * A.shape[1], method="highs"
        )
        if feasible.status == 2:
            raise ValueError(f"region {self.name!r} is infeasible")
        if not feasible.success:
            raise ValueError(f"could not verify region {self.name!r} feasibility: {feasible.message}")
        object.__setattr__(self, "A", A)
        object.__setattr__(self, "b", b)

    @classmethod
    def box(cls, name: str, lower: ArrayLike, upper: ArrayLike) -> Region:
        lower_array = _array(lower, 1, "box lower bound")
        upper_array = _array(upper, 1, "box upper bound")
        if lower_array.size == 0 or lower_array.shape != upper_array.shape:
            raise ValueError("box bounds must be nonempty vectors of equal length")
        if np.any(lower_array > upper_array):
            raise ValueError("box lower bound exceeds upper bound")
        identity = np.eye(lower_array.size)
        return cls(name, np.vstack((-identity, identity)), np.concatenate((-lower_array, upper_array)))

    @property
    def dimension(self) -> int:
        return self.A.shape[1]

    def contains(self, point: ArrayLike, tolerance: float = 1e-7) -> bool:
        point_array = _array(point, 1, "point")
        tolerance = _nonnegative(tolerance, "tolerance")
        if point_array.shape != (self.dimension,):
            raise ValueError("point dimension does not match region")
        return bool(np.all(self.A @ point_array <= self.b + tolerance))

    def to_dict(self) -> dict[str, Any]:
        return {"name": self.name, "A": self.A.tolist(), "b": self.b.tolist()}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> Region:
        data = _object(data, "region", {"name", "A", "b"})
        return cls(data["name"], data["A"], data["b"])


@dataclass(frozen=True, eq=False)
class Workspace:
    """Free region cover, allowed transitions, and optional forbidden sets.

    Adjacency is directed. Self transitions are always added. Missing keys
    therefore mean a region can only remain in itself. Obstacle names must be
    distinct from free-region names; temporal atoms reference free regions.
    """

    regions: tuple[Region, ...]
    adjacency: dict[str, tuple[str, ...]]
    obstacles: tuple[Region, ...] = ()

    def __post_init__(self) -> None:
        regions, obstacles = _sequence(self.regions, "regions"), _sequence(self.obstacles, "obstacles")
        if not regions:
            raise ValueError("workspace must have at least one free region")
        if any(not isinstance(region, Region) for region in regions + obstacles):
            raise ValueError("workspace entries must be Region objects")
        all_names = [region.name for region in regions + obstacles]
        if len(all_names) != len(set(all_names)):
            raise ValueError("workspace region and obstacle names must be unique")
        dimension = regions[0].dimension
        if any(region.dimension != dimension for region in regions + obstacles):
            raise ValueError("all workspace regions must have the same dimension")
        names = {region.name for region in regions}
        if not isinstance(self.adjacency, Mapping):
            raise ValueError("adjacency must map region names to neighbors")
        unknown = set(self.adjacency) - names
        if unknown:
            raise ValueError(f"unknown adjacency source(s): {sorted(unknown)}")
        adjacency: dict[str, tuple[str, ...]] = {}
        for region in regions:
            neighbors = self.adjacency.get(region.name, ())
            neighbors = _sequence(neighbors, "adjacency neighbors")
            if any(not isinstance(neighbor, str) for neighbor in neighbors):
                raise ValueError("adjacency neighbors must be region names")
            unknown = set(neighbors) - names
            if unknown:
                raise ValueError(f"unknown adjacency destination(s): {sorted(unknown)}")
            adjacency[region.name] = tuple(dict.fromkeys((region.name, *neighbors)))
        object.__setattr__(self, "regions", regions)
        object.__setattr__(self, "obstacles", obstacles)
        object.__setattr__(self, "adjacency", adjacency)

    @property
    def dimension(self) -> int:
        return self.regions[0].dimension

    @property
    def region_map(self) -> dict[str, Region]:
        return {region.name: region for region in self.regions}

    def to_dict(self) -> dict[str, Any]:
        return {
            "regions": [region.to_dict() for region in self.regions],
            "adjacency": {key: list(value) for key, value in self.adjacency.items()},
            "obstacles": [region.to_dict() for region in self.obstacles],
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> Workspace:
        data = _object(data, "workspace", {"regions"}, {"adjacency", "obstacles"})
        return cls(
            tuple(Region.from_dict(region) for region in _sequence(data["regions"], "regions")),
            data.get("adjacency", {}),
            tuple(Region.from_dict(region) for region in _sequence(data.get("obstacles", ()), "obstacles")),
        )


@dataclass(frozen=True, eq=False)
class Dynamics:
    """Discrete linear dynamics ``state[t+1] = A state[t] + B input[t]``."""

    A: NDArray[np.float64]
    B: NDArray[np.float64]
    position_indices: tuple[int, ...]
    state_lower: NDArray[np.float64]
    state_upper: NDArray[np.float64]
    input_lower: NDArray[np.float64]
    input_upper: NDArray[np.float64]

    def __post_init__(self) -> None:
        A, B = _array(self.A, 2, "dynamics A"), _array(self.B, 2, "dynamics B")
        if A.shape[0] == 0 or A.shape[0] != A.shape[1]:
            raise ValueError("dynamics A must be a nonempty square matrix")
        if B.shape[0] != A.shape[0] or B.shape[1] == 0:
            raise ValueError("dynamics B must have one row per state and at least one input")
        indices = _sequence(self.position_indices, "position_indices")
        if not indices or any(
            isinstance(i, (bool, np.bool_)) or not isinstance(i, (int, np.integer)) for i in indices
        ):
            raise ValueError("position_indices must be nonempty integer indices")
        if len(indices) != len(set(indices)) or min(indices) < 0 or max(indices) >= A.shape[0]:
            raise ValueError("position_indices must be unique valid state indices")
        indices = tuple(int(index) for index in indices)
        bounds = {
            key: _array(getattr(self, key), 1, key)
            for key in ("state_lower", "state_upper", "input_lower", "input_upper")
        }
        for prefix, size in (("state", A.shape[0]), ("input", B.shape[1])):
            lower, upper = bounds[f"{prefix}_lower"], bounds[f"{prefix}_upper"]
            if lower.shape != (size,) or upper.shape != (size,):
                raise ValueError(f"{prefix} bounds must have length {size}")
            if np.any(lower > upper):
                raise ValueError(f"{prefix} lower bound exceeds upper bound")
        object.__setattr__(self, "A", A)
        object.__setattr__(self, "B", B)
        object.__setattr__(self, "position_indices", indices)
        for key, value in bounds.items():
            object.__setattr__(self, key, value)

    @property
    def state_dimension(self) -> int:
        return self.A.shape[0]

    @property
    def input_dimension(self) -> int:
        return self.B.shape[1]

    @property
    def workspace_dimension(self) -> int:
        return len(self.position_indices)

    def to_dict(self) -> dict[str, Any]:
        return {
            "A": self.A.tolist(),
            "B": self.B.tolist(),
            "position_indices": list(self.position_indices),
            "state_lower": self.state_lower.tolist(),
            "state_upper": self.state_upper.tolist(),
            "input_lower": self.input_lower.tolist(),
            "input_upper": self.input_upper.tolist(),
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> Dynamics:
        data = _object(
            data,
            "dynamics",
            {"A", "B", "position_indices", "state_lower", "state_upper", "input_lower", "input_upper"},
        )
        return cls(
            **{
                key: data[key]
                for key in (
                    "A",
                    "B",
                    "position_indices",
                    "state_lower",
                    "state_upper",
                    "input_lower",
                    "input_upper",
                )
            }
        )


@dataclass(frozen=True, eq=False)
class Robot:
    """A robot with an optional componentwise final-state constraint.

    ``terminal_state=None`` leaves the final state unconstrained. Within a
    supplied tuple, each ``None`` leaves just that component unconstrained.
    For a double integrator, ``(None, 0, None, 0)`` requires arrival at rest.
    """

    name: str
    dynamics: Dynamics
    initial_state: NDArray[np.float64]
    terminal_state: tuple[float | None, ...] | None = None

    def __post_init__(self) -> None:
        _name(self.name, "robot name")
        if not isinstance(self.dynamics, Dynamics):
            raise ValueError("robot dynamics must be a Dynamics object")
        state = _array(self.initial_state, 1, "initial state")
        if state.shape != (self.dynamics.state_dimension,):
            raise ValueError("initial state dimension does not match dynamics")
        if np.any(state < self.dynamics.state_lower) or np.any(state > self.dynamics.state_upper):
            raise ValueError("initial state is outside state bounds")
        object.__setattr__(self, "initial_state", state)
        if self.terminal_state is not None:
            mask = _sequence(self.terminal_state, "terminal_state")
            if len(mask) != self.dynamics.state_dimension:
                raise ValueError("terminal state dimension does not match dynamics")
            normalized = []
            for index, value in enumerate(mask):
                if value is None:
                    normalized.append(None)
                    continue
                if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
                    raise ValueError("terminal state entries must be finite real numbers or None")
                value = float(value)
                if not np.isfinite(value):
                    raise ValueError("terminal state entries must be finite real numbers or None")
                if not self.dynamics.state_lower[index] <= value <= self.dynamics.state_upper[index]:
                    raise ValueError("terminal state is outside state bounds")
                normalized.append(value)
            object.__setattr__(self, "terminal_state", tuple(normalized))

    @property
    def initial_position(self) -> NDArray[np.float64]:
        return self.initial_state[list(self.dynamics.position_indices)]

    def to_dict(self) -> dict[str, Any]:
        result = {
            "name": self.name,
            "dynamics": self.dynamics.to_dict(),
            "initial_state": self.initial_state.tolist(),
        }
        if self.terminal_state is not None:
            result["terminal_state"] = list(self.terminal_state)
        return result

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> Robot:
        data = _object(data, "robot", {"name", "dynamics", "initial_state"}, {"terminal_state"})
        return cls(
            data["name"],
            Dynamics.from_dict(data["dynamics"]),
            data["initial_state"],
            data.get("terminal_state"),
        )


@dataclass(frozen=True, eq=False)
class Problem:
    workspace: Workspace
    robots: tuple[Robot, ...]
    specification: Formula
    separation: float = 0.2
    name: str = "motion-planning"

    def __post_init__(self) -> None:
        _name(self.name, "problem name")
        if not isinstance(self.workspace, Workspace):
            raise ValueError("workspace must be a Workspace object")
        robots = _sequence(self.robots, "robots")
        if not robots or any(not isinstance(robot, Robot) for robot in robots):
            raise ValueError("problem requires at least one Robot")
        robot_names = {robot.name for robot in robots}
        if len(robot_names) != len(robots):
            raise ValueError("robot names must be unique")
        for robot in robots:
            if robot.dynamics.workspace_dimension != self.workspace.dimension:
                raise ValueError(f"robot {robot.name!r} position dimension does not match workspace")
            if not any(region.contains(robot.initial_position) for region in self.workspace.regions):
                raise ValueError(f"robot {robot.name!r} initial position is outside the free workspace")
            # Match trajectory semantics: strict obstacle interiors are excluded;
            # contact with a boundary is permitted within numerical tolerance.
            if any(
                np.all(obstacle.A @ robot.initial_position < obstacle.b - 1e-7)
                for obstacle in self.workspace.obstacles
            ):
                raise ValueError(f"robot {robot.name!r} initial position lies in an obstacle")
        if not isinstance(self.specification, Formula):
            raise ValueError("specification must be an LTL Formula")
        region_names = set(self.workspace.region_map)
        for robot_name, region_name in atoms(self.specification):
            if robot_name not in robot_names:
                raise ValueError(f"specification references unknown robot {robot_name!r}")
            if region_name not in region_names:
                raise ValueError(f"specification references unknown free region {region_name!r}")
        pending = [self.specification]
        while pending:
            formula = pending.pop()
            if formula.op == "count" and formula.region not in region_names:
                raise ValueError(f"specification references unknown free region {formula.region!r}")
            pending.extend(formula.args)
        object.__setattr__(self, "robots", robots)
        object.__setattr__(self, "separation", _nonnegative(self.separation, "separation"))

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "name": self.name,
            "workspace": self.workspace.to_dict(),
            "robots": [robot.to_dict() for robot in self.robots],
            "specification": self.specification.to_dict(),
            "separation": self.separation,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> Problem:
        data = _object(
            data,
            "problem",
            {"workspace", "robots", "specification"},
            {"schema_version", "separation", "name"},
        )
        version = data.get("schema_version", 1)
        if type(version) is not int or version != 1:
            raise ValueError("unsupported problem schema_version")
        return cls(
            Workspace.from_dict(data["workspace"]),
            tuple(Robot.from_dict(robot) for robot in _sequence(data["robots"], "robots")),
            formula_from_dict(data["specification"]),
            separation=data.get("separation", 0.2),
            name=data.get("name", "motion-planning"),
        )

    def save(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict(), indent=2, allow_nan=False) + "\n", encoding="utf-8")

    @classmethod
    def load(cls, path: str | Path) -> Problem:
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))


def integrator_chain(
    dimensions: int = 2, order: int = 2, dt: float = 1.0, input_bound: float = 1.0, state_bound: float = 100.0
) -> Dynamics:
    """Exact zero-order-hold discretization of independent derivative chains.

    The axis-major state layout is ``[x, x_dot, ..., y, y_dot, ...]``.
    For order two, the control is acceleration and ``B = [dt**2/2, dt]``
    per axis. Order three uses jerk control.
    """
    for value, label in ((dimensions, "dimensions"), (order, "order")):
        if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)) or value < 1:
            raise ValueError(f"{label} must be a positive integer")
    dt = _nonnegative(dt, "dt")
    if dt == 0:
        raise ValueError("dt must be positive")
    input_bound, state_bound = (
        _nonnegative(input_bound, "input_bound"),
        _nonnegative(state_bound, "state_bound"),
    )
    block_A = np.zeros((order, order))
    block_B = np.zeros((order, 1))
    for row in range(order):
        for column in range(row, order):
            block_A[row, column] = dt ** (column - row) / factorial(column - row)
        block_B[row, 0] = dt ** (order - row) / factorial(order - row)
    A = np.kron(np.eye(dimensions), block_A)
    B = np.kron(np.eye(dimensions), block_B)
    return Dynamics(
        A,
        B,
        tuple(axis * order for axis in range(dimensions)),
        np.full(dimensions * order, -state_bound),
        np.full(dimensions * order, state_bound),
        np.full(dimensions, -input_bound),
        np.full(dimensions, input_bound),
    )
