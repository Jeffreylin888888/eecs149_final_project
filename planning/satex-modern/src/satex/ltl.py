"""Linear temporal logic over finite traces and infinite ultimately periodic traces.

``horizon`` always counts *unique* states, indexed 0 through horizon - 1.
With ``loop_start=None`` this module implements LTLf: next is strong (false at
the last state), until must have a witness, and release is the dual of until.
With a loop index, the final state's successor is that index and formulas have
ordinary infinite-trace LTL semantics. In particular, an unfulfilled eventuality
cannot make itself true by circulating around a loop.

Symbolic encoding expands until over the finite set of reachable unique states.
The concrete evaluator instead computes least/greatest Boolean fixed points;
it is useful as an independent check on a solver's result.
"""

from __future__ import annotations

from collections.abc import Callable, Collection, Mapping, Sequence
from dataclasses import dataclass
from functools import lru_cache
from typing import Any

import z3

_ARITIES = {
    "true": 0,
    "false": 0,
    "atom": 0,
    "count": 0,
    "not": 1,
    "next": 1,
    "eventually": 1,
    "always": 1,
    "until": 2,
    "release": 2,
    "implies": 2,
}
_ALIASES = {
    "x": "next",
    "f": "eventually",
    "g": "always",
    "u": "until",
    "r": "release",
    "!": "not",
    "&": "and",
    "|": "or",
    "->": "implies",
}


def _name(value: Any, label: str) -> None:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{label} must be a nonempty string")


@dataclass(frozen=True)
class Formula:
    """An immutable, hashable temporal-logic expression.

    Use the named constructors below, or ``Formula.from_dict`` for JSON input.
    ``&``, ``|``, and ``~`` are Boolean operators on formulas. Python's ``and``
    and ``or`` are intentionally unsupported; they do not build expressions.
    """

    op: str
    args: tuple[Formula, ...] = ()
    robot: str | None = None
    region: str | None = None
    robots: tuple[str, ...] = ()
    minimum: int | None = None
    maximum: int | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.op, str):
            raise ValueError("formula operator must be a string")
        op = _ALIASES.get(self.op.lower(), self.op.lower())
        object.__setattr__(self, "op", op)
        object.__setattr__(self, "args", tuple(self.args))
        if isinstance(self.robots, str):
            raise ValueError("robots must be a sequence of names, not a single string")
        object.__setattr__(self, "robots", tuple(self.robots))
        if op not in _ARITIES and op not in {"and", "or"}:
            raise ValueError(f"unknown formula operator: {op!r}")
        if op in _ARITIES and len(self.args) != _ARITIES[op]:
            raise ValueError(f"{op} requires {_ARITIES[op]} arguments")
        if any(not isinstance(arg, Formula) for arg in self.args):
            raise TypeError("formula arguments must be Formula instances")
        if op == "atom":
            _name(self.robot, "robot")
            _name(self.region, "region")
        elif op == "count":
            _name(self.region, "region")
            for robot in self.robots:
                _name(robot, "robot")
            if len(set(self.robots)) != len(self.robots):
                raise ValueError("count robot names must be unique")
            minimum = 1 if self.minimum is None else self.minimum
            object.__setattr__(self, "minimum", minimum)
            if type(minimum) is not int or minimum < 0:
                raise ValueError("count minimum must be a nonnegative integer")
            if self.maximum is not None:
                if type(self.maximum) is not int or self.maximum < minimum:
                    raise ValueError("count maximum must be an integer >= minimum")

    def __and__(self, other: Formula) -> Formula:
        return And(self, other)

    def __or__(self, other: Formula) -> Formula:
        return Or(self, other)

    def __invert__(self) -> Formula:
        return Not(self)

    def __bool__(self) -> bool:
        raise TypeError("Formula has no Python truth value; use &, |, ~ or evaluate()")

    def to_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {"op": self.op}
        if self.args:
            result["args"] = [arg.to_dict() for arg in self.args]
        if self.op == "atom":
            result.update(robot=self.robot, region=self.region)
        elif self.op == "count":
            result.update(region=self.region, robots=list(self.robots), minimum=self.minimum)
            if self.maximum is not None:
                result["maximum"] = self.maximum
        return result

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> Formula:
        if not isinstance(data, Mapping):
            raise TypeError("formula JSON must be an object")
        allowed = {"op", "args", "robot", "region", "robots", "minimum", "maximum"}
        extra = set(data) - allowed
        if extra:
            raise ValueError(f"unknown formula fields: {sorted(extra)}")
        if "op" not in data:
            raise ValueError("formula JSON requires 'op'")
        return cls(
            op=data["op"],
            args=tuple(cls.from_dict(arg) for arg in data.get("args", ())),
            robot=data.get("robot"),
            region=data.get("region"),
            robots=data.get("robots", ()),
            minimum=data.get("minimum"),
            maximum=data.get("maximum"),
        )


TRUE = Formula("true")
FALSE = Formula("false")


def Atom(robot: str, region: str) -> Formula:
    return Formula("atom", robot=robot, region=region)


def And(*args: Formula) -> Formula:
    return Formula("and", args)


def Or(*args: Formula) -> Formula:
    return Formula("or", args)


def Not(arg: Formula) -> Formula:
    return Formula("not", (arg,))


def Next(arg: Formula) -> Formula:
    return Formula("next", (arg,))


def Until(left: Formula, right: Formula) -> Formula:
    return Formula("until", (left, right))


def Release(left: Formula, right: Formula) -> Formula:
    return Formula("release", (left, right))


def Eventually(arg: Formula) -> Formula:
    return Formula("eventually", (arg,))


def Always(arg: Formula) -> Formula:
    return Formula("always", (arg,))


def Implies(left: Formula, right: Formula) -> Formula:
    return Formula("implies", (left, right))


def Count(region: str, robots: Sequence[str], minimum: int = 1, maximum: int | None = None) -> Formula:
    """Count distinct named robots occupying a region in the current state."""
    return Formula("count", region=region, robots=robots, minimum=minimum, maximum=maximum)


def from_dict(data: Mapping[str, Any]) -> Formula:
    return Formula.from_dict(data)


def atoms(formula: Formula) -> frozenset[tuple[str, str]]:
    """Return all (robot, region) references, including cardinality predicates."""
    found: set[tuple[str, str]] = set()
    seen: set[Formula] = set()
    pending = [formula]
    while pending:
        node = pending.pop()
        if node in seen:
            continue
        seen.add(node)
        if node.op == "atom":
            found.add((node.robot, node.region))
        elif node.op == "count":
            found.update((robot, node.region) for robot in node.robots)
        pending.extend(node.args)
    return frozenset(found)


def _validate_trace_length(horizon: int, loop_start: int | None) -> None:
    if type(horizon) is not int or horizon <= 0:
        raise ValueError("horizon must be a positive integer counting unique states")
    if loop_start is not None:
        if type(loop_start) is not int or not 0 <= loop_start < horizon:
            raise ValueError("loop_start must index a unique state in the trace")


def encode(
    formula: Formula,
    horizon: int,
    atom_at: Callable[[str, str, int], z3.BoolRef],
    loop_start: int | None = None,
) -> z3.BoolRef:
    """Encode satisfaction at state 0, with no hidden solver constraints.

    The caller provides each atomic region-membership Boolean. For a lasso,
    it must independently constrain the robots' physical closing transition
    from the last state to ``loop_start``. Boolean periodicity alone does not
    guarantee that a geometric trajectory can traverse that transition.
    """
    _validate_trace_length(horizon, loop_start)

    def successor(t: int) -> int | None:
        return t + 1 if t + 1 < horizon else loop_start

    @lru_cache(maxsize=None)
    def reachable(t: int) -> tuple[int, ...]:
        states: list[int] = []
        seen: set[int] = set()
        while t is not None and t not in seen:
            states.append(t)
            seen.add(t)
            t = successor(t)
        return tuple(states)

    def until_at(left: Formula, right: Formula, t: int, negate_operands: bool = False) -> z3.BoolRef:
        # Once a state repeats, operand truth values repeat too. A first
        # right-hand witness must therefore occur on this unique-state path.
        prefix = z3.BoolVal(True)
        witnesses: list[z3.BoolRef] = []
        for state in reachable(t):
            a, b = visit(left, state), visit(right, state)
            if negate_operands:
                a, b = z3.Not(a), z3.Not(b)
            witnesses.append(z3.And(prefix, b))
            prefix = z3.And(prefix, a)
        return z3.Or(*witnesses)

    @lru_cache(maxsize=None)
    def visit(node: Formula, t: int) -> z3.BoolRef:
        op = node.op
        if op in {"true", "false"}:
            return z3.BoolVal(op == "true")
        if op == "atom":
            return atom_at(node.robot, node.region, t)
        if op == "count":
            predicates = [(atom_at(robot, node.region, t), 1) for robot in node.robots]
            if not predicates:
                return z3.BoolVal(node.minimum == 0)
            lower = z3.PbGe(predicates, node.minimum)
            upper = z3.BoolVal(True) if node.maximum is None else z3.PbLe(predicates, node.maximum)
            return z3.And(lower, upper)
        if op == "and":
            return z3.And(*(visit(arg, t) for arg in node.args))
        if op == "or":
            return z3.Or(*(visit(arg, t) for arg in node.args))
        if op == "not":
            return z3.Not(visit(node.args[0], t))
        if op == "implies":
            return z3.Implies(visit(node.args[0], t), visit(node.args[1], t))
        if op == "next":
            next_t = successor(t)
            return z3.BoolVal(False) if next_t is None else visit(node.args[0], next_t)
        if op == "eventually":
            return z3.Or(*(visit(node.args[0], state) for state in reachable(t)))
        if op == "always":
            return z3.And(*(visit(node.args[0], state) for state in reachable(t)))
        if op == "until":
            return until_at(*node.args, t)
        if op == "release":
            return z3.Not(until_at(*node.args, t, negate_operands=True))
        raise AssertionError(f"unhandled validated operator: {op}")

    return visit(formula, 0)


State = Mapping[str, str | Collection[str]]


def evaluate(formula: Formula, trace: Sequence[State], loop_start: int | None = None) -> bool:
    """Evaluate a concrete trace independently using Boolean fixed points.

    Each state maps robot names to a region name, or to a collection of active
    region names (useful for overlapping regions). A missing robot occupies
    none of the named regions. An empty trace is rejected in both modes.
    """
    n = len(trace)
    _validate_trace_length(n, loop_start)
    if any(not isinstance(state, Mapping) for state in trace):
        raise TypeError("each trace state must map robot names to regions")

    def atom_value(robot: str, region: str, t: int) -> bool:
        active = trace[t].get(robot, ())
        return active == region if isinstance(active, str) else region in active

    def shift(values: tuple[bool, ...], finite_end: bool = False) -> tuple[bool, ...]:
        return values[1:] + ((finite_end if loop_start is None else values[loop_start]),)

    def fixed_point(left: tuple[bool, ...], right: tuple[bool, ...], greatest: bool) -> tuple[bool, ...]:
        current = (greatest,) * n
        # Monotone Boolean equations on n states converge in at most n changes.
        # Release uses weak-next at the finite boundary; until uses strong-next.
        for _ in range(n + 1):
            following = shift(current, finite_end=greatest)
            if greatest:
                updated = tuple(b and (a or nxt) for a, b, nxt in zip(left, right, following))
            else:
                updated = tuple(b or (a and nxt) for a, b, nxt in zip(left, right, following))
            if updated == current:
                return updated
            current = updated
        raise AssertionError("monotone temporal fixed point did not converge")

    @lru_cache(maxsize=None)
    def visit(node: Formula) -> tuple[bool, ...]:
        op = node.op
        if op in {"true", "false"}:
            return (op == "true",) * n
        if op == "atom":
            return tuple(atom_value(node.robot, node.region, t) for t in range(n))
        if op == "count":
            counts = (sum(atom_value(robot, node.region, t) for robot in node.robots) for t in range(n))
            return tuple(
                count >= node.minimum and (node.maximum is None or count <= node.maximum) for count in counts
            )
        if op == "not":
            return tuple(not value for value in visit(node.args[0]))
        if op in {"and", "or"}:
            operands = [visit(arg) for arg in node.args]
            reduce = all if op == "and" else any
            return tuple(reduce(values[t] for values in operands) for t in range(n))
        if op == "implies":
            return tuple(not a or b for a, b in zip(visit(node.args[0]), visit(node.args[1])))
        if op == "next":
            return shift(visit(node.args[0]))
        if op == "eventually":
            return fixed_point((True,) * n, visit(node.args[0]), greatest=False)
        if op == "always":
            return fixed_point((False,) * n, visit(node.args[0]), greatest=True)
        if op in {"until", "release"}:
            return fixed_point(visit(node.args[0]), visit(node.args[1]), greatest=op == "release")
        raise AssertionError(f"unhandled validated operator: {op}")

    return visit(formula)[0]


__all__ = [
    "Formula",
    "Atom",
    "And",
    "Or",
    "Not",
    "Next",
    "Until",
    "Release",
    "Eventually",
    "Always",
    "Implies",
    "Count",
    "TRUE",
    "FALSE",
    "atoms",
    "encode",
    "evaluate",
    "from_dict",
]
