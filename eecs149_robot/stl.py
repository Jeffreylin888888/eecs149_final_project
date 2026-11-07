"""Discrete-time STL monitors; finite traces use strong finite-prefix semantics."""
from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, Sequence, TypeVar

T = TypeVar("T")


@dataclass(frozen=True)
class Result:
    satisfied: bool
    robustness: float
    explanation: str


def always(trace: Sequence[T], margin: Callable[[T], float], a: int = 0,
           b: int | None = None) -> Result:
    stop = len(trace) - 1 if b is None else b
    if a < 0 or stop < a or stop >= len(trace):
        return Result(False, float("-inf"), "Incomplete G window")
    rho = min(float(margin(v)) for v in trace[a:stop + 1])
    return Result(rho >= 0, rho, f"G[{a},{stop}]")


def eventually(trace: Sequence[T], margin: Callable[[T], float], a: int = 0,
               b: int | None = None) -> Result:
    stop = len(trace) - 1 if b is None else b
    if a < 0 or stop < a or stop >= len(trace):
        return Result(False, float("-inf"), "Incomplete F window")
    rho = max(float(margin(v)) for v in trace[a:stop + 1])
    return Result(rho >= 0, rho, f"F[{a},{stop}]")


def obstacle_clearance_margin(pos: tuple[float, float],
                              obstacles: list[tuple[float, float]],
                              safe_distance: float) -> float:
    import math
    return min((math.dist(pos, o) - safe_distance for o in obstacles), default=float("inf"))
