"""Validated, hardware-independent domain model."""
from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import Mapping

Cell = tuple[int, int]


class Action(str, Enum):
    NORTH = "N"
    SOUTH = "S"
    WEST = "W"
    EAST = "E"
    PICK = "PICK"
    DROP = "DROP"
    WAIT = "WAIT"


DIRECTION: dict[Action, Cell] = {
    Action.NORTH: (0, -1),
    Action.SOUTH: (0, 1),
    Action.WEST: (-1, 0),
    Action.EAST: (1, 0),
}


@dataclass(frozen=True)
class World:
    width: int
    height: int
    base: Cell
    objects: Mapping[str, Cell]
    recipients: Mapping[str, Cell]
    blocked: frozenset[Cell] = frozenset()

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("World dimensions must be positive")
        for cell in [self.base, *self.objects.values(), *self.recipients.values()]:
            if not self.free(cell):
                raise ValueError(f"Station is outside free space: {cell}")
        for c in self.blocked:
            if not self.in_bounds(c):
                raise ValueError(f"Blocked cell outside map: {c}")

    def in_bounds(self, cell: Cell) -> bool:
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def free(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and cell not in self.blocked

    def neighbors(self, cell: Cell):
        for action, (dx, dy) in DIRECTION.items():
            nxt = (cell[0] + dx, cell[1] + dy)
            if self.free(nxt):
                yield action, nxt


@dataclass(frozen=True)
class Request:
    request_id: str
    item: str
    recipient: str
    priority: int
    due_steps: int = 100

    def __post_init__(self) -> None:
        if not self.request_id or not self.item or not self.recipient:
            raise ValueError("Request identifiers must be nonempty")
        if self.due_steps <= 0:
            raise ValueError("due_steps must be positive")


@dataclass(frozen=True)
class PlanStep:
    action: Action
    cell: Cell
    phase: int


@dataclass(frozen=True)
class Plan:
    request: Request
    steps: tuple[PlanStep, ...]
    start: Cell
    goal: Cell
    formula: str

    @property
    def cost(self) -> int:
        return len(self.steps)


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class Twist:
    linear_m_s: float
    angular_rad_s: float


@dataclass
class Trace:
    cells: list[Cell] = field(default_factory=list)
    holding: list[str | None] = field(default_factory=list)
    events: list[str] = field(default_factory=list)
    collisions: int = 0
