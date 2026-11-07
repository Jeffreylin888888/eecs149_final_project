"""Executable model with invariant checks and request service events."""
from __future__ import annotations
from dataclasses import dataclass, field
from .manipulation import ArmController, MockArm
from .model import Action, Cell, Plan, Request, Trace, World
from .scheduler import Scheduler
from .synthesis import NoPlanError, synthesize, validate_plan


class SafetyError(RuntimeError):
    pass


@dataclass
class RobotSimulator:
    world: World
    position: Cell | None = None
    held: str | None = None
    completed: list[str] = field(default_factory=list)
    trace: Trace = field(default_factory=Trace)
    arm: ArmController = field(default_factory=lambda: ArmController(MockArm()))

    def __post_init__(self):
        if self.position is None:
            self.position = self.world.base
        if not self.world.free(self.position):
            raise SafetyError("Cannot initialize robot inside an obstacle")
        self.trace.cells.append(self.position)
        self.trace.holding.append(None)
        self.trace.events.append("start")

    def execute_step(self, plan: Plan, index: int) -> None:
        step = plan.steps[index]
        a = step.action
        if a in (Action.NORTH, Action.SOUTH, Action.WEST, Action.EAST):
            nxt = dict(self.world.neighbors(self.position)).get(a)
            if nxt is None:
                self.trace.collisions += 1
                raise SafetyError("Blocked or out-of-bounds move")
            self.position = nxt
        elif a is Action.PICK:
            if self.held is not None or self.position != self.world.objects[plan.request.item]:
                raise SafetyError("Pick precondition false")
            self.arm.pick()
            self.held = plan.request.item
        elif a is Action.DROP:
            if self.held != plan.request.item or self.position != self.world.recipients[plan.request.recipient]:
                raise SafetyError("Drop precondition false")
            self.arm.place()
            self.held = None
            self.completed.append(plan.request.request_id)
        elif a is Action.WAIT:
            if self.position != self.world.base or self.held is not None:
                raise SafetyError("Return-to-base precondition false")
        else:
            raise SafetyError("Unknown action")
        if step.cell != self.position or not self.world.free(self.position):
            raise SafetyError("Plan position or safety invariant violated")
        self.trace.cells.append(self.position)
        self.trace.holding.append(self.held)
        self.trace.events.append(a.value)

    def execute(self, req: Request) -> Plan:
        plan = synthesize(self.world, req, self.position)
        if not validate_plan(self.world, plan):
            raise SafetyError("Synthesized plan invalid")
        for i in range(len(plan.steps)):
            self.execute_step(plan, i)
        return plan


def run_prioritized(world: World, requests: list[Request]) -> RobotSimulator:
    scheduler = Scheduler()
    for req in requests:
        scheduler.submit(req)
    sim = RobotSimulator(world)
    while (req := scheduler.pop()) is not None:
        sim.execute(req)
    return sim


def sample_world() -> World:
    return World(width=7, height=6, base=(0, 0),
                 objects={"medicine": (1, 4), "food": (3, 4), "water": (6, 5)},
                 recipients={"alice": (6, 0), "bob": (6, 4), "eve": (0, 5)},
                 blocked=frozenset({(2, 1), (2, 2), (2, 3), (4, 2), (5, 2)}))
