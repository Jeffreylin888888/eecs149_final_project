"""Shortest-path reactive-plan synthesis via product of map and task automaton.

Supported LTL fragment: ordered eventually-goal events with an invariant:
  G(!collision) & F(PICK(item) & X F(DROP(recipient) & X F(BASE)))

The deterministic automaton phases are 0:not picked, 1:carrying, 2:dropped,
3:returned to base. BFS searches grid x automaton, so the resulting plan is
shortest in discrete actions, and avoids obstacles at each action.
"""
from __future__ import annotations
from collections import deque
from dataclasses import dataclass
from .model import Action, Cell, Plan, PlanStep, Request, World


class NoPlanError(RuntimeError):
    pass


@dataclass(frozen=True)
class _State:
    cell: Cell
    phase: int


def synthesize(world: World, req: Request, start: Cell | None = None,
               max_steps: int | None = None) -> Plan:
    if req.item not in world.objects:
        raise ValueError(f"Unknown item {req.item!r}")
    if req.recipient not in world.recipients:
        raise ValueError(f"Unknown recipient {req.recipient!r}")
    origin = world.base if start is None else start
    if not world.free(origin):
        raise ValueError("Start must be in free space")
    target_item, target_user = world.objects[req.item], world.recipients[req.recipient]
    first = _State(origin, 0)
    parent: dict[_State, tuple[_State, Action] | None] = {first: None}
    queue = deque([first])
    final: _State | None = None
    while queue:
        node = queue.popleft()
        if node.phase == 3:
            final = node
            break
        choices: list[tuple[Action, _State]] = []
        if node.phase == 0 and node.cell == target_item:
            choices.append((Action.PICK, _State(node.cell, 1)))
        if node.phase == 1 and node.cell == target_user:
            choices.append((Action.DROP, _State(node.cell, 2)))
        if node.phase == 2 and node.cell == world.base:
            choices.append((Action.WAIT, _State(node.cell, 3)))
        choices.extend((action, _State(c, node.phase)) for action, c in world.neighbors(node.cell))
        for action, successor in choices:
            if successor not in parent:
                parent[successor] = (node, action)
                queue.append(successor)
    if final is None:
        raise NoPlanError("No collision-free ordered pick/deliver/return plan")
    out: list[PlanStep] = []
    node = final
    while parent[node] is not None:
        prev, action = parent[node]
        out.append(PlanStep(action, node.cell, node.phase))
        node = prev
    out.reverse()
    deadline = req.due_steps if max_steps is None else min(req.due_steps, max_steps)
    # Include return to base in deadline, a conservative interpretation.
    if len(out) > deadline:
        raise NoPlanError(f"Shortest completion {len(out)} steps > bound {deadline}")
    formula = ("G(!collision) & F(PICK_" + req.item + " & X F(DROP_" +
               req.recipient + " & X F(BASE)))")
    return Plan(req, tuple(out), origin, world.base, formula)


def validate_plan(world: World, plan: Plan) -> bool:
    """Independent semantic replay of a synthesized plan."""
    cell = plan.start
    holding: str | None = None
    delivered = False
    returned = False
    for step in plan.steps:
        if step.action in (Action.NORTH, Action.SOUTH, Action.EAST, Action.WEST):
            candidates = dict(world.neighbors(cell))
            if step.action not in candidates or candidates[step.action] != step.cell:
                return False
            cell = step.cell
        elif step.action is Action.PICK:
            if holding is not None or cell != world.objects[plan.request.item] or delivered:
                return False
            holding = plan.request.item
        elif step.action is Action.DROP:
            if holding != plan.request.item or cell != world.recipients[plan.request.recipient]:
                return False
            holding, delivered = None, True
        elif step.action is Action.WAIT:
            if not delivered or cell != world.base:
                return False
            returned = True
        else:
            return False
        if step.cell != cell or not world.free(cell):
            return False
    return delivered and returned and holding is None and cell == world.base
