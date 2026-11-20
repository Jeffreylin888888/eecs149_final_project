"""Small deterministic examples and the original benchmark's grid geometry."""

from __future__ import annotations

from itertools import product

from .ltl import Always, And, Atom, Count, Eventually, Or
from .models import Problem, Region, Robot, Workspace, integrator_chain


def reach_avoid() -> Problem:
    """A double integrator must travel around a rectangular obstacle.

    The three free polytopes form an upside-down U. At the region level the
    only route from start to goal passes through bridge. Eight to twelve
    sampled states allow a comfortable solution with bounded acceleration.
    """
    regions = (
        Region.box("start", [0, 0], [2, 4]),
        Region.box("bridge", [2, 3], [4, 4]),
        Region.box("goal", [4, 0], [6, 4]),
    )
    obstacle = Region.box("obstacle", [2.2, 0], [3.8, 2.8])
    workspace = Workspace(
        regions, {"start": ("bridge",), "bridge": ("start", "goal"), "goal": ("bridge",)}, (obstacle,)
    )
    robot = Robot("r1", integrator_chain(), [1, 0, 1, 0], (None, 0, None, 0))
    return Problem(workspace, (robot,), Eventually(Atom("r1", "goal")), name="reach-avoid")


def multi_robot_patrol() -> Problem:
    """Two robots repeatedly visit both ends of their own patrol lanes.

    Use lasso semantics: each GF obligation must have a witness on the loop,
    and the full position/velocity state must close dynamically. The lanes
    keep the two robots physically separated while demonstrating independent
    temporal obligations in a shared workspace.
    """
    regions = []
    adjacency = {}
    robots = []
    obligations = []
    dynamics = integrator_chain(input_bound=2.0)
    for name, bottom in (("r1", 0.0), ("r2", 2.0)):
        names = tuple(f"{name}_{suffix}" for suffix in ("left", "lane", "right"))
        for region_name, low_x, high_x in zip(names, (0.0, 1.0, 3.0), (1.0, 3.0, 4.0)):
            regions.append(Region.box(region_name, [low_x, bottom], [high_x, bottom + 1]))
        adjacency.update({names[0]: (names[1],), names[1]: (names[0], names[2]), names[2]: (names[1],)})
        robots.append(Robot(name, dynamics, [0.5, 0, bottom + 0.5, 0]))
        obligations.extend(
            (
                Always(Or(*(Atom(name, region) for region in names))),
                Always(Eventually(Atom(name, names[0]))),
                Always(Eventually(Atom(name, names[2]))),
            )
        )
    return Problem(
        Workspace(tuple(regions), adjacency),
        tuple(robots),
        And(*obligations),
        separation=0.5,
        name="multi-robot-patrol",
    )


def three_dimensional() -> Problem:
    """A three-axis, third-order (jerk-controlled) robot reaches a 3D goal."""
    regions = (
        Region.box("start", [0, 0, 0], [1, 2, 2]),
        Region.box("middle", [1, 0, 0], [3, 2, 2]),
        Region.box("goal", [3, 0, 0], [4, 2, 2]),
    )
    workspace = Workspace(regions, {"start": ("middle",), "middle": ("start", "goal"), "goal": ("middle",)})
    robot = Robot(
        "r1", integrator_chain(dimensions=3, order=3), [0.5, 0, 0, 1, 0, 0, 1, 0, 0], (None, 0, 0) * 3
    )
    return Problem(workspace, (robot,), Eventually(Atom("r1", "goal")), name="three-dimensional")


def multi_robot_crossing() -> Problem:
    """Two double integrators swap sides through the same middle region.

    Both start on y=1, so a direct simultaneous swap would collide. The solver
    must coordinate timing or use available vertical space while maintaining
    the sampled infinity-norm separation of 0.5.
    """
    regions = (
        Region.box("left", [0, 0], [2, 4]),
        Region.box("middle", [2, 0], [4, 4]),
        Region.box("right", [4, 0], [6, 4]),
    )
    workspace = Workspace(regions, {"left": ("middle",), "middle": ("left", "right"), "right": ("middle",)})
    dynamics = integrator_chain(input_bound=2.0)
    robots = (
        Robot("r1", dynamics, [1, 0, 1, 0], (None, 0, None, 0)),
        Robot("r2", dynamics, [5, 0, 1, 0], (None, 0, None, 0)),
    )
    specification = And(Eventually(Always(Atom("r1", "right"))), Eventually(Always(Atom("r2", "left"))))
    return Problem(workspace, robots, specification, separation=0.5, name="multi-robot-crossing")


def team_recurrence() -> Problem:
    """Repeated joint meetings plus individual recurring target visits.

    The cardinality obligation requires both robots to occupy the meeting
    region at the same sampled time infinitely often. Each also repeatedly
    returns to its own end of the workspace. Use lasso mode with 12–16 unique
    states; all position and velocity components must close at the loop.
    """
    regions = (
        Region.box("left_target", [0, 0], [2, 4]),
        Region.box("meeting", [2, 0], [4, 4]),
        Region.box("right_target", [4, 0], [6, 4]),
    )
    workspace = Workspace(
        regions,
        {
            "left_target": ("meeting",),
            "meeting": ("left_target", "right_target"),
            "right_target": ("meeting",),
        },
    )
    dynamics = integrator_chain(input_bound=2.0)
    robots = (Robot("r1", dynamics, [1, 0, 1.5, 0]), Robot("r2", dynamics, [5, 0, 2.5, 0]))
    specification = And(
        Always(Eventually(Count("meeting", ("r1", "r2"), minimum=2))),
        Always(Eventually(Atom("r1", "left_target"))),
        Always(Eventually(Atom("r2", "right_target"))),
    )
    return Problem(workspace, robots, specification, separation=0.5, name="team-recurrence")


def legacy_workspace(x_ticks: int = 6, y_ticks: int = 4, z_ticks: int = 4) -> Workspace:
    """Re-create the original SatEX LTL benchmark's unit-cell workspace.

    Cell indices start at one. The original marks cells with an even x index
    from 2 through 30 and a z index in {1, 3} as obstacles, independently of
    y. Adjacency joins cells sharing a face, and self transitions are allowed.
    Here obstacle cells are separated from free regions; names such as
    ``cell_1_1_1`` avoid the original concatenated-tag ambiguity.

    Geometry follows ``robotic_LTL_workspace`` in the 2016 SatEX release,
    copyright The Regents of the University of California. See the bundled
    third-party notice for the original license and acknowledgment.
    """
    for value, label in ((x_ticks, "x_ticks"), (y_ticks, "y_ticks"), (z_ticks, "z_ticks")):
        if isinstance(value, bool) or not isinstance(value, int) or value < 2:
            raise ValueError(f"{label} must be an integer of at least two")
    cells = tuple(product(range(1, x_ticks), range(1, y_ticks), range(1, z_ticks)))
    blocked = {cell for cell in cells if cell[0] in range(2, 31, 2) and cell[2] in (1, 3)}
    free = set(cells) - blocked

    def name(cell: tuple[int, int, int]) -> str:
        return "cell_" + "_".join(map(str, cell))

    def region(cell: tuple[int, int, int]) -> Region:
        return Region.box(name(cell), [value - 1 for value in cell], cell)

    adjacency = {}
    for cell in cells:
        if cell not in free:
            continue
        neighbors = []
        for axis in range(3):
            for delta in (-1, 1):
                neighbor = list(cell)
                neighbor[axis] += delta
                if tuple(neighbor) in free:
                    neighbors.append(name(tuple(neighbor)))
        adjacency[name(cell)] = tuple(neighbors)
    return Workspace(
        tuple(region(cell) for cell in cells if cell in free),
        adjacency,
        tuple(region(cell) for cell in cells if cell in blocked),
    )


def legacy_reach_avoid(x_ticks: int = 6, y_ticks: int = 4, z_ticks: int = 4) -> Problem:
    """Reach the far cell in the legacy grid using corrected ZOH dynamics.

    The default geometry and zero initial state follow the supplied benchmark.
    This example uses three independent jerk-controlled axes and omits its yaw
    subsystem. It is therefore a modern demonstration, not a reproduction of
    the paper's timing or the legacy implementation's numerical trajectories.
    """
    workspace = legacy_workspace(x_ticks, y_ticks, z_ticks)
    goal = f"cell_{x_ticks - 1}_{y_ticks - 1}_{z_ticks - 1}"
    if goal not in workspace.region_map:
        raise ValueError("the requested far corner is an obstacle in the legacy grid")
    robot = Robot("r1", integrator_chain(dimensions=3, order=3), [0] * 9)
    return Problem(workspace, (robot,), Eventually(Atom("r1", goal)), name="legacy-reach-avoid")
