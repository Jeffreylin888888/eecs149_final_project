"""Command line demo, synthesis visualization, and self-test launcher."""
from __future__ import annotations
import argparse
import json
import unittest
from .model import Request
from .simulator import RobotSimulator, run_prioritized, sample_world
from .synthesis import synthesize, validate_plan


def main() -> None:
    parser = argparse.ArgumentParser(description="EECS149 specification-driven robot")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("demo", help="Simulate prioritized manipulation and return-to-base")
    sub.add_parser("test", help="Run offline unit tests")
    sp = sub.add_parser("synthesize", help="Compute one collision-free product-automaton plan")
    sp.add_argument("--item", choices=["medicine", "food", "water"], default="medicine")
    sp.add_argument("--recipient", choices=["alice", "bob", "eve"], default="alice")
    sp.add_argument("--deadline", type=int, default=100)
    args = parser.parse_args()
    if args.command == "test":
        import pathlib
        root = pathlib.Path(__file__).resolve().parent.parent
        result = unittest.TextTestRunner(verbosity=2).run(
            unittest.defaultTestLoader.discover(str(root / "tests")))
        if not result.wasSuccessful():
            raise SystemExit(1)
    elif args.command == "synthesize":
        world = sample_world()
        req = Request("R1", args.item, args.recipient, 10, args.deadline)
        p = synthesize(world, req)
        print(json.dumps({"specification": p.formula, "steps": p.cost,
                          "valid": validate_plan(world, p),
                          "actions": [{"action": s.action.value, "cell": s.cell,
                                       "phase": s.phase} for s in p.steps]}, indent=2))
    elif args.command == "demo":
        world = sample_world()
        requests = [
            Request("food-bob", "food", "bob", priority=2),
            Request("medicine-alice", "medicine", "alice", priority=10),
            Request("water-eve", "water", "eve", priority=1),
        ]
        sim = run_prioritized(world, requests)
        print(json.dumps({"completed_in_order": sim.completed,
                          "last_position": sim.position, "collision_count": sim.trace.collisions,
                          "steps": len(sim.trace.events) - 1,
                          "arm_pose_commands": len(sim.arm.driver.history)}, indent=2))


if __name__ == "__main__":
    main()
