import unittest
from eecs149_robot.model import Action, Request, World
from eecs149_robot.synthesis import synthesize, validate_plan, NoPlanError
from eecs149_robot.simulator import RobotSimulator, run_prioritized, sample_world, SafetyError


class SynthesisTests(unittest.TestCase):
    def test_ordered_delivery(self):
        w = sample_world()
        p = synthesize(w, Request("r", "medicine", "alice", 10))
        self.assertTrue(validate_plan(w, p))
        self.assertEqual([s.action for s in p.steps if s.action in (Action.PICK, Action.DROP, Action.WAIT)],
                         [Action.PICK, Action.DROP, Action.WAIT])
        self.assertLess(p.cost, 100)

    def test_deadline_infeasible(self):
        with self.assertRaises(NoPlanError):
            synthesize(sample_world(), Request("r", "water", "alice", 1, 3))

    def test_unreachable(self):
        w = World(3, 3, (0, 0), {"x": (2, 0)}, {"y": (2, 2)},
                  frozenset({(1, 0), (1, 1), (1, 2)}))
        with self.assertRaises(NoPlanError):
            synthesize(w, Request("r", "x", "y", 1))

    def test_priority_queue(self):
        sim = run_prioritized(sample_world(), [Request("low", "food", "bob", 1),
                                                    Request("hi", "medicine", "alice", 5)])
        self.assertEqual(sim.completed, ["hi", "low"])
        self.assertEqual(sim.trace.collisions, 0)
        self.assertEqual(sim.position, sim.world.base)
        self.assertIsNone(sim.held)

    def test_collision_rejected(self):
        w = sample_world()
        sim = RobotSimulator(w, position=(1, 1))
        p = synthesize(w, Request("r", "medicine", "alice", 1), start=(0, 0))
        with self.assertRaises(SafetyError):
            sim.execute_step(p, 0)


if __name__ == "__main__":
    unittest.main()
