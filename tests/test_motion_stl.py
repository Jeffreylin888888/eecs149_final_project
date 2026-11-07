import math
import unittest
from eecs149_robot.model import Pose
from eecs149_robot.motion import GoToGoal, integrate_odometry, wheels_from_twist
from eecs149_robot.stl import always, eventually, obstacle_clearance_margin
from eecs149_robot.manipulation import ArmController, MockArm, ServoPulse


class MotionTests(unittest.TestCase):
    def test_go_to_goal_and_odometry(self):
        ctl = GoToGoal()
        pose = Pose(0, 0, 0)
        for _ in range(1000):
            cmd = ctl.command(pose, 1, 1)
            if cmd.linear_m_s == cmd.angular_rad_s == 0:
                break
            left, right = wheels_from_twist(cmd, ctl.cfg.wheel_base_m)
            pose = integrate_odometry(pose, left * 0.02, right * 0.02,
                                      ctl.cfg.wheel_base_m)
        self.assertLess(math.dist((pose.x, pose.y), (1, 1)), 0.03)

    def test_stl(self):
        series = [0, 1, 3, 4]
        self.assertTrue(always(series, lambda v: float(v + 0.1)).satisfied)
        self.assertTrue(eventually(series, lambda v: float(v - 2), 0, 3).satisfied)
        self.assertFalse(eventually(series, lambda v: float(v - 2), 0, 1).satisfied)
        self.assertFalse(always(series, lambda v: float(v), 0, 10).satisfied)
        self.assertGreater(obstacle_clearance_margin((0, 0), [(2, 0)], 0.5), 0)

    def test_arm_three_servo_sequence(self):
        mock = MockArm()
        arm = ArmController(mock)
        arm.pick()
        arm.place()
        self.assertEqual(len(mock.history), 6)
        with self.assertRaises(ValueError):
            ServoPulse(500, 1500, 1500)


if __name__ == "__main__":
    unittest.main()
