"""Wheel odometry and bounded go-to-goal differential-drive feedback."""
from __future__ import annotations
import math
from dataclasses import dataclass
from .model import Pose, Twist


def wrap(theta: float) -> float:
    return (theta + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class DriveConfig:
    wheel_radius_m: float = 0.035
    wheel_base_m: float = 0.14
    max_linear_m_s: float = 0.12
    max_angular_rad_s: float = 1.1
    tolerance_m: float = 0.025
    turn_tolerance_rad: float = 0.12
    kp_distance: float = 0.8
    kp_heading: float = 2.8


class GoToGoal:
    def __init__(self, cfg: DriveConfig | None = None):
        self.cfg = cfg or DriveConfig()

    def command(self, pose: Pose, target_x: float, target_y: float) -> Twist:
        dx, dy = target_x - pose.x, target_y - pose.y
        dist = math.hypot(dx, dy)
        if dist <= self.cfg.tolerance_m:
            return Twist(0.0, 0.0)
        heading = wrap(math.atan2(dy, dx) - pose.theta)
        angular = max(-self.cfg.max_angular_rad_s,
                      min(self.cfg.max_angular_rad_s, self.cfg.kp_heading * heading))
        if abs(heading) > self.cfg.turn_tolerance_rad:
            return Twist(0.0, angular)
        linear = max(0.0, min(self.cfg.max_linear_m_s, self.cfg.kp_distance * dist))
        return Twist(linear, angular)


def wheels_from_twist(twist: Twist, wheel_base_m: float) -> tuple[float, float]:
    return (twist.linear_m_s - twist.angular_rad_s * wheel_base_m / 2,
            twist.linear_m_s + twist.angular_rad_s * wheel_base_m / 2)


def integrate_odometry(pose: Pose, d_left_m: float, d_right_m: float,
                       wheel_base_m: float) -> Pose:
    ds = (d_left_m + d_right_m) / 2.0
    dtheta = (d_right_m - d_left_m) / wheel_base_m
    midpoint = pose.theta + dtheta / 2.0
    return Pose(pose.x + ds * math.cos(midpoint),
                pose.y + ds * math.sin(midpoint),
                wrap(pose.theta + dtheta))
