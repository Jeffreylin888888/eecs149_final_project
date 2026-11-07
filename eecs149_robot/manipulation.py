"""Three-servo Romi arm abstraction with simulated implementation and hard limits."""
from __future__ import annotations
from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class ServoPulse:
    lift_us: int
    tilt_us: int
    gripper_us: int

    def __post_init__(self):
        for value in (self.lift_us, self.tilt_us, self.gripper_us):
            if not 900 <= value <= 2200:
                raise ValueError("Servo pulse outside cautious software envelope")


@dataclass(frozen=True)
class ArmProfile:
    """UNVERIFIED placeholder poses. Calibrate against assembled arm before energizing."""
    home: ServoPulse = ServoPulse(1500, 1500, 1500)
    approach: ServoPulse = ServoPulse(1350, 1600, 1900)
    grasp: ServoPulse = ServoPulse(1350, 1600, 1200)
    carry: ServoPulse = ServoPulse(1650, 1450, 1200)
    release: ServoPulse = ServoPulse(1350, 1600, 1900)


class ArmDriver(Protocol):
    def pose(self, pulse: ServoPulse) -> None: ...


class MockArm:
    def __init__(self):
        self.history: list[ServoPulse] = []

    def pose(self, pulse: ServoPulse) -> None:
        self.history.append(pulse)


class ArmController:
    def __init__(self, driver: ArmDriver, profile: ArmProfile | None = None):
        self.driver = driver
        self.profile = profile or ArmProfile()

    def pick(self) -> None:
        self.driver.pose(self.profile.approach)
        self.driver.pose(self.profile.grasp)
        self.driver.pose(self.profile.carry)

    def place(self) -> None:
        self.driver.pose(self.profile.approach)
        self.driver.pose(self.profile.release)
        self.driver.pose(self.profile.home)
