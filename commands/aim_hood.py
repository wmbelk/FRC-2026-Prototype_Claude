import math

import commands2

from constants.field import kHub
from subsystems.controlled_motor import ControlledTalonMotor
from subsystems.drivetrain.drivetrain import SwerveDriveTrain
from subsystems.shooter.shooter_hood import ShooterHood


class AimHood(commands2.Command):
    """
    Continuously aims the shooter hood based on the robot's distance to the hub.

    Pass shooter_motor to enable RPM compensation: if the flywheel is running
    below commanded speed, the hood angle is recalculated for the actual exit
    velocity so shots still arc correctly.
    """

    def __init__(
        self,
        hood: ShooterHood,
        drivetrain: SwerveDriveTrain,
        shooter_motor: ControlledTalonMotor = None,
    ):
        super().__init__()
        self._hood = hood
        self._drivetrain = drivetrain
        self._shooter = shooter_motor
        self.addRequirements(hood)

    def execute(self):
        pose = self._drivetrain.get_state().pose
        dx = kHub.POS.X() - pose.X()
        dy = kHub.POS.Y() - pose.Y()
        distance = math.hypot(dx, dy)

        if self._shooter is not None:
            actual_rps = self._shooter.get_actual_rps()
            self._hood.with_RPS(actual_rps)

        self._hood.angle_hood(distance)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        # Hood stays at its last commanded position — no need to zero on interrupt
        pass
