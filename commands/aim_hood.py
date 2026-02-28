import math

import commands2
from wpilib import SmartDashboard

from constants.field import kHub
from constants.drive import kAutoAlign
from subsystems.controlled_motor import ControlledTalonMotor
from subsystems.drivetrain.drivetrain import SwerveDriveTrain
from subsystems.shooter.shooter_hood import ShooterHood


class AimHood(commands2.Command):
    """
    Continuously aims the shooter hood based on the robot's distance to the hub.

    Velocity lookahead: the hood angles for where the robot will be when the
    ball arrives, not where it is now — the same correction HubAlign uses for
    yaw.  The lookahead time scales linearly with distance (further away →
    longer flight → more lead needed).

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

        SmartDashboard.putNumber("Hood/Velocity Correction Mult", kAutoAlign.CORRECTION_MULT)

    def execute(self):
        drive_state = self._drivetrain.get_state()

        # Current distance used only to scale the velocity lookahead
        current_distance = kHub.POS.distance(drive_state.pose.translation())
        velocity_correction = (
            SmartDashboard.getNumber("Hood/Velocity Correction Mult", kAutoAlign.CORRECTION_MULT)
            * current_distance
        )

        # Estimate where the robot will be when the ball arrives
        estimated_pose = drive_state.pose.transformBy(
            drive_state.velocity * velocity_correction
        )

        dx = kHub.POS.X() - estimated_pose.X()
        dy = kHub.POS.Y() - estimated_pose.Y()
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
