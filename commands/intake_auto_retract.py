import commands2

from constants.field import kDangerZone
from subsystems.controlled_motor import ControlledTalonMotor
from subsystems.drivetrain.drivetrain import SwerveDriveTrain
from util.nt_util import NTTable
from util.robot_zone_checker import RobotZoneChecker


class IntakeAutoRetract(commands2.Command):
    """Stops the intake motor whenever the robot is projected to enter the danger zone.

    Designed to run as a commands2 Trigger-based command:
      - Trigger condition: is_danger_zone_active() AND driver is NOT holding deploy button
      - When the driver holds the deploy button the Trigger condition goes False, cancelling
        this command and allowing SpinMotor to schedule — that is the manual override.

    NTTable "Intake/Auto Retract" exposes two tunable parameters:
      - Lookahead (s)      — how far ahead (in seconds) to project the velocity vector
      - Zone X Margin (m)  — half-width of the danger band around the hub centre in x
    """

    def __init__(
        self,
        intake_motor: ControlledTalonMotor,
        drivetrain: SwerveDriveTrain,
    ):
        super().__init__()
        self._intake = intake_motor
        self._drivetrain = drivetrain
        self.addRequirements(intake_motor)

        self.nt = NTTable("Intake/Auto Retract")
        self._lookahead = self.nt.float("Lookahead (s)", kDangerZone.DEFAULT_LOOKAHEAD_S)
        self._margin = self.nt.float("Zone X Margin (m)", kDangerZone.DEFAULT_X_MARGIN)
        self._in_zone = self.nt.bool("In Danger Zone", False)

    def is_danger_zone_active(self) -> bool:
        """Returns True when the projected robot position is inside the danger band.

        Called by the Trigger predicate every scheduler cycle — also updates the
        'In Danger Zone' NetworkTables entry for Glass/Shuffleboard telemetry.
        """
        state = self._drivetrain.get_state()
        result = RobotZoneChecker.is_projected_in_danger_zone(
            state.pose,
            state.velocity,
            self._lookahead.get(),
            self._margin.get(),
            kDangerZone.HUB_X,
        )
        self._in_zone.set(result)
        return result

    def initialize(self):
        pass

    def execute(self):
        self._intake.stop_motor()

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        pass
