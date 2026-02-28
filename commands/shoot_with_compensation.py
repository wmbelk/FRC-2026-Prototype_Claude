import commands2
from wpilib import SmartDashboard

from subsystems.controlled_motor import ControlledTalonMotor


class ShootWithTransferCompensation(commands2.Command):
    """
    Spins the shooter motor and preemptively compensates for RPM drop by
    reading transfer motor 2's RPM error and adding a proportion of it to
    the shooter's output each cycle.

    The gain is tunable via SmartDashboard ("Shooter/Compensation Gain").
    Start at 0.0 and increase slowly until subsequent shots stay consistent.
    A positive gain boosts the shooter when the transfer motor slows under load.
    """

    def __init__(
        self,
        shooter: ControlledTalonMotor,
        transfer_monitor: ControlledTalonMotor,
    ):
        super().__init__()
        self._shooter = shooter
        self._transfer = transfer_monitor
        self.addRequirements(shooter)

        SmartDashboard.putNumber("Shooter/Compensation Gain", 0.0)

    def execute(self):
        gain = SmartDashboard.getNumber("Shooter/Compensation Gain", 0.0)
        rpm_error = self._transfer.get_rpm_error()
        extra_rps = (rpm_error * gain) / 60.0
        self._shooter.spin(extra_rps)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self._shooter.stop_motor()
