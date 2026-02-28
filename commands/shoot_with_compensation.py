import commands2
from wpilib import SmartDashboard

from subsystems.controlled_motor import ControlledTalonMotor


class ShootWithTransferCompensation(commands2.Command):
    """
    Spins the shooter motor and preemptively compensates for RPM drop by
    reading transfer motor 2's RPM error and adding a proportion of it to
    the shooter's output each cycle.

    Compensation only activates once the shooter has reached its target speed
    (within "Shooter/Ready Threshold %"). During spin-up the motor runs
    normally with no feedforward so the correction doesn't over-compensate.

    Both values are tunable live via SmartDashboard:
      "Shooter/Compensation Gain"  — start at 0.0, increase slowly
      "Shooter/Ready Threshold %"  — % of target RPM within which shooter is "ready"
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
        SmartDashboard.putNumber("Shooter/Ready Threshold %", 10.0)

    def execute(self):
        gain = SmartDashboard.getNumber("Shooter/Compensation Gain", 0.0)
        threshold_pct = SmartDashboard.getNumber("Shooter/Ready Threshold %", 10.0) / 100.0

        if not self._shooter.is_at_target(threshold_pct):
            self._shooter.spin()
            return

        rpm_error = self._transfer.get_rpm_error()
        extra_rps = (rpm_error * gain) / 60.0
        self._shooter.spin(extra_rps)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self._shooter.stop_motor()
