import math

import commands2
import phoenix6
import wpilib
from wpilib import RobotController
from wpilib.simulation import FlywheelSim
from wpimath.system.plant import DCMotor, LinearSystemId

from util.nt_util import NTTable


class ControlledTalonMotor(commands2.Subsystem):
    def __init__(
        self,
        name: str,
        id: int,
        config: phoenix6.configs.TalonFXConfiguration,
        target_rpm: float,
        coast_when_neutral=False,
        motor_type=None,
        moment_of_inertia: float = 0.001,
    ):
        super().__init__()

        self._motor = phoenix6.hardware.TalonFX(id, "rio")
        self.name = name
        self.cfg = config

        for _ in range(5):
            status = self._motor.configurator.apply(self.cfg)
            if status.is_ok():
                break

        self.velocity_voltage = phoenix6.controls.VelocityVoltage(velocity=0, slot=0)

        self.nt = NTTable(self.name).get_subtable("Controlled Motor")
        self.nt.float("k_p", self.cfg.slot0.k_p)
        self.nt.float("k_i", self.cfg.slot0.k_i)
        self.nt.float("k_d", self.cfg.slot0.k_d)
        self.nt.float("Target RPM", target_rpm)
        self.nt.bool("Working", False)

        self._RPS = target_rpm / 60

        if coast_when_neutral:
            self._motor.setNeutralMode(phoenix6.signals.NeutralModeValue.COAST)
        else:
            self._motor.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)

        if wpilib.RobotBase.isSimulation():
            _model = motor_type if motor_type is not None else DCMotor.krakenX60(1)
            _plant = LinearSystemId.flywheelSystem(_model, moment_of_inertia, 1.0)
            self._flywheel_sim = FlywheelSim(_plant, _model)

    def get_rpm_error(self) -> float:
        """Returns target_rpm - actual_rpm. Magnitude grows when motor slows under load."""
        actual_rpm = self._motor.get_velocity().value * 60
        target_rpm = self._RPS * 60
        return target_rpm - actual_rpm

    def is_at_target(self, threshold_pct: float = 0.1) -> bool:
        """Returns True when actual RPM is within threshold_pct (0–1) of target RPM."""
        if self._RPS == 0:
            return True
        return abs(self.get_rpm_error()) / abs(self._RPS * 60) <= threshold_pct

    def spin(self, extra_rps: float = 0.0):
        self._motor.set_control(self.velocity_voltage.with_velocity(self._RPS + extra_rps))
        self.nt.set("Working", True)

    def get_actual_rps(self) -> float:
        """Return the current motor velocity in rotations per second (always positive)."""
        return abs(self._motor.get_velocity().value)

    @property
    def target_rps(self) -> float:
        """Current commanded target velocity in rotations per second (always positive)."""
        return abs(self._RPS)

    def get_rpm(self):
        return self._motor.get_velocity().value * 60

    def stop_motor(self):
        self._motor.set(0)
        self.nt.set("Working", False)

    def periodic(self):
        self.nt.set("RPM", self._motor.get_velocity().value * 60)
        self._RPS = self.nt.get("Target RPM") / 60

        value_changed = (
            self.cfg.slot0.k_p != self.nt.get("k_p")
            or self.cfg.slot0.k_i != self.nt.get("k_i")
            or self.cfg.slot0.k_d != self.nt.get("k_d")
        )

        if value_changed:
            self.cfg.slot0.k_p = self.nt.get("k_p")
            self.cfg.slot0.k_i = self.nt.get("k_i")
            self.cfg.slot0.k_d = self.nt.get("k_d")
            self._motor.configurator.apply(self.cfg)

    def simulationPeriodic(self):
        self._motor.sim_state.set_supply_voltage(RobotController.getBatteryVoltage())
        self._flywheel_sim.setInputVoltage(self._motor.sim_state.motor_voltage)
        self._flywheel_sim.update(0.02)
        sim_rps = self._flywheel_sim.getAngularVelocity() / (2 * math.pi)
        self._motor.sim_state.set_rotor_velocity(sim_rps)
        self._motor.sim_state.add_rotor_position(sim_rps * 0.02)
