import math

import commands2
import phoenix6
import wpilib
from wpilib import RobotController, SmartDashboard
from wpilib.simulation import FlywheelSim
from wpimath.system.plant import DCMotor, LinearSystemId


class ControlledTalonMotor(commands2.Subsystem):
    def __init__(
        self,
        name: str,
        id: int,
        config: phoenix6.configs.TalonFXConfiguration,
        target_rpm: float,
        enable_smartdashboard=False,
        motor_type=None,
        moment_of_inertia: float = 0.001,
    ):
        super().__init__()

        self._motor = phoenix6.hardware.TalonFX(id, "rio")
        self.name = name
        self.cfg = config

        for _ in range(5):
            status = self._motor.configurator.apply(self.cfg)
            if status.is_ok(): break
            
        self.velocity_voltage = phoenix6.controls.VelocityVoltage(velocity=0, slot=0)

        self._RPS = target_rpm / 60

        self.enable_smartdashboard = enable_smartdashboard
        
        if self.enable_smartdashboard:
            SmartDashboard.putNumber(f"{self.name} k_p", self.cfg.slot0.k_p)
            SmartDashboard.putNumber(f"{self.name} k_i", self.cfg.slot0.k_i)
            SmartDashboard.putNumber(f"{self.name} k_d", self.cfg.slot0.k_d)
            SmartDashboard.putNumber(f"{self.name} Target RPM", target_rpm)
            SmartDashboard.putBoolean(f"{self.name} Working", False)
        
        self._motor.setNeutralMode(phoenix6.signals.NeutralModeValue.COAST)

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
        # self._motor.set_control(self.velocity_voltage.with_velocity(self._RPS))
        self._motor.set((self._RPS + extra_rps) / 100)

        if self.enable_smartdashboard:
            SmartDashboard.putBoolean(f"{self.name} Working", True)

    def get_actual_rps(self) -> float:
        """Return the current motor velocity in rotations per second (always positive)."""
        return abs(self._motor.get_velocity().value)

    def stop_motor(self):
        self._motor.set(0)
        if self.enable_smartdashboard:
            SmartDashboard.putBoolean(f"{self.name} Working", False)

    def periodic(self):
        
        SmartDashboard.putNumber(
            f"{self.name} RPM", self._motor.get_velocity().value * 60
        )

        if self.enable_smartdashboard:
            self._RPS = SmartDashboard.getNumber(f"{self.name} Target RPM", 0) / 60

            value_changed = (
                (self.cfg.slot0.k_p != SmartDashboard.getNumber(f"{self.name} k_p", 0))
                or (
                    self.cfg.slot0.k_i
                    != SmartDashboard.getNumber(f"{self.name} k_i", 0)
                )
                or (
                    self.cfg.slot0.k_d
                    != SmartDashboard.getNumber(f"{self.name} k_d", 0)
                )
            )

            if value_changed:
                self.cfg.slot0.k_p = SmartDashboard.getNumber(f"{self.name} k_p", 0)
                self.cfg.slot0.k_i = SmartDashboard.getNumber(f"{self.name} k_i", 0)
                self.cfg.slot0.k_d = SmartDashboard.getNumber(f"{self.name} k_d", 0)
                self._motor.configurator.apply(self.cfg)

    def simulationPeriodic(self):
        self._motor.sim_state.set_supply_voltage(RobotController.getBatteryVoltage())
        self._flywheel_sim.setInputVoltage(self._motor.sim_state.motor_voltage)
        self._flywheel_sim.update(0.02)
        sim_rps = self._flywheel_sim.getAngularVelocity() / (2 * math.pi)
        self._motor.sim_state.set_rotor_velocity(sim_rps)
        self._motor.sim_state.add_rotor_position(sim_rps * 0.02)
