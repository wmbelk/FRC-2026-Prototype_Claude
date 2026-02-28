import math

import commands2
import phoenix6
import wpilib
from wpilib import RobotController, SmartDashboard
from wpilib.simulation import SingleJointedArmSim
from wpimath.system.plant import DCMotor
from wpimath.units import degreesToRadians, radiansToDegrees

from constants.shooter import kHoodMotor


class ShooterHood(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        # Physics constants
        self._gravity = 9.81
        self._hub_height = 1.8288      # meters — target height above ground
        self._shooter_height = 0.3048  # meters — shooter pivot height above ground
        self._ball_weight = 0.215      # kg (unused in angle calc but kept for reference)

        self._min_angle = degreesToRadians(kHoodMotor.MIN_ANGLE_DEG)
        self._max_angle = degreesToRadians(kHoodMotor.MAX_ANGLE_DEG)

        self._efficiency_factor = 0.9
        self.with_RPS(60)  # initialize _exit_velocity

        # Motor
        self._motor = phoenix6.hardware.TalonFX(kHoodMotor.CAN_ID, "rio")
        self._cfg = kHoodMotor._CONFIG

        for _ in range(5):
            status = self._motor.configurator.apply(self._cfg)
            if status.is_ok():
                break

        self._position_voltage = phoenix6.controls.PositionVoltage(position=0, slot=0)

        SmartDashboard.putNumber("Hood/Target Angle deg", 0.0)
        SmartDashboard.putNumber("Hood/Actual Angle deg", 0.0)

        if wpilib.RobotBase.isSimulation():
            _model = DCMotor.krakenX60(1)
            self._arm_sim = SingleJointedArmSim(
                gearbox=_model,
                gearing=kHoodMotor.GEAR_RATIO,
                moi=SingleJointedArmSim.estimateMOI(
                    kHoodMotor.ARM_LENGTH_M, kHoodMotor.ARM_MASS_KG
                ),
                armLength=kHoodMotor.ARM_LENGTH_M,
                minAngle=self._min_angle,
                maxAngle=self._max_angle,
                simulateGravity=True,
                startingAngle=self._min_angle,
            )

    # ------------------------------------------------------------------
    # Configuration helpers
    # ------------------------------------------------------------------

    def with_RPS(self, rps: float) -> "ShooterHood":
        """Update exit velocity from actual shooter RPS. Returns self for chaining."""
        self._exit_velocity = rps * (2 * math.pi) * self._efficiency_factor
        return self

    # ------------------------------------------------------------------
    # Physics
    # ------------------------------------------------------------------

    def calculate_hood_angle(self, distance: float) -> float:
        """Return the optimal hood angle (radians) to reach the hub at the given distance."""
        delta_h = self._hub_height - self._shooter_height
        v = self._exit_velocity
        d = distance
        g = self._gravity

        discriminant = v**4 - g * (g * d**2 + 2 * delta_h * v**2)

        if discriminant < 0:
            return self._max_angle

        sqrt_term = math.sqrt(discriminant)
        theta = math.atan((v**2 - sqrt_term) / (g * d))

        return max(self._min_angle, min(self._max_angle, theta))

    # ------------------------------------------------------------------
    # Control
    # ------------------------------------------------------------------

    def angle_hood(self, distance: float):
        """Calculate and apply the hood angle for the given horizontal distance to target."""
        target_angle = self.calculate_hood_angle(distance)
        self.set_angle(target_angle)

    def set_angle(self, angle_radians: float):
        """Drive the hood motor to the given angle (radians above horizontal)."""
        angle_clamped = max(self._min_angle, min(self._max_angle, angle_radians))
        # Convert radians to motor rotations via gear ratio
        motor_rotations = angle_clamped * kHoodMotor.GEAR_RATIO / (2 * math.pi)
        self._motor.set_control(self._position_voltage.with_position(motor_rotations))
        SmartDashboard.putNumber("Hood/Target Angle deg", radiansToDegrees(angle_clamped))

    def get_actual_angle_radians(self) -> float:
        """Return the current hood angle in radians derived from motor position."""
        motor_pos = self._motor.get_position().value  # rotations
        return motor_pos * (2 * math.pi) / kHoodMotor.GEAR_RATIO

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    def periodic(self):
        SmartDashboard.putNumber(
            "Hood/Actual Angle deg", radiansToDegrees(self.get_actual_angle_radians())
        )

    # ------------------------------------------------------------------
    # Simulation
    # ------------------------------------------------------------------

    def simulationPeriodic(self):
        self._motor.sim_state.set_supply_voltage(RobotController.getBatteryVoltage())
        self._arm_sim.setInputVoltage(self._motor.sim_state.motor_voltage)
        self._arm_sim.update(0.02)

        arm_rad = self._arm_sim.getAngle()
        arm_rps_motor = (self._arm_sim.getVelocity() / (2 * math.pi)) * kHoodMotor.GEAR_RATIO
        motor_pos = (arm_rad / (2 * math.pi)) * kHoodMotor.GEAR_RATIO

        self._motor.sim_state.set_rotor_position(motor_pos)
        self._motor.sim_state.set_rotor_velocity(arm_rps_motor)

        # Soft limits at physical stops
        tol = degreesToRadians(1.0)
        self._motor.sim_state.set_forward_limit(arm_rad >= self._max_angle - tol)
        self._motor.sim_state.set_reverse_limit(arm_rad <= self._min_angle + tol)
