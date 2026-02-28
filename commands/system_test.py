"""
System Test Command for Pit Testing

This command runs a comprehensive test of all robot subsystems.
It should be run with the robot on a stand where wheels don't touch anything,
intake has room to deploy, and shooter/hopper is empty.

The test will:
1. Test the intake by deploying and monitoring spinner RPM
2. Test each flywheel motor by spinning and monitoring RPM
3. Verify vision system is responding
4. Check for resistance that indicates safety issues (ball in hopper, blocked intake)
5. Report results via SmartDashboard
"""

import time

import commands2
from wpilib import SmartDashboard

from constants.intake import kIntakeMotor
from subsystems.controlled_motor import ControlledTalonMotor
from subsystems.intake import IntakeSubsystem


class SystemTestCommand(commands2.Command):
    """
    Sequential system test for all robot subsystems.
    Tests motors, vision, and safety systems.
    """

    MOTOR_TEST_DURATION = 2.0   # seconds to run each motor
    RPM_TOLERANCE = 0.2          # allow 20% below target before flagging
    MIN_RPM_THRESHOLD = 100      # minimum RPM to consider a motor working
    STALL_CHECK_DELAY = 0.5      # seconds after start before checking for stall

    def __init__(
        self,
        intake: IntakeSubsystem,
        shooter_motor: ControlledTalonMotor,
        spindex_motor: ControlledTalonMotor,
        transfer_motor1: ControlledTalonMotor,
        transfer_motor2: ControlledTalonMotor,
        vision_subsystem,
    ):
        super().__init__()

        self._intake = intake
        self._shooter = shooter_motor
        self._spindex = spindex_motor
        self._transfer1 = transfer_motor1
        self._transfer2 = transfer_motor2
        self._vision = vision_subsystem

        self.addRequirements(
            intake,
            shooter_motor,
            spindex_motor,
            transfer_motor1,
            transfer_motor2,
            vision_subsystem,
        )

        self._test_stage = 0
        self._stage_start_time = 0.0
        self._test_passed = True
        self._failure_message = ""

    def initialize(self):
        self._test_stage = 0
        self._stage_start_time = time.time()
        self._test_passed = True
        self._failure_message = ""

        SmartDashboard.putString("System Test/Status", "Starting...")
        SmartDashboard.putString("System Test/Stage", "Initialization")
        SmartDashboard.putBoolean("System Test/Passed", False)
        SmartDashboard.putString("System Test/Failure", "")

    def execute(self):
        elapsed = time.time() - self._stage_start_time

        # Stage 0: Intake — deploy and check spinner RPM
        if self._test_stage == 0:
            SmartDashboard.putString("System Test/Stage", "Testing Intake")
            if elapsed < self.MOTOR_TEST_DURATION:
                self._intake.deploy()
                if elapsed > self.STALL_CHECK_DELAY:
                    rpm = self._intake.get_spinner_rps() * 60
                    if rpm < self.MIN_RPM_THRESHOLD:
                        self._fail("Intake spinner stalled — possible obstruction")
                        self._intake.undeploy()
            else:
                rpm = self._intake.get_spinner_rps() * 60
                target_rpm = kIntakeMotor.TARGET_RPM
                if rpm < target_rpm * (1 - self.RPM_TOLERANCE):
                    self._fail(
                        f"Intake spinner below target RPM (got {rpm:.0f}, expected ~{target_rpm:.0f})"
                    )
                self._intake.undeploy()
                self._advance_stage()

        # Stage 1: Shooter motor
        elif self._test_stage == 1:
            self._test_motor(self._shooter, "Shooter", elapsed)

        # Stage 2: Spindex motor
        elif self._test_stage == 2:
            self._test_motor(self._spindex, "Spindex", elapsed)

        # Stage 3: Transfer motor 1
        elif self._test_stage == 3:
            self._test_motor(self._transfer1, "Transfer 1", elapsed)

        # Stage 4: Transfer motor 2
        elif self._test_stage == 4:
            self._test_motor(self._transfer2, "Transfer 2", elapsed)

        # Stage 5: Vision system
        elif self._test_stage == 5:
            SmartDashboard.putString("System Test/Stage", "Testing Vision")
            if elapsed > 1.0:
                robot_unflatness = SmartDashboard.getNumber("Robot Unflatness", -1)
                if robot_unflatness == -1:
                    self._fail("Vision system not responding")
                self._advance_stage()

        # Stage 6: Complete
        elif self._test_stage == 6:
            SmartDashboard.putString("System Test/Stage", "Complete")

    def _test_motor(self, motor: ControlledTalonMotor, label: str, elapsed: float):
        SmartDashboard.putString("System Test/Stage", f"Testing {label}")
        if elapsed < self.MOTOR_TEST_DURATION:
            motor.spin()
            if elapsed > self.STALL_CHECK_DELAY:
                if motor.get_actual_rps() * 60 < self.MIN_RPM_THRESHOLD:
                    self._fail(f"{label} motor stalled — possible obstruction")
                    motor.stop_motor()
        else:
            actual_rpm = motor.get_actual_rps() * 60
            target_rpm = motor.target_rps * 60
            if actual_rpm < target_rpm * (1 - self.RPM_TOLERANCE):
                self._fail(
                    f"{label} motor below target RPM (got {actual_rpm:.0f}, expected ~{target_rpm:.0f})"
                )
            motor.stop_motor()
            self._advance_stage()

    def _advance_stage(self):
        self._test_stage += 1
        self._stage_start_time = time.time()

    def _fail(self, message: str):
        self._test_passed = False
        self._failure_message = message

    def isFinished(self) -> bool:
        if not self._test_passed:
            SmartDashboard.putBoolean("System Test/Passed", False)
            SmartDashboard.putString("System Test/Failure", self._failure_message)
            SmartDashboard.putString(
                "System Test/Status", f"FAILED: {self._failure_message}"
            )
            return True

        if self._test_stage >= 6:
            SmartDashboard.putBoolean("System Test/Passed", True)
            SmartDashboard.putString("System Test/Status", "PASSED — All systems nominal")
            return True

        return False

    def end(self, interrupted: bool):
        self._intake.undeploy()
        self._shooter.stop_motor()
        self._spindex.stop_motor()
        self._transfer1.stop_motor()
        self._transfer2.stop_motor()

        if interrupted:
            SmartDashboard.putString("System Test/Status", "INTERRUPTED")
            SmartDashboard.putBoolean("System Test/Passed", False)
