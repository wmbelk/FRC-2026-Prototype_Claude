"""
Unit tests for SystemTestCommand.

All subsystem dependencies are replaced with unittest.mock.MagicMock so the
tests run without WPILib or Phoenix 6 hardware initialization.
"""

from unittest.mock import MagicMock, patch, call
import pytest

# Patch wpilib.SmartDashboard before importing the command so the import
# itself doesn't try to initialize HAL.
import sys
import types

# Minimal stubs so the import chain resolves without HAL
_wpilib = types.ModuleType("wpilib")
_wpilib.SmartDashboard = MagicMock()
sys.modules.setdefault("wpilib", _wpilib)

_commands2 = types.ModuleType("commands2")
_commands2.Command = object   # base class — enough for instantiation tests
_commands2.Subsystem = object
sys.modules.setdefault("commands2", _commands2)

# Stub out phoenix6 and constants so imports resolve
for _mod in [
    "phoenix6", "phoenix6.hardware", "phoenix6.controls", "phoenix6.configs",
    "phoenix6.signals", "phoenix6.signals.spn_enums",
]:
    sys.modules.setdefault(_mod, types.ModuleType(_mod))

_const_intake = types.ModuleType("constants.intake")
_const_intake.kIntakeMotor = MagicMock()
_const_intake.kIntakeMotor.TARGET_RPM = 1000
_const_intake.kIntakeDeployer = MagicMock()
sys.modules["constants.intake"] = _const_intake
sys.modules.setdefault("constants", types.ModuleType("constants"))

_sub_motor = types.ModuleType("subsystems.controlled_motor")
_sub_motor.ControlledTalonMotor = object
sys.modules["subsystems.controlled_motor"] = _sub_motor

_sub_intake = types.ModuleType("subsystems.intake")
_sub_intake.IntakeSubsystem = object
sys.modules["subsystems.intake"] = _sub_intake

sys.modules.setdefault("subsystems", types.ModuleType("subsystems"))

# Now import the command under test
from commands.system_test import SystemTestCommand


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_motor(target_rps=100 / 60, actual_rps=None):
    """Return a mock ControlledTalonMotor."""
    m = MagicMock()
    m.target_rps = target_rps
    m.get_actual_rps.return_value = actual_rps if actual_rps is not None else target_rps
    return m


def _make_intake(spinner_rps=None):
    """Return a mock IntakeSubsystem."""
    m = MagicMock()
    target = _const_intake.kIntakeMotor.TARGET_RPM / 60
    m.get_spinner_rps.return_value = spinner_rps if spinner_rps is not None else target
    return m


def _make_cmd(intake=None, shooter=None, spindex=None, t1=None, t2=None):
    """Build a SystemTestCommand with all subsystems mocked."""
    cmd = SystemTestCommand.__new__(SystemTestCommand)
    cmd._intake = intake or _make_intake()
    cmd._shooter = shooter or _make_motor()
    cmd._spindex = spindex or _make_motor()
    cmd._transfer1 = t1 or _make_motor()
    cmd._transfer2 = t2 or _make_motor()
    cmd._vision = MagicMock()
    cmd._test_stage = 0
    cmd._stage_start_time = 0.0
    cmd._test_passed = True
    cmd._failure_message = ""
    return cmd


# ---------------------------------------------------------------------------
# initialize()
# ---------------------------------------------------------------------------

class TestInitialize:
    def test_resets_stage_and_pass_flag(self):
        cmd = _make_cmd()
        cmd._test_stage = 3
        cmd._test_passed = False
        cmd._failure_message = "old error"

        with patch("time.time", return_value=100.0):
            cmd.initialize()

        assert cmd._test_stage == 0
        assert cmd._test_passed is True
        assert cmd._failure_message == ""

    def test_sets_stage_start_time(self):
        cmd = _make_cmd()
        with patch("time.time", return_value=42.5):
            cmd.initialize()
        assert cmd._stage_start_time == 42.5


# ---------------------------------------------------------------------------
# isFinished()
# ---------------------------------------------------------------------------

class TestIsFinished:
    def test_not_finished_mid_test(self):
        cmd = _make_cmd()
        cmd._test_stage = 2
        assert cmd.isFinished() is False

    def test_finished_when_all_stages_done(self):
        cmd = _make_cmd()
        cmd._test_stage = 6
        assert cmd.isFinished() is True

    def test_finished_on_failure(self):
        cmd = _make_cmd()
        cmd._test_passed = False
        cmd._failure_message = "something broke"
        assert cmd.isFinished() is True

    def test_not_finished_at_stage_5(self):
        cmd = _make_cmd()
        cmd._test_stage = 5
        assert cmd.isFinished() is False


# ---------------------------------------------------------------------------
# _fail() and _advance_stage()
# ---------------------------------------------------------------------------

class TestHelpers:
    def test_fail_sets_flag_and_message(self):
        cmd = _make_cmd()
        cmd._fail("test error")
        assert cmd._test_passed is False
        assert cmd._failure_message == "test error"

    def test_advance_stage_increments(self):
        cmd = _make_cmd()
        cmd._test_stage = 2
        with patch("time.time", return_value=99.0):
            cmd._advance_stage()
        assert cmd._test_stage == 3
        assert cmd._stage_start_time == 99.0


# ---------------------------------------------------------------------------
# _test_motor()
# ---------------------------------------------------------------------------

class TestMotorStages:
    def test_spins_motor_during_test_window(self):
        cmd = _make_cmd()
        motor = _make_motor()
        cmd._test_motor(motor, "Shooter", elapsed=0.3)
        motor.spin.assert_called_once()
        motor.stop_motor.assert_not_called()

    def test_stops_and_advances_after_duration(self):
        cmd = _make_cmd()
        motor = _make_motor(target_rps=50.0, actual_rps=50.0)
        initial_stage = cmd._test_stage

        with patch("time.time", return_value=0.0):
            cmd._test_motor(motor, "Shooter", elapsed=SystemTestCommand.MOTOR_TEST_DURATION + 0.1)

        motor.stop_motor.assert_called_once()
        assert cmd._test_stage == initial_stage + 1

    def test_flags_stall_during_window(self):
        cmd = _make_cmd()
        motor = _make_motor(actual_rps=0.0)  # stalled
        cmd._test_motor(motor, "Shooter", elapsed=SystemTestCommand.STALL_CHECK_DELAY + 0.1)
        assert cmd._test_passed is False
        assert "Shooter" in cmd._failure_message

    def test_flags_low_rpm_at_end_of_window(self):
        target = 50.0
        actual = target * (1 - SystemTestCommand.RPM_TOLERANCE) - 1  # just below threshold
        cmd = _make_cmd()
        motor = _make_motor(target_rps=target, actual_rps=actual)

        with patch("time.time", return_value=0.0):
            cmd._test_motor(motor, "Transfer 1", elapsed=SystemTestCommand.MOTOR_TEST_DURATION + 0.1)

        assert cmd._test_passed is False
        assert "Transfer 1" in cmd._failure_message

    def test_passes_when_rpm_within_tolerance(self):
        target = 50.0
        actual = target * (1 - SystemTestCommand.RPM_TOLERANCE) + 1  # just above threshold
        cmd = _make_cmd()
        motor = _make_motor(target_rps=target, actual_rps=actual)

        with patch("time.time", return_value=0.0):
            cmd._test_motor(motor, "Spindex", elapsed=SystemTestCommand.MOTOR_TEST_DURATION + 0.1)

        assert cmd._test_passed is True


# ---------------------------------------------------------------------------
# end()
# ---------------------------------------------------------------------------

class TestEnd:
    def test_all_motors_stopped_on_normal_end(self):
        shooter = _make_motor()
        spindex = _make_motor()
        t1 = _make_motor()
        t2 = _make_motor()
        intake = _make_intake()
        cmd = _make_cmd(intake=intake, shooter=shooter, spindex=spindex, t1=t1, t2=t2)

        cmd.end(interrupted=False)

        intake.undeploy.assert_called_once()
        shooter.stop_motor.assert_called_once()
        spindex.stop_motor.assert_called_once()
        t1.stop_motor.assert_called_once()
        t2.stop_motor.assert_called_once()

    def test_all_motors_stopped_on_interrupt(self):
        shooter = _make_motor()
        cmd = _make_cmd(shooter=shooter)
        cmd.end(interrupted=True)
        shooter.stop_motor.assert_called_once()
