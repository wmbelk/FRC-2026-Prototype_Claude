import commands2
import phoenix6
from phoenix6.controls import Follower
from phoenix6 import signals

from wpilib import SmartDashboard, DigitalInput
from constants.intake import kIntakeDeployer, kIntakeMotor
from util.editable_pid import EditablePID


class IntakeSubsystem(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        # TODO: Set correct CAN IDs in constants/intake.py before enabling
        # kIntakeDeployer.LEFT_CAN_ID and kIntakeDeployer.RIGHT_CAN_ID are currently 0
        self.left_deployer = phoenix6.hardware.TalonFX(kIntakeDeployer.LEFT_CAN_ID, "rio")
        self.right_deployer = phoenix6.hardware.TalonFX(kIntakeDeployer.RIGHT_CAN_ID, "rio")

        self.deployer_cfg = kIntakeDeployer._CONFIG

        self.left_deployer.configurator.apply(self.deployer_cfg)

        self.right_deployer.set_control(
            Follower(
                self.left_deployer.device_id,
                motor_alignment=signals.spn_enums.MotorAlignmentValue.OPPOSED,
            )
        )

        self.deployer_position_voltage = phoenix6.controls.PositionVoltage(position=0, slot=0)

        self.state: str = "undeployed"

        self.deploy_editable_pid = EditablePID(
            "Intake/Deployer", self.left_deployer, self.deployer_cfg, use_slot1=True
        )

        SmartDashboard.putNumber("Intake/Deploy Initial Position", kIntakeDeployer.INITIAL_POSITION)
        SmartDashboard.putNumber("Intake/Deploy Active Position", kIntakeDeployer.DEPLOYED_POSITION)

    def deploy(self):
        self.left_deployer.set_control(
            self.deployer_position_voltage
            .with_position(kIntakeDeployer.DEPLOYED_POSITION)
            .with_slot(0)  # slot0: stiff PID for active positioning
        )
        self.state = "deploying"

    def undeploy(self):
        self.left_deployer.set_control(
            self.deployer_position_voltage
            .with_position(kIntakeDeployer.INITIAL_POSITION)
            .with_slot(0)  # slot0: stiff PID for active retraction
        )
        self.state = "undeploying"

    def periodic(self):
        # When the arm reaches the forward (down/deployed) limit, switch to soft hold PID.
        # slot1 (k_p=0.1) is compliant — the arm yields when hit by another robot
        # rather than fighting back and potentially breaking.
        forward_limit = self.left_deployer.get_forward_limit()
        if forward_limit.value is signals.ForwardLimitValue.CLOSED_TO_GROUND and self.state == "deploying":
            self.left_deployer.set_control(
                self.deployer_position_voltage
                .with_position(kIntakeDeployer.DEPLOYED_POSITION)
                .with_slot(1)  # slot1: soft hold PID for compliance at deployed position
            )
            self.state = "deployed"

        # When the arm reaches the reverse (up/stowed) limit, switch to soft hold PID.
        reverse_limit = self.left_deployer.get_reverse_limit()
        if reverse_limit.value is signals.ReverseLimitValue.CLOSED_TO_GROUND and self.state == "undeploying":
            self.left_deployer.set_control(
                self.deployer_position_voltage
                .with_position(kIntakeDeployer.INITIAL_POSITION)
                .with_slot(1)  # slot1: soft hold at stowed position
            )
            self.state = "undeployed"

        SmartDashboard.putString("Intake/State", self.state)
        kIntakeDeployer.INITIAL_POSITION = SmartDashboard.getNumber(
            "Intake/Deploy Initial Position", kIntakeDeployer.INITIAL_POSITION
        )
        kIntakeDeployer.DEPLOYED_POSITION = SmartDashboard.getNumber(
            "Intake/Deploy Active Position", kIntakeDeployer.DEPLOYED_POSITION
        )

        self.deploy_editable_pid.periodic()
