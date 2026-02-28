# import commands2
# import phoenix6
# from phoenix6.controls import Follower
# from phoenix6 import signals

# from wpilib import SmartDashboard, DigitalInput
# from constants.intake import kIntakeSpinner, kIntakeDeployer
# from util.editable_pid import EditablePID

# class IntakeSubsystem(commands2.Subsystem):
#     def __init__(self):
#         super().__init__()

#         self.left_deployer = phoenix6.hardware.TalonFX(kIntakeDeployer.LEFT_CAN_ID, "rio")
#         self.right_deployer = phoenix6.hardware.TalonFX(kIntakeDeployer.RIGHT_CAN_ID, "rio")
#         self.spinny_motor = phoenix6.hardware.TalonFX(kIntakeSpinner.CAN_ID, "rio")
        
#         self.deployer_cfg = kIntakeDeployer._CONFIG
#         self.spinny_cfg = kIntakeSpinner._CONFIG 
#         self.spinny_cfg.motor_output.neutral_mode = signals.NeutralModeValue.COAST

#         self.left_deployer.configurator.apply(self.deployer_cfg)
#         # self.right_deployer.configurator.apply(self.deployer_cfg) # might need this
#         self.spinny_motor.configurator.apply(self.spinny_cfg)
        
#         self.right_deployer.set_control(
#             Follower(
#                 self.left_deployer.device_id,                                 
#                 motor_alignment = signals.spn_enums.MotorAlignmentValue.OPPOSED
#             )
#         )
        
#         self.deployer_position_voltage = phoenix6.controls.PositionVoltage(position=0, slot=0)
#         self.velocity_voltage = phoenix6.controls.VelocityVoltage(velocity=0, slot=0)
        
#         self.state : str = "undeployed"
        
#         self.deploy_editable_pid = EditablePID("Intake/Deployer", self.left_deployer, self.deployer_cfg, use_slot1=True)
#         self.spinny_editable_pid = EditablePID("Intake/Spinny", self.spinny_motor, self.spinny_cfg)
        
#         SmartDashboard.putNumber("Intake/Deploy Initial Position", kIntakeDeployer.INITIAL_POSITION)
#         SmartDashboard.putNumber("Intake/Deploy Active Position", kIntakeDeployer.DEPLOYED_POSITION)
#         SmartDashboard.putNumber("Intake/Spinny Speed", kIntakeSpinner.TARGET_RPS)
    
#     def deploy(self):
#         self.spinny_motor.set_control(self.velocity_voltage.with_velocity(kIntakeSpinner.TARGET_RPS))
        
#         self.left_deployer.set_control(
#             self.deployer_position_voltage
#             .with_position(kIntakeDeployer.DEPLOYED_POSITION)
#             .with_slot(0)
#         )
#         self.state = "deploying"
    
#     def undeploy(self):
#         self.spinny_motor.disable()
        
#         self.left_deployer.set_control(
#             self.deployer_position_voltage
#             .with_position(kIntakeDeployer.INITIAL_POSITION)
#             .with_slot(0)
#         )
#         self.state = "undeploying"
    
#     def periodic(self):
#         # This only detects left limit switches for now but it still should work ideally
        
#         forward_limit = self.left_deployer.get_forward_limit()
#         if forward_limit.value is signals.ForwardLimitValue.CLOSED_TO_GROUND and self.state == "deploying":
#             self.left_deployer.set_control(self.deployer_position_voltage.with_slot(1))
#             self.state = "deployed"
        
#         reverse_limit = self.left_deployer.get_reverse_limit()
#         if reverse_limit.value is signals.ForwardLimitValue.CLOSED_TO_GROUND and self.state == "undeploying":
#             self.left_deployer.set_control(self.deployer_position_voltage.with_slot(1))
#             self.state = "undeployed"
            
#         SmartDashboard.putString("Intake/State", self.state)
#         kIntakeDeployer.INITIAL_POSITION = SmartDashboard.getNumber("Intake/Deploy Initial Position", kIntakeDeployer.INITIAL_POSITION)
#         kIntakeDeployer.DEPLOYED_POSITION = SmartDashboard.getNumber("Intake/Deploy Active Position", kIntakeDeployer.DEPLOYED_POSITION)
#         kIntakeSpinner.TARGET_RPS = SmartDashboard.getNumber("Intake/Spinny Speed", kIntakeSpinner.TARGET_RPS)
        
#         self.deploy_editable_pid.periodic()
#         self.spinny_editable_pid.periodic()

# --- SIMULATION TEMPLATE ---
# When re-enabling this subsystem, add these imports at the top:
#
#   import wpilib
#   from wpilib import RobotController
#   from wpilib.simulation import FlywheelSim, SingleJointedArmSim
#   from wpimath.system.plant import DCMotor, LinearSystemId
#   from wpimath.units import degreesToRadians, radiansToDegrees
#   import math
#
# Add to __init__ after hardware config:
#
#         if wpilib.RobotBase.isSimulation():
#             _spinner_model = DCMotor.krakenX60(1)
#             self._spinner_sim = FlywheelSim(LinearSystemId.flywheelSystem(_spinner_model, 0.001, 1.0), _spinner_model)
#             self._arm_sim = SingleJointedArmSim(
#                 gearbox=DCMotor.krakenX60(1),
#                 gearing=kIntakeDeployer.GEAR_RATIO,
#                 moi=SingleJointedArmSim.estimateMOI(kIntakeDeployer.ARM_LENGTH_M, kIntakeDeployer.ARM_MASS_KG),
#                 armLength=kIntakeDeployer.ARM_LENGTH_M,
#                 minAngle=degreesToRadians(kIntakeDeployer.MIN_ANGLE_DEG),
#                 maxAngle=degreesToRadians(kIntakeDeployer.MAX_ANGLE_DEG),
#                 simulateGravity=True,
#                 startingAngle=degreesToRadians(0.0),
#             )
#             self._mech = wpilib.Mechanism2d(2.0, 2.0)
#             root = self._mech.getRoot("IntakeArm", 1.0, 0.1)
#             self._arm_ligament = root.appendLigament(
#                 "Arm", kIntakeDeployer.ARM_LENGTH_M, 0.0,
#                 lineWidth=6.0, color=wpilib.Color8Bit(wpilib.Color.kOrange)
#             )
#             wpilib.SmartDashboard.putData("Intake Mechanism", self._mech)
#
# Add simulationPeriodic() method:
#
#     def simulationPeriodic(self):
#         supply_v = RobotController.getBatteryVoltage()
#
#         # Spinner
#         self.spinny_motor.sim_state.set_supply_voltage(supply_v)
#         self._spinner_sim.setInputVoltage(self.spinny_motor.sim_state.motor_voltage)
#         self._spinner_sim.update(0.02)
#         spinner_rps = self._spinner_sim.getAngularVelocity() / (2 * math.pi)
#         self.spinny_motor.sim_state.set_rotor_velocity(spinner_rps)
#         self.spinny_motor.sim_state.add_rotor_position(spinner_rps * 0.02)
#
#         # Deployer arm
#         self.left_deployer.sim_state.set_supply_voltage(supply_v)
#         self._arm_sim.setInputVoltage(self.left_deployer.sim_state.motor_voltage)
#         self._arm_sim.update(0.02)
#         arm_rad = self._arm_sim.getAngle()
#         arm_rps_motor = (self._arm_sim.getVelocity() / (2 * math.pi)) * kIntakeDeployer.GEAR_RATIO
#         motor_pos = (arm_rad / (2 * math.pi)) * kIntakeDeployer.GEAR_RATIO
#         self.left_deployer.sim_state.set_rotor_position(motor_pos)
#         self.left_deployer.sim_state.set_rotor_velocity(arm_rps_motor)
#         # Mirror to right deployer (Follower doesn't propagate through TalonFXSimState)
#         self.right_deployer.sim_state.set_supply_voltage(supply_v)
#         self.right_deployer.sim_state.set_rotor_position(motor_pos)
#         self.right_deployer.sim_state.set_rotor_velocity(arm_rps_motor)
#
#         # Limit switches (Phoenix 6 sets limits on TalonFXSimState directly, not DIOSim)
#         tol = degreesToRadians(1.0)
#         self.left_deployer.sim_state.set_forward_limit(arm_rad >= self._arm_sim.maxAngle - tol)
#         self.left_deployer.sim_state.set_reverse_limit(arm_rad <= self._arm_sim.minAngle + tol)
#
#         # Update Mechanism2d arm angle for Glass visualization
#         self._arm_ligament.setAngle(radiansToDegrees(arm_rad))
