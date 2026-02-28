from phoenix6.configs import TalonFXConfiguration, Slot0Configs


class kIntakeDeployer:
    _CONFIG = TalonFXConfiguration()
    _CONFIG.slot0.k_p = 0.4
    _CONFIG.slot0.k_i = 0
    _CONFIG.slot0.k_d = 0

    _CONFIG.slot1.k_p = 0.1
    _CONFIG.slot1.k_i = 0
    _CONFIG.slot1.k_d = 0

    LEFT_CAN_ID = 0
    RIGHT_CAN_ID = 0

    INITIAL_POSITION = 0
    DEPLOYED_POSITION = 0.5

    # Simulation constants — PLACEHOLDER values; tune to match physical robot
    GEAR_RATIO = 12.8       # Motor rotations per arm output shaft rotation
    ARM_LENGTH_M = 0.4      # Arm pivot-to-end length in meters
    ARM_MASS_KG = 1.5       # Arm mass in kg
    MIN_ANGLE_DEG = 0.0     # Stowed position angle in degrees
    MAX_ANGLE_DEG = 90.0    # Deployed angle in degrees


class kIntakeMotor:
    _CONFIG = TalonFXConfiguration()
    _CONFIG.slot0.k_p = 0
    _CONFIG.slot0.k_i = 6
    _CONFIG.slot0.k_d = 0

    CAN_ID = 22
    TARGET_RPS = 1
    TARGET_RPM = -3600
