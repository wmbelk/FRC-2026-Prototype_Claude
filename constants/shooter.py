from phoenix6.configs import TalonFXConfiguration
from wpimath.geometry import Transform2d, Rotation2d

class kShooterMotor:
    CAN_ID = 24
    _CONFIG = TalonFXConfiguration()
    _CONFIG.slot0.k_p = 0.1
    _CONFIG.slot0.k_i = 0.15
    _CONFIG.slot0.k_d = 0
    TARGET_RPM = -2500

class kShooterConfig:
    SHOOTER_OFFSET = Transform2d(-0.029566, -0.212725, Rotation2d())  # Meters
    SHOOTER_DIRECTION = 0 # 180 for Reverse

class kHoodMotor:
    CAN_ID = 0           # TODO: confirm CAN ID of the Minion motor
    GEAR_RATIO = 20.0    # TODO: motor rotations per output shaft revolution — needs physical measurement

    MIN_ANGLE_DEG = 10.0  # Degrees above horizontal — must match ShooterHood._min_angle
    MAX_ANGLE_DEG = 60.0  # Degrees above horizontal — must match ShooterHood._max_angle

    # Physical properties used in simulation
    ARM_LENGTH_M = 0.15   # TODO: approximate hood pivot-to-end length in meters
    ARM_MASS_KG = 0.3     # TODO: approximate hood mass in kg

    # PID — tune on robot; start conservative
    _CONFIG = TalonFXConfiguration()
    _CONFIG.slot0.k_p = 1.0   # TODO: tune
    _CONFIG.slot0.k_i = 0.0
    _CONFIG.slot0.k_d = 0.1
    