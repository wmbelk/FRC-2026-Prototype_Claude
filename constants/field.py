"""
Holds all of the numeric values set for different subsystems and commands.
This is not for tuning but for known values, such as a position of a field element.
"""

from wpimath.geometry import Translation2d, Pose2d

class kField:
    WIDTH = 8.07
    LENGTH = 16.54

class kHub:
    POS = Pose2d(4.6, 4.0, 0)

class kPassSpots:
    PASS_SPOT_LEFT = Pose2d(2, kField.WIDTH - 1.2, 0)
    PASS_SPOT_RIGHT = Pose2d(2, 1.2, 0)

class kDangerZone:
    """Intake danger zone — the band around the hub and field bumps where the intake should auto-retract."""
    HUB_X = 4.6             # meters — x center of the danger band (hub position)
    DEFAULT_X_MARGIN = 0.5  # meters — half-width of the danger band on each side (NTTable-tunable)
    DEFAULT_LOOKAHEAD_S = 0.5  # seconds — velocity projection window (NTTable-tunable)
    
