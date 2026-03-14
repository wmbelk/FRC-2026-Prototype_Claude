from wpimath.geometry import Pose2d, Rotation2d, Transform2d
from util.flip_util import FlipUtil

class RobotZoneChecker:
    '''Cool utility that checks if a pose is within a certain zone (using opposite coordinates that make a rectangle)'''
    
    # All of these are in meters
    field_height = 8.07
    field_width = 16.54
    
    right_allianze_zone_x = 3.75
    
    left_neutral_zone_x = 5.40
    right_neutral_zone_x = 11.30
    
    @staticmethod
    def is_between(value, num1, num2):
        return (value > num1 and value < num2) or (value > num2 and value < num1)
    
    @staticmethod
    def is_within_pose(pose : Pose2d, first_coordinate : Pose2d, opposite_coordinate : Pose2d) -> bool:
        return (
            RobotZoneChecker.is_between(pose.X(), first_coordinate.X(), opposite_coordinate.X())
        and RobotZoneChecker.is_between(pose.Y(), first_coordinate.Y(), opposite_coordinate.Y())
        )
        
    @staticmethod
    def is_in_neutral_zone(pose : Pose2d):
        return RobotZoneChecker.is_within_pose(
            pose, 
            FlipUtil.fieldPose(Pose2d(RobotZoneChecker.left_neutral_zone_x, 0, Rotation2d())),
            FlipUtil.fieldPose(Pose2d(RobotZoneChecker.right_neutral_zone_x, RobotZoneChecker.field_height, Rotation2d())),
        )
    
    @staticmethod
    def is_in_alliance_zone(pose : Pose2d):
        return RobotZoneChecker.is_within_pose(
            pose, 
            FlipUtil.fieldPose(Pose2d(0, 0, Rotation2d())),
            FlipUtil.fieldPose(Pose2d(RobotZoneChecker.right_allianze_zone_x, RobotZoneChecker.field_height, Rotation2d())),
        )
    
    @staticmethod
    def is_in_left_neutral_zone(pose : Pose2d):
        return RobotZoneChecker.is_within_pose(
            pose, 
            FlipUtil.fieldPose(Pose2d(RobotZoneChecker.left_neutral_zone_x, RobotZoneChecker.field_height / 2, Rotation2d())),
            FlipUtil.fieldPose(Pose2d(RobotZoneChecker.right_neutral_zone_x, RobotZoneChecker.field_height, Rotation2d())),
        )
    
    @staticmethod
    def is_in_right_neutral_zone(pose : Pose2d):
        return RobotZoneChecker.is_within_pose(
            pose,
            FlipUtil.fieldPose(Pose2d(RobotZoneChecker.left_neutral_zone_x, 0, Rotation2d())),
            FlipUtil.fieldPose(Pose2d(RobotZoneChecker.right_neutral_zone_x, RobotZoneChecker.field_height / 2, Rotation2d())),
        )

    @staticmethod
    def is_projected_in_danger_zone(
        pose: Pose2d,
        velocity: Transform2d,
        lookahead_seconds: float,
        zone_x_margin: float,
        hub_x: float,
    ) -> bool:
        """Returns True if the robot's projected position is within zone_x_margin of hub_x in the x direction.

        Projects the current pose forward by (velocity * lookahead_seconds) and checks whether
        the resulting x coordinate falls inside the danger band [hub_x - margin, hub_x + margin].
        """
        projected_pose = pose.transformBy(velocity * lookahead_seconds)
        return abs(projected_pose.X() - hub_x) <= zone_x_margin