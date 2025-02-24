from wpilib import SmartDashboard
from networktables import NetworkTables
from wpimath.geometry import Pose2d, Rotation2d
from commands2 import Subsystem


class LimeLightSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        NetworkTables.initialize()  # Initialize NetworkTables
        self.table = NetworkTables.getTable("limelight")  # Get the LimeLight NetworkTable

    def get_apriltag_pose(self):
        """Retrieve AprilTag position data from LimeLight."""
        tid = self.table.getNumber("tid", -1)  # Target ID (-1 if no tag)
        botpose = self.table.getNumberArray("botpose", [0] * 6)  # Robot pose relative to tag

        if tid == -1:  # No tag detected
            return None

        # Convert to Pose2d object
        return Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]))

    def periodic(self):
        """Update telemetry on SmartDashboard."""
        tag_pose = self.get_apriltag_pose()
        if tag_pose:
            SmartDashboard.putNumber("Tag X", tag_pose.X())
            SmartDashboard.putNumber("Tag Y", tag_pose.Y())
            SmartDashboard.putNumber("Tag Rotation", tag_pose.rotation().degrees())
        else:
            SmartDashboard.putString("Tag Status", "No Tag Detected")