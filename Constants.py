import math
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfile
from wpimath import units

class DriveConstants:
    # Driving Parameters
    kMaxSpeedMetersPerSecond = 4.8
    kMaxAngularSpeed = 2 * math.pi  # radians per second

    # Chassis configuration
    kTrackWidth = units.inchesToMeters(26.5)
    kWheelBase = units.inchesToMeters(26.5)

    kDriveKinematics = SwerveDrive4Kinematics(
        Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    )

    # Angular offsets of the modules relative to the chassis in radians
    kFrontLeftChassisAngularOffset = -math.pi / 2
    kFrontRightChassisAngularOffset = 0
    kBackLeftChassisAngularOffset = math.pi
    kBackRightChassisAngularOffset = math.pi / 2

    # SPARK MAX CAN IDs
    kFrontLeftDrivingCanId = 11
    kRearLeftDrivingCanId = 13
    kFrontRightDrivingCanId = 15
    kRearRightDrivingCanId = 17

    kFrontLeftTurningCanId = 10
    kRearLeftTurningCanId = 12
    kFrontRightTurningCanId = 14
    kRearRightTurningCanId = 16

    kGyroReversed = False

class ModuleConstants:
    # The MAXSwerve module can be configured with one of three pinion gears
    kDrivingMotorPinionTeeth = 14

    # Calculations required for driving motor conversion factors and feed forward
    kDrivingMotorFreeSpeedRps = 5676 / 60  # Assuming the RPM constant from NeoMotorConstants
    kWheelDiameterMeters = 0.0762
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi

    # Gear ratios
    kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
    kDriveWheelVelocityCF = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction
    kDriveWheelPositionCF = kDrivingMotorReduction * kWheelDiameterMeters

class OIConstants:
    kDriverControllerPort = 0
    kDriveDeadband = 0.05

class AutoConstants:
    kMaxSpeedMetersPerSecond = 3
    kMaxAccelerationMetersPerSecondSquared = 3
    kMaxAngularSpeedRadiansPerSecond = math.pi
    kMaxAngularSpeedRadiansPerSecondSquared = math.pi

    kPXController = 1
    kPYController = 1
    kPThetaController = 1

    # Constraint for the motion profiled robot angle controller
    kThetaControllerConstraints = TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
    )

class NeoMotorConstants:
    kFreeSpeedRpm = 5676