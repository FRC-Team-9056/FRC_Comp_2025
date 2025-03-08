import math
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfile
from wpimath import units


class CoralSubsystemConstants:
    kElevatorMotorCanId = 4
    kArmMotorCanId = 3
    kIntakeMotorCanId = 2
    
    class ElevatorSetpoints:
        kFeederStation = 0
        kLevel1 = 1
        kLevel2 = 10
        kLevel3 = 100
        kLevel4 = 165

    class ArmSetpoints:
        kFeederStation = 28
        kLevel1 = 120
        kLevel2 = 130
        kLevel3 = 120
        kLevel4 = 105

    class IntakeSetpoints:
        kForward = -0.25
        kReverse = 0.3

class AlgaeSubsystemConstants:
    kIntakeMotorCanId = 5
    kPivotMotorCanId = 6

    class ArmSetpoints:
        kStow = 0
        kHold = 13
        kDown = 18

    class IntakeSetpoints:
        kForward = 0.5
        kReverse = -0.3
        kHold = 0.35

class DriveConstants:
    # Driving Parameters
    kMaxSpeedMetersPerSecond = 2.6
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
    kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction
    kDriveWheelPositionCF = kDrivingMotorReduction * kWheelDiameterMeters

class OIConstants:
    kDriverControllerPort = 0
    kOperatorControllerPort = 1
    kDriveDeadband = 0.02
    kTriggerButtonThreshold = 0.2

class AutoConstants:
    kMaxSpeedMetersPerSecond = 3
    kMaxAccelerationMetersPerSecondSquared = 3
    kMaxAngularAccelerationRadiansPerSecond = 3
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

class SimulationRobotConstants:
        kPixelsPerMeter = 20

        kElevatorGearing = 25  # 25:1
        kCarriageMass = 4.3 + 3.15 + 0.151  # Kg
        kElevatorDrumRadius = 0.0328 / 2.0  # m
        kMinElevatorHeightMeters = 0.922  # m
        kMaxElevatorHeightMeters = 1.62  # m

        kArmReduction = 60  # 60:1
        kArmLength = 0.433  # m
        kArmMass = 4.3  # Kg
        kMinAngleRads = units.degreesToRadians(-50.1)  # Radians for minimum arm angle
        kMaxAngleRads = units.degreesToRadians(40.9 + 180)  # Radians for max arm angle

        kIntakeReduction = 135  # 135:1
        kIntakeLength = 0.4032262  # m
        kIntakeMass = 5.8738  # Kg
        kIntakeMinAngleRads = units.degreesToRadians(80)
        kIntakeMaxAngleRads = units.degreesToRadians(180)
        kIntakeShortBarLength = 0.1524  # m
        kIntakeLongBarLength = 0.3048  # m
        kIntakeBarAngleRads = units.degreesToRadians(-60)  # Radians for intake bar angle