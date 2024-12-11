#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

#
# The constants module is a convenience place for teams to hold robot-wide
# numerical or boolean constants. Don't use this for any other purpose!
#

import math
import rev
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfile
from wpimath import units as Units
import lib.PIDGains as PIDGains
    
class OIConstants: 
        kDriverControllerPort = 0
        kDriveDeadband = 0.05
        kTriggerButtonThreshold = 0.5

class NeoMotorConstants:
        kFreeSpeedRpm = 5676

class DriveConstants:
        kMaxSpeedMetersPerSecond = 4.8
        kMaxAngularSpeed = 2 * math.pi  # radians per second

        kDirectionSlewRate = 1.2  # radians per second
        kMagnitudeSlewRate = 1.8  # percent per second (1 = 100%)
        kRotationalSlewRate = 2.0  # percent per second (1 = 100%)

        kTrackWidth = Units.inchesToMeters(21.5)
        kWheelBase = Units.inchesToMeters(21.5)

        kDriveKinematics = SwerveDrive4Kinematics(
            Translation2d(kWheelBase / 2, kTrackWidth / 2),
            Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        )

        kFrontLeftChassisAngularOffset = -math.pi / 2
        kFrontRightChassisAngularOffset = 0
        kBackLeftChassisAngularOffset = math.pi
        kBackRightChassisAngularOffset = math.pi / 2

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
        kDrivingMotorPinionTeeth = 14
        kTurningEncoderInverted = True

        kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60
        kWheelDiameterMeters = 0.0762
        kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
        kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
        kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction

        kDrivingEncoderPositionFactor = (kWheelDiameterMeters * math.pi) / kDrivingMotorReduction
        kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * math.pi) / kDrivingMotorReduction) / 60.0

        kTurningEncoderPositionFactor = (2 * math.pi)
        kTurningEncoderVelocityFactor = (2 * math.pi) / 60.0

        kTurningEncoderPositionPIDMinInput = 0  # radians
        kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor  # radians

        kDrivingP = 0.04
        kDrivingI = 0
        kDrivingD = 0
        kDrivingFF = 1 / kDriveWheelFreeSpeedRps
        kDrivingMinOutput = -1
        kDrivingMaxOutput = 1

        kTurningP = 1
        kTurningI = 0
        kTurningD = 0
        kTurningFF = 0
        kTurningMinOutput = -1
        kTurningMaxOutput = 1

        kDrivingMotorIdleMode = 'Brake'
        kTurningMotorIdleMode = 'Brake'

        kDrivingMotorCurrentLimit = 50  # amps
        kTurningMotorCurrentLimit = 20  # amps

class AutoConstants:
        kMaxSpeedMetersPerSecond = 3
        kMaxAccelerationMetersPerSecondSquared = 3
        kMaxAngularSpeedRadiansPerSecond = math.pi
        kMaxAngularSpeedRadiansPerSecondSquared = math.pi

        kPXController = 1
        kPYController = 1
        kPThetaController = 1

        kThetaControllerConstraints = TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
        )