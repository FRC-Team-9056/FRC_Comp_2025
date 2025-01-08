#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import math
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive2Kinematics, SwerveDrive2Odometry, SwerveModuleState, SwerveModulePosition
from wpilib import ADIS16470_IMU
from Constants import DriveConstants
from commands2 import Subsystem
from subsystems import MAXSwerveModule

class DriveSubsystem(Subsystem):
    def __init__(self):
        # Create MAXSwerveModules
        self.m_frontLeft = MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset
        )

        self.m_frontRight = MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset
        )

        self.m_rearLeft = MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset
        )

        self.m_rearRight = MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset
        )

        # The gyro sensor
        self.m_gyro = ADIS16470_IMU()

        # Odometry class for tracking robot pose
        self.m_odometry = SwerveDrive2Odometry(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(self.m_gyro.getAngle(ADIS16470_IMU.IMUAxis.kZ)),
            [
                self.m_frontLeft.getPosition(),
                self.m_frontRight.getPosition(),
                self.m_rearLeft.getPosition(),
                self.m_rearRight.getPosition()
            ]
        )

    def periodic(self):
        # Update the odometry in the periodic block
        self.m_odometry.update(
            Rotation2d.fromDegrees(self.m_gyro.getAngle(ADIS16470_IMU.IMUAxis.kZ)),
            [
                self.m_frontLeft.getPosition(),
                self.m_frontRight.getPosition(),
                self.m_rearLeft.getPosition(),
                self.m_rearRight.getPosition()
            ]
        )

    def getPose(self):
        return self.m_odometry.getPoseMeters()

    def resetOdometry(self, pose: Pose2d):
        self.m_odometry.resetPosition(
            Rotation2d.fromDegrees(self.m_gyro.getAngle(ADIS16470_IMU.IMUAxis.kZ)),
            [
                self.m_frontLeft.getPosition(),
                self.m_frontRight.getPosition(),
                self.m_rearLeft.getPosition(),
                self.m_rearRight.getPosition()
            ],
            pose
        )

    def drive(self, xSpeed, ySpeed, rot, fieldRelative):
        # Convert the commanded speeds into the correct units for the drivetrain
        xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond
        ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond
        rotDelivered = rot * DriveConstants.kMaxAngularSpeed

        if fieldRelative:
            swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(self.m_gyro.getAngle(ADIS16470_IMU.IMUAxis.kZ))
                )
            )
        else:
            swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
            )
        
        # Desaturate wheel speeds
        SwerveDrive2Kinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond)

        # Set the desired state for each swerve module
        self.m_frontLeft.setDesiredState(swerveModuleStates[0])
        self.m_frontRight.setDesiredState(swerveModuleStates[1])
        self.m_rearLeft.setDesiredState(swerveModuleStates[2])
        self.m_rearRight.setDesiredState(swerveModuleStates[3])

    def setX(self):
        self.m_frontLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.m_frontRight.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.m_rearLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.m_rearRight.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))

    def setModuleStates(self, desiredStates):
        # Desaturate wheel speeds
        SwerveDrive2Kinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond)

        # Set the desired state for each swerve module
        self.m_frontLeft.setDesiredState(desiredStates[0])
        self.m_frontRight.setDesiredState(desiredStates[1])
        self.m_rearLeft.setDesiredState(desiredStates[2])
        self.m_rearRight.setDesiredState(desiredStates[3])

    def resetEncoders(self):
        self.m_frontLeft.resetEncoders()
        self.m_rearLeft.resetEncoders()
        self.m_frontRight.resetEncoders()
        self.m_rearRight.resetEncoders()

    def zeroHeading(self):
        self.m_gyro.reset()

    def getHeading(self):
        return Rotation2d.fromDegrees(self.m_gyro.getAngle(ADIS16470_IMU.IMUAxis.kZ)).getDegrees()

    def getTurnRate(self):
        return self.m_gyro.getRate(ADIS16470_IMU.IMUAxis.kZ) * (DriveConstants.kGyroReversed if -1.0 else 1.0)
