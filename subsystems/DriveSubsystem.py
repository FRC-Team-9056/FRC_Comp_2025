#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import math
import wpilib
from wpilib import ADIS16470_IMU, ADXRS450_Gyro
from wpimath.geometry import Rotation2d
from wpimath.filter import SlewRateLimiter
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveDrive4Odometry, SwerveModuleState, SwerveModulePosition
from commands2.subsystem import Subsystem as SubsystemBase
from Constants import DriveConstants
import utils.SwerveUtils as SwerveUtils
from subsystems.MAXSwerveModule import MAXSwerveModule

class DriveSubsystem(SubsystemBase):
    def __init__(self):
        # Initialize swerve modules
        self.m_frontLeft = MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset)

        self.m_frontRight = MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset)

        self.m_rearLeft = MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset)

        self.m_rearRight = MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset)

        # Gyro sensor (IMU)
        self.m_gyro = ADXRS450_Gyro()

        # Slew rate limiter initialization
        self.m_currentRotation = 0.0
        self.m_currentTranslationDir = 0.0
        self.m_currentTranslationMag = 0.0
        self.m_magLimiter = SlewRateLimiter(DriveConstants.kMagnitudeSlewRate)
        self.m_rotLimiter = SlewRateLimiter(DriveConstants.kRotationalSlewRate)
        self.m_prevTime = wpilib.Timer.getFPGATimestamp()

        # Odometry for tracking robot pose
        self.m_odometry = SwerveDrive4Odometry(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(self.m_gyro.getAngle(ADIS16470_IMU.IMUAxis.kZ)),
            [
                self.m_frontLeft.getPosition(),
                self.m_frontRight.getPosition(),
                self.m_rearLeft.getPosition(),
                self.m_rearRight.getPosition()
            ])

    def periodic(self):
        # Update odometry
        self.m_odometry.update(
            Rotation2d.fromDegrees(self.m_gyro.getAngle(ADIS16470_IMU.IMUAxis.kZ)),
            [
                self.m_frontLeft.getPosition(),
                self.m_frontRight.getPosition(),
                self.m_rearLeft.getPosition(),
                self.m_rearRight.getPosition()
            ])

    def getPose(self):
        """Returns the current estimated pose of the robot."""
        return self.m_odometry.getPoseMeters()

    def resetOdometry(self, pose):
        """Resets the odometry to the specified pose."""
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

    def drive(self, xSpeed, ySpeed, rot, fieldRelative, rateLimit):
        """Drives the robot based on joystick inputs."""
        xSpeedCommanded = 0.0
        ySpeedCommanded = 0.0

        if rateLimit:
            # Convert XY speeds to polar for rate limiting
            inputTranslationDir = math.atan2(ySpeed, xSpeed)
            inputTranslationMag = math.sqrt(xSpeed**2 + ySpeed**2)

            directionSlewRate = (abs(DriveConstants.kDirectionSlewRate / self.m_currentTranslationMag)
                                 if self.m_currentTranslationMag != 0.0 else 500.0)

            currentTime = wpilib.Timer.getFPGATimestamp()
            elapsedTime = currentTime - self.m_prevTime
            angleDif = SwerveUtils.AngleDifference(inputTranslationDir, self.m_currentTranslationDir)
            if angleDif < 0.45 * math.pi:
                self.m_currentTranslationDir = SwerveUtils.StepTowardsCircular(
                    self.m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime)
                self.m_currentTranslationMag = self.m_magLimiter.calculate(inputTranslationMag)
            elif angleDif > 0.85 * math.pi:
                if self.m_currentTranslationMag > 1e-4:
                    self.m_currentTranslationMag = self.m_magLimiter.calculate(0.0)
                else:
                    self.m_currentTranslationDir = SwerveUtils.WrapAngle(self.m_currentTranslationDir + math.pi)
                    self.m_currentTranslationMag = self.m_magLimiter.calculate(inputTranslationMag)
            else:
                self.m_currentTranslationDir = SwerveUtils.StepTowardsCircular(
                    self.m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime)
                self.m_currentTranslationMag = self.m_magLimiter.calculate(0.0)
            self.m_prevTime = currentTime

            xSpeedCommanded = self.m_currentTranslationMag * math.cos(self.m_currentTranslationDir)
            ySpeedCommanded = self.m_currentTranslationMag * math.sin(self.m_currentTranslationDir)
            self.m_currentRotation = self.m_rotLimiter.calculate(rot)

        else:
            xSpeedCommanded = xSpeed
            ySpeedCommanded = ySpeed
            self.m_currentRotation = rot

        # Convert the commanded speeds to the correct units
        xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond
        ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond
        rotDelivered = self.m_currentRotation * DriveConstants.kMaxAngularSpeed

        # Generate the swerve module states
        swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(self.m_gyro.getAngle(ADIS16470_IMU.IMUAxis.kZ)))
            if fieldRelative else ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered))

        # Desaturate wheel speeds
        SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond)

        # Set the desired state for each swerve module
        self.m_frontLeft.setDesiredState(swerveModuleStates[0])
        self.m_frontRight.setDesiredState(swerveModuleStates[1])
        self.m_rearLeft.setDesiredState(swerveModuleStates[2])
        self.m_rearRight.setDesiredState(swerveModuleStates[3])

    def setX(self):
        """Sets the wheels into an X formation to prevent movement."""
        self.m_frontLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.m_frontRight.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.m_rearLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.m_rearRight.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))

    def setModuleStates(self, desiredStates):
        """Sets the swerve module states."""
        SwerveDrive4Kinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond)
        self.m_frontLeft.setDesiredState(desiredStates[0])
        self.m_frontRight.setDesiredState(desiredStates[1])
        self.m_rearLeft.setDesiredState(desiredStates[2])
        self.m_rearRight.setDesiredState(desiredStates[3])

    def resetEncoders(self):
        """Resets the drive encoders."""
        self.m_frontLeft.resetEncoders()
        self.m_rearLeft.resetEncoders()
        self.m_frontRight.resetEncoders()
        self.m_rearRight.resetEncoders()

    def zeroHeading(self):
        """Zeroes the heading of the robot."""
        self.m_gyro.reset()

    def getHeading(self):
        """Returns the heading of the robot in degrees."""
        return Rotation2d.fromDegrees(self.m_gyro.getAngle(ADIS16470_IMU.IMUAxis.kZ)).getDegrees()

    def getTurnRate(self):
        """Returns the turn rate of the robot."""
        return self.m_gyro.getRate(ADIS16470_IMU.IMUAxis)