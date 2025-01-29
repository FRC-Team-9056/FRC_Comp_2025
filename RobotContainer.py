#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
from wpimath.controller import PIDController, ProfiledPIDController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import Trajectory, TrajectoryConfig, TrajectoryGenerator
from wpilib import XboxController
from commands2 import Command, RunCommand, SwerveControllerCommand
from commands2.button import JoystickButton
from Constants import AutoConstants, DriveConstants, OIConstants
from subsystems.DriveSubsystem import DriveSubsystem

class RobotContainer:
    def __init__(self):
        # The robot's subsystems
        self.m_robotDrive = DriveSubsystem()

        # The driver's controller
        self.m_driverController = XboxController(OIConstants.kDriverControllerPort)

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure default comemands
        self.m_robotDrive.setDefaultCommand(
            RunCommand(
                lambda: self.m_robotDrive.drive(
                    -self.applyDeadband(self.m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -self.applyDeadband(self.m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -self.applyDeadband(self.m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    False),
                self.m_robotDrive
            )
        )

    def applyDeadband(self, value, deadband):
        """Apply a deadband to a joystick input"""
        return value if abs(value) > deadband else 0.0

    def configureButtonBindings(self):
        JoystickButton(self.m_driverController, XboxController.Button.kRightStick).whileTrue(
            RunCommand(lambda: self.m_robotDrive.setX(), self.m_robotDrive)
        )

    def getAutonomousCommand(self):
        # Create config for trajectory
        config = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared
        ).setKinematics(DriveConstants.kDriveKinematics)

        """
        # An example trajectory to follow. All units in meters.
        exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            Pose2d(0, 0, Rotation2d(0)),
            [Translation2d(1, 1), Translation2d(2, -1)],
            Pose2d(3, 0, Rotation2d(0)),
            config
        )
        

        thetaController = ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints
        )
        thetaController.enableContinuousInput(-math.pi, math.pi)

        swerveControllerCommand = SwerveControllerCommand(
            exampleTrajectory,
            self.m_robotDrive.getPose,
            DriveConstants.kDriveKinematics,

            # Position controllers
            PIDController(AutoConstants.kPXController, 0, 0),
            PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            self.m_robotDrive.setModuleStates,
            self.m_robotDrive
        )

        # Reset odometry to the starting pose of the trajectory.
        self.m_robotDrive.resetOdometry(Trajectory.initialPose)

        # Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(lambda: self.m_robotDrive.drive(0, 0, 0, False))
        """