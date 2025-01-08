#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
from wpilib import XboxController
import commands2 as command
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import Trajectory, TrajectoryConfig, TrajectoryGenerator
from wpimath.controller import PIDController, ProfiledPIDController
import Constants
from subsystems.DriveSubsystem import DriveSubsystem
from utils.GamepadUtils import GamepadUtils

class RobotContainer:
    def __init__(self):
        # The robot's subsystems
        self.m_robotDrive = DriveSubsystem()

        # The driver's controller
        self.m_driverController = XboxController(Constants.OIConstants.kDriverControllerPort)

        # Configure the button bindings
        # self.configureButtonBindings()

        # Configure default commands

        self.m_robotDrive.setDefaultCommand(
            command.RunCommand(
                lambda: self.m_robotDrive.drive(
                    -GamepadUtils.squareInput(self.m_driverController.getLeftY(), Constants.OIConstants.kDriveDeadband),
                    -GamepadUtils.squareInput(self.m_driverController.getLeftX(), Constants.OIConstants.kDriveDeadband),
                    -GamepadUtils.squareInput(self.m_driverController.getRightX(), Constants.OIConstants.kDriveDeadband),
                    True, False),
                self.m_robotDrive)
        )

    def getAutonomousCommand(self):
        # Create config for trajectory
        config = TrajectoryConfig(
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
        ).setKinematics(Constants.DriveConstants.kDriveKinematics)
        """
        # Example trajectory
        exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            Pose2d(0, 0, Rotation2d(0)),
            [Translation2d(1, 1), Translation2d(2, -1)],
            Pose2d(3, 0, Rotation2d(0)),
            config
        )

        # Create theta controller
        thetaController = ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints
        )
        thetaController.enableContinuousInput(-math.pi, math.pi)

        # Swerve controller command
        swerveControllerCommand = command.SwerveControllerCommand(
            exampleTrajectory,
            self.m_robotDrive.getPose,
            Constants.DriveConstants.kDriveKinematics,
            PIDController(Constants.AutoConstants.kPXController, 0, 0),
            PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            self.m_robotDrive.setModuleStates,
            self.m_robotDrive
        )

        # Reset odometry to the starting pose of the trajectory
        self.m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose())

        # Run path following command, then stop at the end
        return swerveControllerCommand.andThen(lambda: self.m_robotDrive.drive(0, 0, 0, False, False))
        """