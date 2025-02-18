#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import Constants
import math
from wpilib import XboxController
from commands2 import SwerveControllerCommand
from commands2 import RunCommand, Command
from commands2.button import JoystickButton
from Constants import OIConstants, AutoConstants, DriveConstants
from subsystems.DriveSubsystem import DriveSubsystem
from subsystems.AlgaeSubsystem import AlgaeSubsystem
from subsystems.CoralSubsystem import CoralSubsystem
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator, TrapezoidProfile, TrapezoidProfileRadians
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.controller import PIDController, ProfiledPIDController, HolonomicDriveController, ProfiledPIDControllerRadians
from commands2 import SequentialCommandGroup, InstantCommand


class RobotContainer:
    def __init__(self):
        # The robot's subsystems
        self.m_robotDrive = DriveSubsystem()
        self.m_coralSubsystem = CoralSubsystem()
        self.m_algaeSubsystem = AlgaeSubsystem()

        self.m_robotDrive.zeroHeading()


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
                    True),
                self.m_robotDrive
            )
        )

    print("finished init")

    def applyDeadband(self, value, deadband):
        """Apply a deadband to a joystick input"""
        return value if abs(value) > deadband else 0.0

    def configureButtonBindings(self):
        JoystickButton(self.m_driverController, XboxController.Button.kRightStick).whileTrue(
            RunCommand(lambda: self.m_robotDrive.setX(), self.m_robotDrive)
        )

         # Left Bumper -> Run tube intake
        JoystickButton(self.m_driverController, XboxController.Button.kLeftBumper).whileTrue(
            RunCommand(lambda: self.m_coralSubsystem.run_intake_command(), self.m_coralSubsystem)
        )

        # Right Bumper -> Run tube intake in reverse
        JoystickButton(self.m_driverController, XboxController.Button.kRightBumper).whileTrue(
            RunCommand(lambda: self.m_coralSubsystem.reverse_intake_command(), self.m_coralSubsystem)
        )

        # B Button -> Elevator/Arm to human player position, set ball intake to stow when idle
        JoystickButton(self.m_driverController, XboxController.Button.kB).onTrue(
            RunCommand(lambda: self.m_coralSubsystem.set_setpoint_command(Constants.CoralSubsystemConstants.ElevatorSetpoints.kFeederStation), self.m_coralSubsystem)
        )

        # A Button -> Elevator/Arm to level 2 position
        JoystickButton(self.m_driverController, XboxController.Button.kA).onTrue(
            RunCommand(lambda: self.m_coralSubsystem.set_setpoint_command(Constants.CoralSubsystemConstants.ElevatorSetpoints.kLevel2), self.m_coralSubsystem)
        ) 

        # X Button -> Elevator/Arm to level 3 position
        JoystickButton(self.m_driverController, XboxController.Button.kX).onTrue(
            RunCommand(lambda: self.m_coralSubsystem.set_setpoint_command(Constants.CoralSubsystemConstants.ElevatorSetpoints.kLevel3), self.m_coralSubsystem)
        )

        # Y Button -> Elevator/Arm to level 4 position
        JoystickButton(self.m_driverController, XboxController.Button.kY).onTrue(
            RunCommand(lambda: self.m_coralSubsystem.set_setpoint_command(Constants.CoralSubsystemConstants.ElevatorSetpoints.kLevel4), self.m_coralSubsystem)
        )

        # Right Trigger -> Run ball intake, set to leave out when idle
        while self.m_driverController.getRightTriggerAxis() > OIConstants.kTriggerButtonThreshold:
            RunCommand(lambda: self.m_algaeSubsystem.run_intake_command(), self.m_algaeSubsystem)

        # Left Trigger -> Run ball intake in reverse, set to stow when idle
        while self.m_driverController.getLeftTriggerAxis() > OIConstants.kTriggerButtonThreshold:
            RunCommand(lambda: self.m_algaeSubsystem.run_intake_command(), self.m_algaeSubsystem)
    
    print("finished button bindings")

    def getSimulationTotalCurrentDraw(self):
        # For each subsystem with simulation, returns total current draw
        return self.m_coralSubsystem.get_simulation_current_draw() + self.m_algaeSubsystem.get_simulation_current_draw()


class AutonomousCommand:
    def __init__(self, robot_drive: DriveSubsystem):
        self.robot_drive = robot_drive
    
    def get_autonomous_command(self):
        config = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared
        )
        config.setKinematics(DriveConstants.kDriveKinematics)
        
        #Forward trajectory
        forward_trajectory = TrajectoryGenerator.generateTrajectory(
            Pose2d(0, 0, Rotation2d(0)),
            [],
            Pose2d(1, 0, Rotation2d(0)),
            config
        )

        self.robot_drive.resetOdometry(forward_trajectory.initialPose())

        #Backward trajectory
        backward_trajectory = TrajectoryGenerator.generateTrajectory(
            Pose2d(1, 0, Rotation2d(0)),
            [],
            Pose2d(0, 0, Rotation2d(0)),
            config
        )

        theta_controller = ProfiledPIDControllerRadians(
            AutoConstants.kPThetaController, 0, 0,
            TrapezoidProfileRadians.Constraints(
                AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                AutoConstants.kMaxAngularAccelerationRadiansPerSecond
            )
        )

        holonomic_controller = HolonomicDriveController(
            PIDController(AutoConstants.kPXController, 0, 0),  # X control
            PIDController(AutoConstants.kPYController, 0, 0),  # Y control
            theta_controller  # Theta control (ProfiledPIDController)
        )


        forward_command = SwerveControllerCommand(
            forward_trajectory,
            self.robot_drive.getPose,
            DriveConstants.kDriveKinematics,
            holonomic_controller,
            self.robot_drive.setModuleStates,
            [self.robot_drive]
        )

        backward_command = SwerveControllerCommand(
            backward_trajectory,
            self.robot_drive.getPose,
            DriveConstants.kDriveKinematics,
            holonomic_controller,
            self.robot_drive.setModuleStates,
            [self.robot_drive]
        )

        return SequentialCommandGroup(
            forward_command,
            InstantCommand(lambda: print("Finished Forward")),
            InstantCommand(lambda: self.robot_drive.drive(0, 0, 0, False), self.robot_drive),

            InstantCommand(lambda: self.robot_drive.resetOdometry(backward_trajectory.initialPose()), self.robot_drive),

            backward_command,
            InstantCommand(lambda: print("Finished Backward")),
            InstantCommand(lambda: self.robot_drive.drive(0, 0, 0, False), self.robot_drive)
        )