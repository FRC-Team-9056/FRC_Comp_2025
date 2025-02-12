#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import Constants
from wpilib import XboxController
from commands2 import RunCommand
from commands2.button import JoystickButton
from Constants import OIConstants
from subsystems.DriveSubsystem import DriveSubsystem
from subsystems.AlgaeSubsystem import AlgaeSubsystem
from subsystems.CoralSubsystem import CoralSubsystem
print("starting robotcontainer")

class RobotContainer:
    def __init__(self):
        # The robot's subsystems
        self.m_robotDrive = DriveSubsystem()
        self.m_coralSubsystem = CoralSubsystem()
        self.m_algaeSubsystem = AlgaeSubsystem()

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
        JoystickButton(lambda: self.m_driverController.getRightTriggerAxis() > OIConstants.kTriggerButtonThreshold,
                       XboxController.Axis.kRightTrigger).whileTrue(
            RunCommand(lambda: self.m_algaeSubsystem.run_intake_command(), self.m_algaeSubsystem)
        )

        # Left Trigger -> Run ball intake in reverse, set to stow when idle
        JoystickButton(lambda: self.m_driverController.getLeftTriggerAxis() > OIConstants.kTriggerButtonThreshold,
                       XboxController.Axis.kRightTrigger).whileTrue(
            RunCommand(lambda: self.m_algaeSubsystem.run_intake_command(), self.m_algaeSubsystem)
        )
    
    print("finished button bindings")

    def getSimulationTotalCurrentDraw(self):
        # For each subsystem with simulation, returns total current draw
        return self.m_coralSubsystem.get_simulation_current_draw() + self.m_algaeSubsystem.get_simulation_current_draw()


    """
    def getAutonomousCommand(self):
        # Create config for trajectory
        config = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared
        ).setKinematics(DriveConstants.kDriveKinematics)

    
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