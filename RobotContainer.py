#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
from commands2 import RunCommand, WaitCommand, SequentialCommandGroup, SwerveControllerCommand
from commands2.button import CommandXboxController
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator, TrapezoidProfileRadians
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.controller import PIDController, HolonomicDriveController, ProfiledPIDControllerRadians
# Our Libaries/functions/constants
from Constants import OIConstants, AutoConstants, DriveConstants, CoralSubsystemConstants
from subsystems.DriveSubsystem import DriveSubsystem
from subsystems.AlgaeSubsystem import AlgaeSubsystem
from subsystems.CoralSubsystem import CoralSubsystem

class RobotContainer:
    """
    Container class for the robot subystems, default commands, simple
    autonomous routines, and controller bindings
    """
    def __init__(self):
        # The robot's subsystems
        self.m_robotDrive = DriveSubsystem()
        self.m_coralSubsystem = CoralSubsystem()
        self.m_algaeSubsystem = AlgaeSubsystem()

        self.m_robotDrive.zeroHeading()

        # The controller port assignments
        self.m_driverController = CommandXboxController(OIConstants.kDriverControllerPort)
        self.m_operatorController= CommandXboxController(OIConstants.kOperatorControllerPort)

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure default comemands
        ## Drive default
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
        ## Algae Default
        
        self.m_algaeSubsystem.setDefaultCommand(
            RunCommand(
                lambda: self.m_algaeSubsystem.idle_command(),
                self.m_algaeSubsystem
            )
        )

    def applyDeadband(self, value, deadband):
        """Applys a deadband to a joystick input"""
        return value if abs(value) > deadband else 0.0

    def configureButtonBindings(self):
        """Configures the default button bindings"""
        # Locks the wheels into an X shape so that they cannot move
        self.m_driverController.rightStick().whileTrue(
            RunCommand(
                lambda: self.m_robotDrive.setX(),
                self.m_robotDrive
            )
        )

        ### Coral Subystem Commands ### 
         # Left Bumper -> Run tube intake
        self.m_operatorController.leftBumper().whileTrue(
            RunCommand(
                lambda: self.m_coralSubsystem.run_intake_command(),
                self.m_coralSubsystem
            )
        ).onFalse(
            RunCommand(
                lambda: self.m_coralSubsystem.stop_intake_command()
            )
        )

        # Right Bumper -> Run tube intake in reverse
        self.m_operatorController.rightBumper().whileTrue(
            RunCommand(
                lambda: self.m_coralSubsystem.reverse_intake_command(),
                self.m_coralSubsystem
            )
        ).onFalse(
            RunCommand(
                lambda: self.m_coralSubsystem.stop_intake_command()
            )
        )

        # B Button -> Elevator/Arm to human player position, set ball intake to stow when idle
        self.m_operatorController.b().onTrue(
            RunCommand(
                lambda: self.m_coralSubsystem.set_setpoint_command(
                CoralSubsystemConstants.ElevatorSetpoints.kFeederStation
                ),
                self.m_coralSubsystem
            )
        )

        # A Button -> Elevator/Arm to level 2 position
        self.m_operatorController.a().onTrue(
            RunCommand(
                lambda: self.m_coralSubsystem.set_setpoint_command(
                CoralSubsystemConstants.ElevatorSetpoints.kLevel1
                ),
                self.m_coralSubsystem
            )
        )

        # X Button -> Elevator/Arm to level 3 position
        self.m_operatorController.x().onTrue(
            RunCommand(
                lambda: self.m_coralSubsystem.set_setpoint_command(
                CoralSubsystemConstants.ElevatorSetpoints.kLevel2
                ),
                self.m_coralSubsystem
            )
        )

        # Y Button -> Elevator/Arm to level 4 position
        self.m_operatorController.y().onTrue(
            RunCommand(lambda: self.m_coralSubsystem.set_setpoint_command(
                CoralSubsystemConstants.ElevatorSetpoints.kLevel3
                ),
                self.m_coralSubsystem
            )
        )

        ### Algae Subsystem Controlls ###

        # Left Trigger -> Run ball intake, set to leave out when idle
        self.m_operatorController.leftTrigger(OIConstants.kTriggerButtonThreshold).whileTrue(
            RunCommand(
                lambda: self.m_algaeSubsystem.run_intake_command(),
                self.m_algaeSubsystem
            )
        )

        # Right Trigger -> Run ball intake in reverse, set to stow when idle
        self.m_operatorController.rightTrigger(OIConstants.kTriggerButtonThreshold).whileTrue(
            RunCommand(
                lambda: self.m_algaeSubsystem.reverse_intake_command(),
                self.m_algaeSubsystem
            )
        )
        
        self.m_operatorController.leftStick().onTrue(
            RunCommand(
                lambda: self.m_algaeSubsystem.stow_command()
            )
        )


    '''
    def getSimulationTotalCurrentDraw(self):
        # For each subsystem with simulation, returns total current draw
        return self.m_coralSubsystem.get_simulation_current_draw() + self.m_algaeSubsystem.get_simulation_current_draw()
    '''

class AutonomousCommand:
    def __init__(self, robot_drive: DriveSubsystem, coral_system: CoralSubsystem):
        self.robot_drive = robot_drive
        self.coral_system = coral_system
    
    def get_autonomous_command(self):
        """returns the default autonomous command to run"""
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

        # Create a PIDController for turning
        theta_controller = ProfiledPIDControllerRadians(
            AutoConstants.kPThetaController, 0, 0,
            TrapezoidProfileRadians.Constraints(
                AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                AutoConstants.kMaxAngularAccelerationRadiansPerSecond
            )
        )

        theta_controller.enableContinuousInput(-2 * math.pi, 2 * math.pi)  # Ensure smooth turning


        #Backward trajectory
        backward_trajectory = TrajectoryGenerator.generateTrajectory(
            Pose2d(1, 0, Rotation2d(0)),  # Start where the previous move ended, but rotated
            [],
            Pose2d(0, 0, Rotation2d(math.pi)),  # Move backward another 1 meter
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
            [self.robot_drive],
            Rotation2d(math.pi)
        )

         # Wait for 1 second to let the turn finish
        wait_command = WaitCommand(1)

        return SequentialCommandGroup(
            # Drive robot forward
            forward_command,
            # Stop robot in place
            RunCommand(lambda: self.robot_drive.setX(), self.robot_drive),
            # Spit out some coral at level 1
            SequentialCommandGroup(
                RunCommand(
                    lambda: self.coral_system.set_setpoint_command(
                        CoralSubsystemConstants.ElevatorSetpoints.kLevel1
                    ),
                    self.coral_system
                ),
                RunCommand(
                    lambda: self.coral_system.move_to_setpoint(),
                    self.coral_system
                ),
                RunCommand(
                    lambda: self.coral_system.reverse_intake_command(),
                    self.coral_system
                ).withTimeout(2)
            )
        )
