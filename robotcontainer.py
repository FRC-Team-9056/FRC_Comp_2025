#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
from wpilib.interfaces import GenericHID
from wpimath.filter import SlewRateLimiter

import commands2
import commands2.button

import constants

from commands.autos import Autos
from commands.launchnote import LaunchNote
from commands.preparelaunch import PrepareLaunch
from commands.roll_in import GrabNote
from commands.roll_out import PlaceNote
from commands.climb import Elevatorin
from commands.handsintheair import Elevatorout

from subsystems.can_drivesubsystem import DriveSubsystem
from subsystems.can_launchersubsystem import LauncherSubsystem
from subsystems.rollerclawsubsystem import ClawSubsystem
from subsystems.elevatorsubsystem import ElevatorSubsystem

# from subsystems.pwm_drivesubsystem import DriveSubsystem
# from subsystems.pwm_launchersubsystem import LauncherSubsystem


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The driver's controller
        self.driverController = commands2.button.CommandXboxController(
            constants.kDriverControllerPort
        )
        self.operatorController = commands2.button.CommandXboxController(
            constants.kOperatorControllerPort
        )

        # The robot's subsystems
        self.drive = DriveSubsystem()
        self.launcher = LauncherSubsystem()
        self.claw = ClawSubsystem()
        self.elevator = ElevatorSubsystem()

        self.configureButtonBindings()
 
        self.drive.setDefaultCommand(
            # A double stick-stick trank command, with forward/backward controlled on the left fore
            # and aft, and right controlled by fore and aft

            commands2.cmd.run(
                lambda: self.drive.tankDrive(
                    -self.driverController.getLeftY(),
                    -self.driverController.getRightY(),
                ),
                self.drive,
            )
        )


    def configureButtonBindings(self):
        ### Launcher ###
        # Intake #
        self.operatorController.leftBumper().whileTrue(
            PrepareLaunch(self.launcher)
            .withTimeout(constants.kLauncherDelay)
            .andThen(LaunchNote(self.launcher))
            .handleInterrupt(lambda: self.launcher.stop())
        )
        # Launch #
        self.operatorController.rightBumper().whileTrue(self.launcher.getIntakeCommand())

        ### Roller Claw ###
        # Intake #
        self.operatorController.a().whileTrue(
            GrabNote(self.claw)
            .handleInterrupt(lambda: self.claw.stop)
        )

        # Release #
        self.operatorController.b().whileTrue(
            PlaceNote(self.claw)
            .handleInterrupt(lambda: self.claw.stop)
        )

        ### Elevator ###
        # Climb #
        #self.operatorController.x().whileTrue()
        self.operatorController.x().whileTrue(
            Elevatorin(self.elevator)
            .handleInterrupt(lambda: self.elevator.stop)
        )
        # Decend #
        #self.operatorController.y().whileTrue()
        self.operatorController.y().whileTrue(
            Elevatorout(self.elevator)
            .handleInterrupt(lambda: self.elevator.stop)
        )
}?"
