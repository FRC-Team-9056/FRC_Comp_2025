#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import wpimath
from wpilib.interfaces import GenericHID
from wpimath.filter import SlewRateLimiter

import commands2
import commands2.button

import constants

# import autonomous commands here
#from commands.autos import Autos

from FRC_Comp_2025.subsystems.drivesubsystem import DriveSubsystem


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        """Robot initialization function"""
        self.driverController = commands2.button.CommandXboxController(
            constants.kDriverControllerPort
        )
        self.operatorController = commands2.button.CommandXboxController(
            constants.kOperatorControllerPort
        )

        self.swerve = DriveSubsystem()

        self.swerve.setDefaultCommand(
            # A double stick-stick swerve drive command. Set boolean below to make it field relative

            commands2.cmd.run(
                lambda: self.swerve.driveWithJoystick(
                    self.driverController,
                    True 
                ),
                self.swerve,
            )
        )


    def configureButtonBindings(self):
        ### Launcher Example from 2024###
        # Intake #
        #self.operatorController.leftBumper().whileTrue(
        #    PrepareLaunch(self.launcher)
        #    .withTimeout(constants.kLauncherDelay)
        #    .andThen(LaunchNote(self.launcher))
        #    .handleInterrupt(lambda: self.launcher.stop())
        #)
        ### Replace Pass with button bindings
        pass

    def getAutonomousCommand(self) -> commands2.Command:
        #return Autos.exampleAuto()
        # this is an example from last year of importing those commands replace pass
        # with the default command
        pass