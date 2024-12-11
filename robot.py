#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import typing
import wpilib
import commands2

from robotcontainer import RobotContainer
#from wpilib import SmartDashboard


class MyRobot(commands2.TimedCommandRobot):
    """
    Our default robot class, pass it to wpilib.run

    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    #autonomousCommand: typing.Optional[commands2.Command] = None
    #self.chooser.setDefaultOption("Default Auto", self.defaultAuto)
    #self.chooser.addOption("My Auto", self.customAuto)
    #SmartDashboard.putData("Auto choices", self.chooser)

    #self.defaultAuto = "Default"
    #self.customAuto = "My Auto";
    #self.chooser = wpilib.SendableChooser()

    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """

        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = RobotContainer()
        #self.autonomousCommand = self.container.getAutonomousCommand()
        self.timer = wpilib.Timer()

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        #if self.autonomousCommand:
        #    self.autonomousCommand.schedule()
        self.timer.restart()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""

    def teleopInit(self) -> None:
        """This functtion is called to initiate teleop"""
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        #if self.autonomousCommand:
        #    self.autonomousCommand.cancel()

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""

    def testInit(self) -> None:
        """ This function is the test mode"""
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(MyRobot)
