#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import rev

import constants


class ClawSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()
        """The two motors of the launcher subsytem"""
        self.claw = rev.CANSparkMax(
            constants.kRollerClawMotor, rev.CANSparkMax.MotorType.kBrushed
        )

    def getIntakeCommand(self) -> commands2.Command:
        """The startEnd helper method takes a method to call when the command is initialized and one to
        call when it ends"""
        return commands2.cmd.startEnd(
            # When the command is initialized, set the wheels to the intake speed values
            lambda: self.setClawWheel(
                constants.kIntakeClawSpeed
            ),
            # When the command stops, stop the wheels
            lambda: self.stop(),
            self,
        )
 
    def setClawWheel(self, speed: float) -> None:
        """An accessor method to set the speed (technically the output percentage) of the launch wheel"""
        self.ClawWheel.set(speed)

    def stop(self) -> None:
        """A helper method to stop both wheels. You could skip having a method like this and call the
        individual accessors with speed = 0 instead"""
        self.ClawWheel.set(0)