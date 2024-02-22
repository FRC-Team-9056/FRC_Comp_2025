#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import constants

from subsystems.elevatorsubsystem import ElevatorSubsystem
#from subsystems.pwm_launchersubsystem import LauncherSubsystem


class Elevatorout(commands2.Command):
    def __init__(self, elevator: ElevatorSubsystem) -> None:
        super().__init__()
        self.elevator = elevator
        self.addRequirements(elevator)

    def initialize(self) -> None:
        self.elevator.elevatorout(constants.kElevDt)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.elevator.stop()