#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import constants
from subsystems import elevatorsubsystem

#from subsystems.pwm_launchersubsystem import LauncherSubsystem


class Elevatorin (commands2.Command):
    def __init__(self, elevator: elevatorsubsystem) -> None:
        super().__init__()
        self.elevator = elevator
        self.addRequirements(elevator)

    def initialize(self) -> None:
        self.elevator.elevatorin (constants.kElevDt)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.elevator.stop()