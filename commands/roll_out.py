#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import constants

from subsystems.rollerclawsubsystem import ClawSubsystem
#from subsystems.pwm_launchersubsystem import LauncherSubsystem


class PlaceNote(commands2.Command):
    def __init__(self, claw: ClawSubsystem) -> None:
        super().__init__()
        self.claw = claw
        self.addRequirements(claw)

    def initialize(self) -> None:
        self.claw.setClawWheel(constants.kOuttakeClawSpeed)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.claw.stop()