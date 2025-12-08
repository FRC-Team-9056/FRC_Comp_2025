import wpilib
from wpimath.controller import PIDController
from commands2 import Subsystem
from networktables import NetworkTables

class LimelightSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self.table = NetworkTables.getTable("limelight")

    def get_tx(self):
        """Horizontal offset from crosshair to target (-27 to 27 degrees)"""
        return self.table.getNumber("tx", 0.0)

    def get_ty(self):
        """Vertical offset from crosshair to target (-20.5 to 20.5 degrees)"""
        return self.table.getNumber("ty", 0.0)

    def get_tv(self):
        """Whether the limelight has any valid targets (0 or 1)"""
        return self.table.getNumber("tv", 0.0)

    def has_target(self):
        return self.get_tv() == 1.0