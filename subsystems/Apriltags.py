from commands2 import Command
from wpimath.controller import PIDController
from subsystems.LimelightSubsystem import LimelightSubsystem
from subsystems.DriveSubsystem import DriveSubsystem
import wpimath

class AutoAlignToTag(Command):
    def __init__(self, DriveSubsystem, LimelightSubsystem):
        super().__init__()
        self.drive = DriveSubsystem
        self.limelight = LimelightSubsystem

        self.pid = PIDController(0.02, 0.0, 0.002)
        self.pid.setTolerance(1.0)  # Degrees

        self.addRequirements(self.drive)

    def initialize(self):
        self.pid.reset()

    def execute(self):
        if not self.limelight.has_target():
            self.drive.drive(0, 0, 0, fieldRelative=True)
            return

        tx = self.limelight.get_tx()
        output = self.pid.calculate(tx, 0)

        # Clamp rotation speed
        output = max(min(output, 0.5), -0.5)

        # Only rotate in place
        self.drive.drive(0, 0, output, fieldRelative=True)

    def isFinished(self):
        return self.pid.atSetpoint() and self.limelight.has_target()

    def end(self, interrupted):
        self.drive.drive(0, 0, 0, fieldRelative=True)