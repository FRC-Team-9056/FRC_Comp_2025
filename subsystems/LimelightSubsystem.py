from ntcore import NetworkTableInstance
from subsystems.MAXSwerveModule import MAXSwerveModule
from commands2 import Command


class LimelightSubsystem:
    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("limelight")

    def has_target(self):
        return self.table.getNumber("tv", 0.0) == 1.0

    def get_tx(self):
        return self.table.getNumber("tx", 0.0)

    def get_tid(self):
        return self.table.getNumber("tid", -1)

    def get_botpose(self):
        return self.table.getNumberArray("botpose", [])

class AutoAlignToTag(Command):
    def __init__(self, drivetrain, limelight):
        super().__init__()
        self.drivetrain = MAXSwerveModule
        self.limelight = LimelightSubsystem
        self.kP = 0.03  # Tune this value!
        self.min_cmd = 0.05  # Minimum speed to overcome static friction

        self.setName("AutoAlignToTag")
        self.addRequirements(drivetrain)

    def initialize(self):
        print("Auto-align starting...")

    def execute(self):
        if self.limelight.has_target():
            tx = self.limelight.get_tx()
            turn_cmd = self.kP * tx

            # Add minimum command to make sure it turns
            if abs(turn_cmd) < self.min_cmd:
                turn_cmd = self.min_cmd * (1 if tx > 0 else -1)

            self.drivetrain.set_desired_state(0, -turn_cmd)
        else:
            self.drivetrain.set_desired_state(0, 0)  # Stop if no target

    def isFinished(self):
        if not self.limelight.has_target():
            return True
        return abs(self.limelight.get_tx()) < 1.0  # Degrees tolerance

    def end(self, interrupted):
        self.drivetrain.set_desired_state(0, 0)
        print("Auto-align finished.")
