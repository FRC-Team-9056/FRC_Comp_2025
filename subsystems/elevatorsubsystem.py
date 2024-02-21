import math
import wpilib
import wpimath.controller
import wpimath.estimator
import wpimath.units
import wpimath.trajectory
import wpimath.system
import wpimath.system.plant
import commands2
import wpilib
import wpilib.drive
import rev

import constants


class ElevatorSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.left = rev.CANSparkMax(
            constants.kLeftElevatorMotor, rev.CANSparkMax.MotorType.kBrushless
        )
        self.right = rev.CANSparkMax(
            constants.kRightElevatorMotor, rev.CANSparkMax.MotorType.kBrushless
        )

        self.elevatorMotors = wpilib.MotorControllerGroup(self.left, self.right)

        self.kDt = constants.kElevDt

        self.encoder = self.left.getEncoder()

        # Create a PID controller whose setpoint's change is subject to maximum
        # velocity and acceleration constraints.
        self.Elevconstraints = wpimath.trajectory.TrapezoidProfile.Constraints(1.75, 0.75)
        self.Elevcontroller = wpimath.controller.ProfiledPIDController(
            1.3, 0, 0.7, self.Elevconstraints, self.kDt
        )

        self.encoder.setDistancePerPulse(1 / 360 * 2 * math.pi * 1.5)

    def teleopPeriodic(self) -> None:
        if self.joystick.getRawButtonPressed(2):
            self.controller.setGoal(5)
        elif self.joystick.getRawButtonPressed(3):
            self.controller.setGoal(0)

        # Move to Command
        # Run controller and update motor output
        self.elevatorMotors.set(self.controller.calculate(self.encoder.getDistance()))