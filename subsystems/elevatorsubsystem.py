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

        self.encoder.setPosition(1 / 360 * 2 * math.pi * 1.5)

        # Move to Command
        # Run controller and update motor output
        # self.elevatorMotors.set(self.Elevcontroller.calculate(self.encoder.getDistance()))

    def Moveelevatorcommand(self) -> commands2.Command:
        #this is the method for the elevator#
        return commands2.cmd.startEnd(
        #when this command is initialized, extend the elevator#
        lambda: self.elevatorin(
            self.Elevcontroller.calculate(self.encoder.getDistance())
        ),
        #when this command is initialized, bring the elevator back#
        lambda: self.stop(),
        self,
    ) 

    def elevatorin(self, speed: float) -> None:
        #an method to extend the elevator#
        self.elevatorMotors.set(speed)

    def elevatorout(self, speed: float) -> None:
        #undo(if I know I know)#
        self.elevatorMotors.set(-speed)

    def stop(self) -> None:
        self.elevatorMotors.set(0)



