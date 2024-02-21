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
            constants.kRightElevatorMortor, rev.CANSparkMax.MotorType.kBrushless
        )

        self.elevatorMotors = wpilib.MotorControllerGroup(self.left, self.right)

        # Establishes elevator profile #
        self.profile = wpimath.trajectory.TrapezoidProfile(
            wpimath.trajectory.TrapezoidProfile.Constraints(
                wpimath.units.feetToMeters(3.0),
                wpimath.units.feetToMeters(6.0),  # Max elevator speed and acceleration.
            )
        )

        self.lastProfiledReference = wpimath.trajectory.TrapezoidProfile.State()

        # The plant holds a state-space model of our elevator. This system has the following properties:

        # States: [position, velocity], in meters and meters per second.
        # Inputs (what we can "put in"): [voltage], in volts.
        # Outputs (what we can measure): [position], in meters.

        # This elevator is driven by two NEO motors.
        self.elevatorPlant = wpimath.system.plant.LinearSystemId.elevatorSystem(
            wpimath.system.plant.DCMotor.NEO(2),
            constants.kCarriageMass,
            constants.kDrumRadius,
            constants.kElevatorGearing,
        )

        # The observer fuses our encoder data and voltage inputs to reject noise.
        self.observer = wpimath.estimator.KalmanFilter_2_1_1(
            self.elevatorPlant,
            [
                wpimath.units.inchesToMeters(2),
                wpimath.units.inchesToMeters(40),
            ],  # How accurate we think our model is, in meters and meters/second.
            [
                0.001
            ],  # How accurate we think our encoder position data is. In this case we very highly trust our encoder position reading.
            0.020,
        )

        # A LQR uses feedback to create voltage commands.
        self.controller = wpimath.controller.LinearQuadraticRegulator_2_1(
            self.elevatorPlant,
            [
                wpimath.units.inchesToMeters(1.0),
                wpimath.units.inchesToMeters(10.0),
            ],  # qelms. Position
            # and velocity error tolerances, in meters and meters per second. Decrease this to more
            # heavily penalize state excursion, or make the controller behave more aggressively. In
            # this example we weight position much more highly than velocity, but this can be
            # tuned to balance the two.
            [12.0],  # relms. Control effort (voltage) tolerance. Decrease this to more
            # heavily penalize control effort, or make the controller less aggressive. 12 is a good
            # starting point because that is the (approximate) maximum voltage of a battery.
            0.020,  # Nominal time between loops. 0.020 for TimedRobot, but can be
            # lower if using notifiers.
        )

        # The state-space loop combines a controller, observer, feedforward and plant for easy control.
        self.loop = wpimath.system.LinearSystemLoop_2_1_1(
            self.elevatorPlant, self.controller, self.observer, 12.0, 0.020
        )

        # An encoder set up to measure flywheel velocity in radians per second.
        self.encoder = wpilib.Encoder(kEncoderAChannel, kEncoderBChannel)

        self.motor = wpilib.PWMSparkMax(kMotorPort)

        # A joystick to read the trigger from.
        self.joystick = wpilib.Joystick(constants.kOperatorControllerPort)

        # Circumference = pi * d, so distance per click = pi * d / counts
        self.encoder.setDistancePerPulse(math.tau * kDrumRadius / 4096)

        # The robot's elevator

