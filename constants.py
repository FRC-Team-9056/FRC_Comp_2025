#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

#
# The constants module is a convenience place for teams to hold robot-wide
# numerical or boolean constants. Don't use this for any other purpose!
#

import math
import wpilib
import wpimath.controller
import wpimath.estimator
import wpimath.units
import wpimath.trajectory
import wpimath.system
import wpimath.system.plant

### Operator Interface ###
kDriverControllerPort = 0
kOperatorControllerPort = 0

### Drivetrain ###
kLeftMotor1Port = 1
kLeftMotor2Port = 2
kRightMotor1Port = 3
kRightMotor2Port = 4

kDTCurrentLimit = 60

### Launcher ###
kFeederMotor = 5
kLauncherMotor = 6
kLauncherCurrentLimit = 80
kFeedCurrentLimit = 80

kLauncherSpeed = 1
kLaunchFeederSpeed = 1
kIntakeLauncherSpeed = -1
kIntakeFeederSpeed = -0.2
kLauncherDelay = 1

### Elevator ###
kLeftElevatorMotor = 7
kRightElevatorMortor = 8

kEncoderAChannel = 0
kEncoderBChannel = 1
kHighGoalPosition = wpimath.units.feetToMeters(3)
kLowGoalPosition = wpimath.units.feetToMeters(0)


kCarriageMass = 4.5
# kilograms

# A 1.5in diameter drum has a radius of 0.75in, or 0.019in.
kDrumRadius = 1.5 / 2.0 * 25.4 / 1000.0

# Reduction between motors and encoder, as output over input. If the elevator spins slower than
# the motors, this number should be greater than one.
kElevatorGearing = 6.0

### Roller Claw ###
kRollerClawMotor = 9

kIntakeClawSpeed = -1
kOuttakeClawSpeed = 1