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
kOperatorControllerPort = 1

### Drivetrain ###
kLeftMotor1Port = 1
kLeftMotor2Port = 2
kRightMotor1Port = 3
kRightMotor2Port = 4

kMaxSpeed = 3.0  # 3 meters per second
kMaxAngularSpeed = math.pi  # 1/2 rotation per second
kWheelRadius = 0.0508
kEncoderResolution = 4096
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau

# Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
kSlewRateLimiter = 3