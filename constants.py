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
kRightElevatorMotor = 8

kElevDt = 0.02

### Roller Claw ###
kRollerClawMotor = 9

kClawCurrentLimit = 10
kIntakeClawSpeed = -1
kOuttakeClawSpeed = 1