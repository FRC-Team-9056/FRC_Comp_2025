# FRC_Comp_2024
Code for the Kitbot in 2024

# To use this repository please follow the directions below:
https://www.taniarascia.com/getting-started-with-git/

# Structure
This robot is a command based robot. What that means is that the code is set up where each subystem is a small peice of the puzzle that is called in "robotcontainer". These subsystems have commands associated with them, these are found in the commands folder.

## Robot.py
This is the part of the code that the robo rio actually runs. This is where the stages of the game are called as the game moves from mode to mode.

## Constants
This is where most changable things in the code base are found. If you swap a motor to a different controller that will be found here. If you want to speed up a motor, limit amperage, change a delay time, etc. Those variables *should* be declared here and here alone. This makes modifications easier and you don't have to go chasing variables down in other portions of the code.

This does not mean that **all** changes happen here. There are still functions and arguments that need to be changed. But, if you are setting up a number or variable that you will call again and again or need to tune, declare it here.

## Robot Container
This is where the robot is initialized, each subsystem is called and set-up, and commands are established. 
The robot container is called in robot.py to run as directed

### Subsystems
#### Claw
This is the roller claw subsytem

#### Drive
This is the drivebase subsystem

####Launcher subsystem

#### Elevator subsystem

This describes the elevator subsystem.
