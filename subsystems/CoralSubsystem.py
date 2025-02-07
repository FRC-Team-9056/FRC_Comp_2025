#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
from rev import SparkMax
from wpilib.simulation import ElevatorSim, SingleJointedArmSim
from rev import SparkMaxSim
from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from wpimath.system.plant import DCMotor
from wpilib import RobotController
from commands2 import Subsystem

# Constants
from Constants import CoralSubsystemConstants, SimulationRobotConstants

class CoralSubsystem(Subsystem):
    
    def __init__(self):
        super().__init__()

        # Motor controllers
        self.arm_motor = SparkMax(CoralSubsystemConstants.kArmMotorCanId)
        self.arm_encoder = self.arm_motor.getEncoder()
        
        self.elevator_motor = SparkMax(CoralSubsystemConstants.kElevatorMotorCanId)
        self.elevator_encoder = self.elevator_motor.getEncoder()
        
        self.intake_motor = SparkMax(CoralSubsystemConstants.kIntakeMotorCanId)
        
        # State variables
        self.was_reset_by_button = False
        self.was_reset_by_limit = False
        self.arm_current_target = CoralSubsystemConstants.ArmSetpoints.kFeederStation
        self.elevator_current_target = CoralSubsystemConstants.ElevatorSetpoints.kFeederStation

        # Simulation setup
        self.elevator_motor_model = DCMotor.neoVortex(1)
        self.arm_motor_model = DCMotor.NEO(1)
        self.elevator_sim = ElevatorSim(self.elevator_motor_model, SimulationRobotConstants.kElevatorGearing,
                                        SimulationRobotConstants.kCarriageMass, SimulationRobotConstants.kElevatorDrumRadius,
                                        SimulationRobotConstants.kMinElevatorHeightMeters, SimulationRobotConstants.kMaxElevatorHeightMeters)
        
        self.arm_sim = SingleJointedArmSim(self.arm_motor_model, SimulationRobotConstants.kArmReduction, 
                                           SimulationRobotConstants.kArmLength)

        # SmartDashboard - Mechanism visualization
        self.mechanism_2d = wpilib.SmartDashboard.putData(50, 50)

    def move_to_setpoint(self):
        # Using SPARK controller reference to set targets
        self.arm_motor.set(self.arm_current_target)
        self.elevator_motor.set(self.elevator_current_target)
    
    def zero_elevator_on_limit_switch(self):
        if not self.was_reset_by_limit and self.elevator_motor.getReverseLimitSwitch().get():
            self.elevator_encoder.setPosition(0)
            self.was_reset_by_limit = True
        elif not self.elevator_motor.getReverseLimitSwitch().get():
            self.was_reset_by_limit = False

    def zero_on_user_button(self):
        if not self.was_reset_by_button and RobotController.getUserButton():
            self.was_reset_by_button = True
            self.arm_encoder.setPosition(0)
            self.elevator_encoder.setPosition(0)
        elif not RobotController.getUserButton():
            self.was_reset_by_button = False
    
    def set_intake_power(self, power):
        self.intake_motor.set(power)
    
    def set_setpoint_command(self, setpoint):
        if setpoint == CoralSubsystemConstants.ArmSetpoints.kFeederStation:
            self.arm_current_target = CoralSubsystemConstants.ArmSetpoints.kFeederStation
            self.elevator_current_target = CoralSubsystemConstants.ElevatorSetpoints.kFeederStation
        elif setpoint == CoralSubsystemConstants.Setpoint.kLevel1:
            self.arm_current_target = CoralSubsystemConstants.ArmSetpoints.kLevel1
            self.elevator_current_target = CoralSubsystemConstants.ElevatorSetpoints.kLevel1
        # Add other setpoints as necessary

    def run_intake_command(self):
        self.set_intake_power(CoralSubsystemConstants.IntakeSetpoints.kForward)
    
    def reverse_intake_command(self):
        self.set_intake_power(CoralSubsystemConstants.IntakeSetpoints.kReverse)

    def periodic(self):
        self.move_to_setpoint()
        self.zero_elevator_on_limit_switch()
        self.zero_on_user_button()

        # Update SmartDashboard
        SmartDashboard.putNumber("Coral/Arm/Target Position", self.arm_current_target)
        SmartDashboard.putNumber("Coral/Arm/Actual Position", self.arm_encoder.getPosition())
        SmartDashboard.putNumber("Coral/Elevator/Target Position", self.elevator_current_target)
        SmartDashboard.putNumber("Coral/Elevator/Actual Position", self.elevator_encoder.getPosition())
        SmartDashboard.putNumber("Coral/Intake/Applied Output", self.intake_motor.getAppliedOutput())

    def simulation_periodic(self):
        # Update simulation models
        self.elevator_sim.setInput(self.elevator_motor.getAppliedOutput() * RobotController.getBatteryVoltage())
        self.arm_sim.setInput(self.arm_motor.getAppliedOutput() * RobotController.getBatteryVoltage())
        
        # Update elevator and arm simulation
        self.elevator_sim.update(0.02)
        self.arm_sim.update(0.02)

        # Iteration of elevator and arm motor simulation
        self.elevator_motor.simulate(self.elevator_sim)
        self.arm_motor.simulate(self.arm_sim)

