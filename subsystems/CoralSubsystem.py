#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
from wpimath import units
from rev import SparkMax, SparkLowLevel, SparkFlex, SparkBase
from wpilib.simulation import ElevatorSim, SingleJointedArmSim
from commands2 import Subsystem
from wpilib import SmartDashboard
from wpimath.system.plant import DCMotor
from wpilib import RobotController
from commands2 import Subsystem
from Configs import Configs

# Constants
from Constants import CoralSubsystemConstants, SimulationRobotConstants

class CoralSubsystem(Subsystem):
     class Setpoint:
        kFeederStation = "FeederStation"
        kLevel1 = "Level1"
        kLevel2 = "Level2"
        kLevel3 = "Level3"
        kLevel4 = "Level4"
    
     def __init__(self):
        super().__init__()

        # Motor controllers
        self.arm_motor = SparkMax(CoralSubsystemConstants.kArmMotorCanId, SparkLowLevel.MotorType.kBrushless)
        self.arm_encoder = self.arm_motor.getEncoder()
        
        self.elevator_motor = SparkFlex(CoralSubsystemConstants.kElevatorMotorCanId, SparkLowLevel.MotorType.kBrushless)
        self.elevator_encoder = self.elevator_motor.getEncoder()
        
        self.intake_motor = SparkMax(CoralSubsystemConstants.kIntakeMotorCanId, SparkLowLevel.MotorType.kBrushless)

        self.elevator_motor.configure(Configs.CoralSubsystem.elevatorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters)
        self.intake_motor.configure(Configs.CoralSubsystem.intakeConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters)

        
        # State variables
        self.was_reset_by_button = False
        self.was_reset_by_limit = False
        self.arm_current_target = CoralSubsystemConstants.ArmSetpoints.kFeederStation
        self.elevator_current_target = CoralSubsystemConstants.ElevatorSetpoints.kFeederStation

        # Simulation setup
        self.elevator_motor_model = DCMotor.neoVortex(1)
        self.arm_motor_model = DCMotor.NEO(1)
        '''
        self.elevator_sim = ElevatorSim(gearbox=self.elevator_motor_model, gearing=SimulationRobotConstants.kElevatorGearing,
                                        carriageMass= units.meters(SimulationRobotConstants.kCarriageMass), drumRadius=SimulationRobotConstants.kElevatorDrumRadius,
                                        minHeight=SimulationRobotConstants.kMinElevatorHeightMeters, maxHeight=SimulationRobotConstants.kMaxElevatorHeightMeters)
        
        self.arm_sim = SingleJointedArmSim(self.arm_motor_model, SimulationRobotConstants.kArmReduction, 
                                           SimulationRobotConstants.kArmLength)
      '''

        # SmartDashboard - Mechanism visualization
        self.mechanism_2d = wpilib.Mechanism2d(100, 100)
        wpilib.SmartDashboard.putData("Coral Mechanism", self.mechanism_2d)

     def move_to_setpoint(self):
        # Using SPARK controller reference to set targets
        self.arm_motor.getClosedLoopController().setReference(self.arm_current_target, SparkFlex.ControlType.kMAXMotionPositionControl)
        self.elevator_motor.getClosedLoopController().setReference(self.elevator_current_target, SparkFlex.ControlType.kMAXMotionPositionControl)
      
     def elv_up(self):
         self.elevator_motor.set(.99)

     def elv_down(self):
         self.elevator_motor.set(-0.99)

     def arm_run(self):
         self.arm_motor.set(.5)

     def arm_reverse(self):
         self.arm_motor.set(-0.5)

    
     def zero_elevator_on_limit_switch(self):
        if not self.was_reset_by_limit and self.elevator_motor.getReverseLimitSwitch().get():
            self.elevator_encoder.setPosition(0)
            self.was_reset_by_limit = True
        elif not self.elevator_motor.getReverseLimitSwitch().get():
            self.was_reset_by_limit = False

     def zero_on_user_button(self):
        if not self.was_reset_by_button and RobotController.getUserButton():
            self.was_reset_by_button = True
            self.elevator_encoder.setPosition(self.elevator_sim.getPosition())
            self.arm_encoder.setPosition(math.degrees(self.arm_sim.getAngle()))

        elif not RobotController.getUserButton():
            self.was_reset_by_button = False
    
     def set_intake_power(self, power):
        self.intake_motor.set(power)
    
     def set_setpoint_command(self, setpoint):
        if setpoint == CoralSubsystemConstants.ArmSetpoints.kFeederStation:
            self.arm_current_target = CoralSubsystemConstants.ArmSetpoints.kFeederStation
            self.elevator_current_target = CoralSubsystemConstants.ElevatorSetpoints.kFeederStation
        elif setpoint == CoralSubsystemConstants.ArmSetpoints.kLevel1:
            self.arm_current_target = CoralSubsystemConstants.ArmSetpoints.kLevel1
            self.elevator_current_target = CoralSubsystemConstants.ElevatorSetpoints.kLevel1
        elif setpoint == self.Setpoint.kLevel2:
            self.arm_current_target = CoralSubsystemConstants.ArmSetpoints.kLevel2
            self.elevator_current_target = CoralSubsystemConstants.ElevatorSetpoints.kLevel2
        elif setpoint == self.Setpoint.kLevel3:
            self.arm_current_target = CoralSubsystemConstants.ArmSetpoints.kLevel3
            self.elevator_current_target = CoralSubsystemConstants.ElevatorSetpoints.kLevel3
        elif setpoint == self.Setpoint.kLevel4:
            self.arm_current_target = CoralSubsystemConstants.ArmSetpoints.kLevel4
            self.elevator_current_target = CoralSubsystemConstants.ElevatorSetpoints.kLevel4


     def run_intake_command(self):
        self.set_intake_power(CoralSubsystemConstants.IntakeSetpoints.kForward)
    
     def reverse_intake_command(self):
        self.set_intake_power(CoralSubsystemConstants.IntakeSetpoints.kReverse)
      
     def stop_intake_command(self):
         self.set_intake_power(0)


     def periodic(self):
        #self.move_to_setpoint()
        #self.zero_elevator_on_limit_switch()
        # self.zero_on_user_button()
        pass

     def simulation_periodic(self):
        # Update simulation models
        self.elevator_sim.setInput(self.elevator_motor.getAppliedOutput() * RobotController.getBatteryVoltage())
        self.arm_sim.setInput(self.arm_motor.getAppliedOutput() * RobotController.getBatteryVoltage())
        
        # Update elevator and arm simulation
        self.elevator_sim.update(0.02)
        self.arm_sim.update(0.02)


     def get_simulation_current_draw(self):
        return self.elevator_sim.getCurrentDraw() + self.arm_sim.getCurrentDraw()


