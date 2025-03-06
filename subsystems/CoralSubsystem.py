#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

#import math
#import wpilib
#from wpimath import units
from rev import SparkMax, SparkLowLevel, SparkFlex, SparkBase
#from wpilib.simulation import ElevatorSim, SingleJointedArmSim
from commands2 import Subsystem
#from wpilib import SmartDashboard
#from wpimath.system.plant import DCMotor
#from wpilib import RobotController
from Configs import Configs

# Constants
from Constants import CoralSubsystemConstants #, SimulationRobotConstants

class CoralSubsystem(Subsystem):
    """The Coral Subsystem"""
    class DisplaySetpoint:
        """Sets up language for SmartDashboard for the elevator Position"""
        kFeederStation = "FeederStation"
        kLevel1 = "Level1"
        kLevel2 = "Level2"
        kLevel3 = "Level3"
        kLevel4 = "Level4" 
  
    def __init__(self):
        super().__init__()
   
        # Motor controllers
        self.arm_motor = SparkMax(
            CoralSubsystemConstants.kArmMotorCanId, 
            SparkLowLevel.MotorType.kBrushless
            )
        self.arm_motor.configure(
            Configs.CoralSubsystem.armConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
            )
        self.arm_encoder = self.arm_motor.getEncoder()
        self.arm_controller = self.arm_motor.getClosedLoopController()
        self.arm_encoder.setPosition(0)
        
        self.elevator_motor = SparkFlex(
            CoralSubsystemConstants.kElevatorMotorCanId,
            SparkLowLevel.MotorType.kBrushless
            )
        self.elevator_motor.configure(
            Configs.CoralSubsystem.elevatorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
            )
        self.elevator_encoder = self.elevator_motor.getEncoder()
        self.elevator_controller = self.elevator_motor.getClosedLoopController()
        
        self.intake_motor = SparkMax(
            CoralSubsystemConstants.kIntakeMotorCanId,
            SparkLowLevel.MotorType.kBrushless
            )
        self.intake_motor.configure(
            Configs.CoralSubsystem.intakeConfig,
             SparkBase.ResetMode.kResetSafeParameters,
             SparkBase.PersistMode.kPersistParameters)
        
        # State variables
        self.was_reset_by_button = False
        self.was_reset_by_limit = False
        self.arm_current_target = CoralSubsystemConstants.ArmSetpoints.kFeederStation
        self.elevator_current_target = CoralSubsystemConstants.ElevatorSetpoints.kFeederStation
  
        '''
        # Simulation setup
        self.elevator_motor_model = DCMotor.neoVortex(1)
        self.arm_motor_model = DCMotor.NEO(1)
        
        self.elevator_sim = ElevatorSim(gearbox=self.elevator_motor_model, gearing=SimulationRobotConstants.kElevatorGearing,
                                        carriageMass= units.meters(SimulationRobotConstants.kCarriageMass), drumRadius=SimulationRobotConstants.kElevatorDrumRadius,
                                        minHeight=SimulationRobotConstants.kMinElevatorHeightMeters, maxHeight=SimulationRobotConstants.kMaxElevatorHeightMeters)
        
        self.arm_sim = SingleJointedArmSim(self.arm_motor_model, SimulationRobotConstants.kArmReduction, 
                                           SimulationRobotConstants.kArmLength)
        
   
        # SmartDashboard - Mechanism visualization
        self.mechanism_2d = wpilib.Mechanism2d(100, 100)
        #wpilib.SmartDashboard.putData("Coral Mechanism", self.mechanism_2d)
        '''
  
    def move_to_setpoint(self):
        """Using SPARK controller reference to set targets with MAXMotion"""
        self.arm_controller.setReference(
            self.arm_current_target,
            SparkLowLevel.ControlType.kMAXMotionPositionControl
            )
        self.elevator_controller.setReference(
            self.elevator_current_target,
            SparkLowLevel.ControlType.kMAXMotionPositionControl
            )
   
    def zero_elevator_on_limit_switch(self):
        """sets elevator to zero when it sees a limit switch"""
        if not self.was_reset_by_limit and self.elevator_motor.getReverseLimitSwitch().get():
            self.elevator_encoder.setPosition(0)
            self.was_reset_by_limit = True
        elif not self.elevator_motor.getReverseLimitSwitch().get():
            self.was_reset_by_limit = False
  
    def zero_on_user_button(self):
        """zeroes motor on user button"""
        print("Zeroed")

        '''
        # Methods for the reset of the elevator position, I assume for sim
        if not self.was_reset_by_button and RobotController.getUserButton():
            self.was_reset_by_button = True
            self.elevator_encoder.setPosition(self.elevator_sim.getPosition())
            self.arm_encoder.setPosition(math.degrees(self.arm_sim.getAngle()))
   
        elif not RobotController.getUserButton():
            self.was_reset_by_button = False
        '''
   
    def set_intake_power(self, power):
        """Sets the intake power of the coral"""
        self.intake_motor.set(power)
  
    def set_setpoint_command(self, setpoint):
        """Changes the desired setpoint that self.move_to_setpoint is looking for"""
        if setpoint == CoralSubsystemConstants.ElevatorSetpoints.kFeederStation:
            self.arm_current_target = CoralSubsystemConstants.ArmSetpoints.kFeederStation
            self.elevator_current_target = CoralSubsystemConstants.ElevatorSetpoints.kFeederStation
        elif setpoint == CoralSubsystemConstants.ElevatorSetpoints.kLevel1:
            self.arm_current_target = CoralSubsystemConstants.ArmSetpoints.kLevel1
            self.elevator_current_target = CoralSubsystemConstants.ElevatorSetpoints.kLevel1
        elif setpoint == CoralSubsystemConstants.ElevatorSetpoints.kLevel2:
            self.arm_current_target = CoralSubsystemConstants.ArmSetpoints.kLevel2
            self.elevator_current_target = CoralSubsystemConstants.ElevatorSetpoints.kLevel2
        elif setpoint == CoralSubsystemConstants.ElevatorSetpoints.kLevel3:
            self.arm_current_target = CoralSubsystemConstants.ArmSetpoints.kLevel3
            self.elevator_current_target = CoralSubsystemConstants.ElevatorSetpoints.kLevel3
        elif setpoint == CoralSubsystemConstants.ElevatorSetpoints.kLevel4:
            self.arm_current_target = CoralSubsystemConstants.ArmSetpoints.kLevel4
            self.elevator_current_target = CoralSubsystemConstants.ElevatorSetpoints.kLevel4
  
    def run_intake_command(self):
        """Sets intake to pull in coral"""
        self.set_intake_power(CoralSubsystemConstants.IntakeSetpoints.kForward)
  
    def reverse_intake_command(self):
        """Sets intake to eject Coral"""
        self.set_intake_power(CoralSubsystemConstants.IntakeSetpoints.kReverse)

    def stop_intake_command(self):
        self.set_intake_power(0)
 
    def periodic(self):
        self.move_to_setpoint()
        self.zero_elevator_on_limit_switch()
        #self.zero_on_user_button()
        
        '''
        # Update SmartDashboard
        SmartDashboard.putNumber("Coral/Arm/Target Position", self.arm_current_target)
        SmartDashboard.putNumber("Coral/Arm/Actual Position", self.arm_encoder.getPosition())
        SmartDashboard.putNumber("Coral/Elevator/Target Position", self.elevator_current_target)
        SmartDashboard.putNumber("Coral/Elevator/Actual Position", self.elevator_encoder.getPosition())
        SmartDashboard.putNumber("Coral/Intake/Applied Output", self.intake_motor.getAppliedOutput())
        '''

    '''
    # Simulation periodic changes
    def simulation_periodic(self):
        # Update simulation models
        self.elevator_sim.setInput(self.elevator_motor.getAppliedOutput() * RobotController.getBatteryVoltage())
        self.arm_sim.setInput(self.arm_motor.getAppliedOutput() * RobotController.getBatteryVoltage())
        
        # Update elevator and arm simulation
        self.elevator_sim.update(0.02)
        self.arm_sim.update(0.02)
   
    def get_simulation_current_draw(self):
        return self.elevator_sim.getCurrentDraw() + self.arm_sim.getCurrentDraw()
    '''