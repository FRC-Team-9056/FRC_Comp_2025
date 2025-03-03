#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
from rev import SparkFlex, SparkLowLevel, SparkMax, SparkBase
#from wpilib import MechanismLigament2d
#from wpilib.simulation import SingleJointedArmSim
#from wpilib import SmartDashboard
from commands2 import Subsystem
from commands2 import Command
#from wpimath.system.plant import DCMotor
from Constants import AlgaeSubsystemConstants
from Configs import Configs

class AlgaeSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        # Initialize arm SPARK motor and encoder
        self.arm_motor = SparkFlex(AlgaeSubsystemConstants.kPivotMotorCanId, SparkLowLevel.MotorType.kBrushless)
        self.arm_motor.configure(Configs.AlgaeSubsystem.armConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        self.arm_encoder = self.arm_motor.getEncoder()
        self.arm_controller = self.arm_motor.getClosedLoopController()

        # Initialize intake SPARK motor
        self.intake_motor = SparkMax(AlgaeSubsystemConstants.kIntakeMotorCanId, SparkLowLevel.MotorType.kBrushless)
        self.intake_motor.configure(Configs.AlgaeSubsystem.intakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        # Initialize member variables
        self.stow_when_idle = True
        self.was_reset = False
        '''
        # Simulation setup for arm
        self.arm_motor_model = DCMotor.NEO(1)  # assuming Neo Motor
        self.arm_motor_sim = SingleJointedArmSim(
            self.arm_motor_model,
            SimulationRobotConstants.kIntakeReduction,
            SimulationRobotConstants.kIntakeLength,
            SimulationRobotConstants.kIntakeMinAngleRads,
            SimulationRobotConstants.kIntakeMaxAngleRads
        )
        
        # Mechanism2d setup for visualizing the subsystem
        self.intake_pivot_mechanism = MechanismLigament2d(50, 50)
        #self.mech2d_root = self.intake_pivot_mechanism("Ball Intake Root", 28, 3)
        self.intake_pivot_mechanism = self.intake_pivot_mechanism.appendLigament(
            wpilib.SmartDashboard.putData(
                "Intake Pivot",
                SimulationRobotConstants.kIntakeShortBarLength * SimulationRobotConstants.kPixelsPerMeter,
                math.degrees(SimulationRobotConstants.kIntakeMinAngleRads)
            )
        )
        '''
        #wpilib.SmartDashboard.putData("Algae Subsystem", self.)

        # Zero the arm encoder
        self.arm_encoder.setPosition(0)

    def zero_on_user_button(self):
        """Zero the arm encoder when the user button is pressed."""
        if not self.was_reset and wpilib.RobotController.getUserButton():
            self.was_reset = True
            self.arm_encoder.setPosition(0)
        elif not wpilib.RobotController.getUserButton():
            self.was_reset = False
    
    # Utility functions to set in more complex commands
    def set_intake_power(self, power):
        """Set the intake motor power."""
        self.intake_motor.set(power)

    def set_intake_position(self, position):
        """Set the arm motor position using closed-loop control."""
        self.arm_controller.setReference(position, SparkLowLevel.ControlType.kMAXMotionPositionControl)

    # Commands called in robotcontainer to do things
    def run_intake_command(self):
        """Command to run the intake motor and extend the arm."""
        self.stow_when_idle = False,
        self.set_intake_power(AlgaeSubsystemConstants.IntakeSetpoints.kForward),
        self.set_intake_position(AlgaeSubsystemConstants.ArmSetpoints.kDown)

    def reverse_intake_command(self):
        """Command to reverse the intake motor."""
        self.stow_when_idle = True
        self.set_intake_power(AlgaeSubsystemConstants.IntakeSetpoints.kReverse)
        self.set_intake_position(AlgaeSubsystemConstants.IntakeSetpoints.kHold)

    def stow_command(self):
        """Command to stow the intake arm."""
        self.stow_when_idle = True

    def idle_command(self):
        """Command to stop the intake and keep the arm in the stow position."""
        if self.stow_when_idle:
            self.set_intake_power(0)
            self.set_intake_position(AlgaeSubsystemConstants.ArmSetpoints.kStow)
        else:
            self.set_intake_power(AlgaeSubsystemConstants.IntakeSetpoints.kHold)
            self.set_intake_position(AlgaeSubsystemConstants.ArmSetpoints.kHold)

    def periodic(self):
        """Called periodically to update the status and display values."""
        self.zero_on_user_button()
        self.idle_command()

        """
        SmartDashboard.putNumber("Algae/Arm/Position", self.arm_encoder.getPosition())
        SmartDashboard.putNumber("Algae/Intake/Applied Output", self.intake_motor.getAppliedOutput())
        self.intake_pivot_mechanism.setAngle(
            math.degrees(SimulationRobotConstants.kIntakeMinAngleRads)
            + math.degrees(self.arm_encoder.getPosition() / SimulationRobotConstants.kIntakeReduction)
        )
        """
    """
    def get_simulation_current_draw(self):
        # Returns simulated current draw
        return self.arm_motor_sim.getCurrentDraw()
    
    def simulation_periodic(self):
        # Simulation logic for the subsystem.
        self.arm_motor_sim.setInput(self.arm_motor.getAppliedOutput() * wpilib.RobotController.getBatteryVoltage())
        self.arm_motor_sim.update(0.020)
        self.arm_encoder.setPosition(math.degrees(self.arm_motor_sim.getAngle()))
    """