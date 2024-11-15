#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#


from rev import CANSparkMax
from rev import SparkPIDController
from rev import AbsoluteEncoder, RelativeEncoder
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from Constants import ModuleConstants

class MAXSwerveModule:
    def __init__(self, drivingCANId, turningCANId, chassisAngularOffset):
        # Create Spark Max motor controllers for driving and turning
        self.m_drivingSparkMax = CANSparkMax(drivingCANId, CANSparkMax.MotorType.kBrushless)
        self.m_turningSparkMax = CANSparkMax(turningCANId, CANSparkMax.MotorType.kBrushless)

        # Restore factory defaults to ensure a known state
        self.m_drivingSparkMax.restoreFactoryDefaults()
        self.m_turningSparkMax.restoreFactoryDefaults()

        # Set up encoders and PID controllers for both motors
        self.m_drivingEncoder = self.m_drivingSparkMax.getEncoder()
        self.m_turningEncoder = self.m_turningSparkMax.getAbsoluteEncoder(AbsoluteEncoder.Type.kDutyCycle)
        self.m_drivingPIDController = self.m_drivingSparkMax.getPIDController()
        self.m_turningPIDController = self.m_turningSparkMax.getPIDController()
        self.m_drivingPIDController.setFeedbackDevice(self.m_drivingEncoder)
        self.m_turningPIDController.setFeedbackDevice(self.m_turningEncoder)

        # Apply position and velocity conversion factors for the driving encoder
        self.m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
        self.m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor)

        # Apply position and velocity conversion factors for the turning encoder
        self.m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
        self.m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor)

        # Invert the turning encoder based on configuration
        self.m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted)

        # Enable PID wrapping for the turning motor
        self.m_turningPIDController.setPositionPIDWrappingEnabled(True)
        self.m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput)
        self.m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput)

        # Set the PID gains for the driving motor
        self.m_drivingPIDController.setP(ModuleConstants.kDrivingP)
        self.m_drivingPIDController.setI(ModuleConstants.kDrivingI)
        self.m_drivingPIDController.setD(ModuleConstants.kDrivingD)
        self.m_drivingPIDController.setFF(ModuleConstants.kDrivingFF)
        self.m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput)

        # Set the PID gains for the turning motor
        self.m_turningPIDController.setP(ModuleConstants.kTurningP)
        self.m_turningPIDController.setI(ModuleConstants.kTurningI)
        self.m_turningPIDController.setD(ModuleConstants.kTurningD)
        self.m_turningPIDController.setFF(ModuleConstants.kTurningFF)
        self.m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput)

        # Configure motor idle modes and current limits
        self.m_drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode)
        self.m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode)
        self.m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit)
        self.m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit)

        # Burn flash to save configurations to non-volatile memory
        self.m_drivingSparkMax.burnFlash()
        self.m_turningSparkMax.burnFlash()

        # Set chassis angular offset
        self.m_chassisAngularOffset = chassisAngularOffset

        # Initialize desired state with the current turning angle
        self.m_desiredState = SwerveModuleState(0.0, Rotation2d(self.m_turningEncoder.getPosition()))
        self.m_drivingEncoder.setPosition(0)

    def getState(self):
        # Return the current state of the module
        # Apply chassis angular offset to the encoder position
        return SwerveModuleState(self.m_drivingEncoder.getVelocity(),
                                 Rotation2d(self.m_turningEncoder.getPosition() - self.m_chassisAngularOffset))

    def getPosition(self):
        # Return the current position of the module
        # Apply chassis angular offset to the encoder position
        return SwerveModulePosition(self.m_drivingEncoder.getPosition(),
                                    Rotation2d(self.m_turningEncoder.getPosition() - self.m_chassisAngularOffset))

    def setDesiredState(self, desiredState):
        # Apply chassis angular offset to the desired state
        correctedDesiredState = SwerveModuleState(desiredState.speedMetersPerSecond,
                                                  desiredState.angle.plus(Rotation2d.fromRadians(self.m_chassisAngularOffset)))

        # Optimize the reference state to avoid turning more than 90 degrees
        optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                                                           Rotation2d(self.m_turningEncoder.getPosition()))

        # Command the driving and turning SPARKS MAX towards their respective setpoints
        self.m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity)
        self.m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition)

        # Save the desired state
        self.m_desiredState = desiredState

    def resetEncoders(self):
        # Reset the driving encoder position to 0
        self.m_drivingEncoder.setPosition(0)