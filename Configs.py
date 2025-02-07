import math
from rev import SparkMaxConfig
from Constants import ModuleConstants

class Configs:
    class MAXSwerveModule: 
        drivingConfig = SparkMaxConfig()
        turningConfig = SparkMaxConfig()

        @staticmethod
        def initialize():
            # Use module constants to calculate conversion factors and feed forward gain.
            drivingFactor = ModuleConstants.kWheelDiameterMeters * math.pi / ModuleConstants.kDrivingMotorReduction
            turningFactor = 2 * math.pi
            drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps

            # Configure driving motor settings
            Configs.MAXSwerveModule.drivingConfig \
                .smartCurrentLimit(50) \
                .setIdleMode(idleMode=SparkMaxConfig.IdleMode.kBrake)
            Configs.MAXSwerveModule.drivingConfig.encoder.positionConversionFactor(drivingFactor) 
            Configs.MAXSwerveModule.drivingConfig.encoder.velocityConversionFactor(drivingFactor / 60.0)   
            Configs.MAXSwerveModule.drivingConfig.closedLoop \
                .pid(0, 0, 0) \
                .velocityFF(drivingVelocityFeedForward) \
                .outputRange(-1, 1)

            # Configure turning motor settings
            Configs.MAXSwerveModule.turningConfig \
                .smartCurrentLimit(20) \
                .setIdleMode(idleMode=SparkMaxConfig.IdleMode.kBrake)
            Configs.MAXSwerveModule.turningConfig.absoluteEncoder.inverted(True)
            Configs.MAXSwerveModule.turningConfig.absoluteEncoder.positionConversionFactor(turningFactor)  # radians
            Configs.MAXSwerveModule.turningConfig.absoluteEncoder.velocityConversionFactor(turningFactor / 60.0)  # radians per second
            Configs.MAXSwerveModule.turningConfig.closedLoop \
                .pid(1, 0, 0) \
                .outputRange(-1, 1) \
                .positionWrappingEnabled(True) \
                .positionWrappingInputRange(0, turningFactor) \
                .setFeedbackSensor(Configs.MAXSwerveModule.turningConfig.closedLoop.FeedbackSensor.kAbsoluteEncoder) # this might need to change to Primary encoder for the feedback loop, but I doubt it.
    class CoralSubsystem:
        # Configuring the arm, elevator, and intake motors
        armConfig = SparkMaxConfig()
        elevatorConfig = SparkMaxConfig()
        intakeConfig = SparkMaxConfig()

        @staticmethod
        def configure():
            # Configuring arm motor
            Configs.CoralSubsystem.armConfig.setIdleMode(idleMode=SparkMaxConfig.IdleMode.kCoast)
            Configs.CoralSubsystem.armConfig.setSmartCurrentLimit(40)
            Configs.CoralSubsystem.armConfig.setVoltageCompensation(12)

            # Configuring arm motor closed loop controller
            Configs.CoralSubsystem.armConfig.setFeedbackSensor(idleMode=SparkMaxConfig.FeedbackSensor.kPrimaryEncoder)
            Configs.CoralSubsystem.armConfig.setPID(0.1, 0, 0)
            Configs.CoralSubsystem.armConfig.setOutputRange(-1, 1)
            Configs.CoralSubsystem.armConfig.setMaxMotion(max_velocity=2000, max_acceleration=10000)
            Configs.CoralSubsystem.armConfig.setAllowedClosedLoopError(0.25)

            # Configuring elevator motor
            Configs.CoralSubsystem.elevatorConfig.setIdleMode(idleMode=SparkMaxConfig.IdleMode.kCoast)
            Configs.CoralSubsystem.elevatorConfig.setSmartCurrentLimit(50)
            Configs.CoralSubsystem.elevatorConfig.setVoltageCompensation(12)

            # Configuring the elevator reverse limit switch
            Configs.CoralSubsystem.elevatorConfig.setReverseLimitSwitchEnabled(True)
            Configs.CoralSubsystem.elevatorConfig.setReverseLimitSwitchType(SparkMaxConfig.Spark.LimitSwitchType.kNormallyOpen)

            # Configuring elevator motor closed loop controller
            Configs.CoralSubsystem.elevatorConfig.setFeedbackSensor(SparkMaxConfig.FeedbackSensor.kPrimaryEncoder)
            Configs.CoralSubsystem.elevatorConfig.setPID(0.1, 0, 0)
            Configs.CoralSubsystem.elevatorConfig.setOutputRange(-1, 1)
            Configs.CoralSubsystem.elevatorConfig.setMaxMotion(max_velocity=4200, max_acceleration=6000)
            Configs.CoralSubsystem.elevatorConfig.setAllowedClosedLoopError(0.5)

            # Configuring intake motor
            Configs.CoralSubsystem.intakeConfig.setInverted(True)
            Configs.CoralSubsystem.intakeConfig.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
            Configs.CoralSubsystem.intakeConfig.setSmartCurrentLimit(40)

    class AlgaeSubsystem:
        # Configuring the intake and arm motors for the Algae subsystem
        intakeConfig = SparkMaxConfig()
        armConfig = SparkMaxConfig()

        @staticmethod
        def configure():
            # Configuring the arm motor settings
            Configs.AlgaeSubsystem.armConfig.setSmartCurrentLimit(40)
            Configs.AlgaeSubsystem.armConfig.setPID(0.1, 0, 0)
            Configs.AlgaeSubsystem.armConfig.setOutputRange(-0.5, 0.5)

            # Configuring the intake motor settings
            Configs.AlgaeSubsystem.intakeConfig.setInverted(True)
            Configs.AlgaeSubsystem.intakeConfig.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
            Configs.AlgaeSubsystem.intakeConfig.setSmartCurrentLimit(40)
# Call the initialization function
Configs.MAXSwerveModule.initialize()