import math
from rev import SparkMaxConfig, SparkFlexConfig 
from Constants import ModuleConstants

class Configs:
    class MAXSwerveModule:
        drivingConfig = SparkFlexConfig()
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
                .setFeedbackSensor(Configs.MAXSwerveModule.turningConfig.closedLoop.FeedbackSensor.kAbsoluteEncoder) # this might need to change to Primary encoder for the feedback loop, but I doubt it.closedLoop.positionWrappingInputRange(0, turningFactor)

    class CoralSubsystem:
        armConfig = SparkMaxConfig()
        elevatorConfig = SparkFlexConfig()
        intakeConfig = SparkMaxConfig()

        @staticmethod
        def configure():
            # Configure arm motor
            Configs.CoralSubsystem.armConfig.setIdleMode(idleMode=SparkMaxConfig.IdleMode.kCoast)
            Configs.CoralSubsystem.armConfig.smartCurrentLimit(40)  
            Configs.CoralSubsystem.armConfig.voltageCompensation(12)
            Configs.CoralSubsystem.armConfig.closedLoop.setFeedbackSensor(Configs.CoralSubsystem.armConfig.closedLoop.FeedbackSensor.kPrimaryEncoder)
            Configs.CoralSubsystem.armConfig.closedLoop.pid(0.1,0,0)
            Configs.CoralSubsystem.armConfig.closedLoop.outputRange(-1, 1)
            #!!!!!!!!The Numbers Might need to get changed!!!!!!!! -----Alex
            Configs.CoralSubsystem.armConfig.closedLoop.maxMotion.maxVelocity(2000).maxAcceleration(10000).allowedClosedLoopError(0.25)

            # Configure elevator motor
            Configs.CoralSubsystem.elevatorConfig.setIdleMode(idleMode=SparkMaxConfig.IdleMode.kCoast)
            Configs.CoralSubsystem.elevatorConfig.smartCurrentLimit(50)
            Configs.CoralSubsystem.elevatorConfig.voltageCompensation(12)
            Configs.CoralSubsystem.elevatorConfig.limitSwitch.reverseLimitSwitchEnabled(True)
            Configs.CoralSubsystem.elevatorConfig.closedLoop.outputRange(-1, 1)
            Configs.CoralSubsystem.elevatorConfig.closedLoop.maxMotion.maxVelocity(4200).maxAcceleration(6000).allowedClosedLoopError(0.5)

            # Configure intake motor
            Configs.CoralSubsystem.intakeConfig.inverted(True)
            Configs.CoralSubsystem.intakeConfig.setIdleMode(idleMode=SparkMaxConfig.IdleMode.kBrake)
            Configs.CoralSubsystem.intakeConfig.smartCurrentLimit(40)

    class AlgaeSubsystem:
        intakeConfig = SparkFlexConfig()
        armConfig = SparkFlexConfig()

        @staticmethod
        def configure():
            # Configure arm motor
            Configs.AlgaeSubsystem.armConfig.smartCurrentLimit(40)
            Configs.AlgaeSubsystem.armConfig.closedLoop.setFeedbackSensor(Configs.AlgaeSubsystem.armConfig.closedLoop.FeedbackSensor.kPrimaryEncoder)
            Configs.AlgaeSubsystem.armConfig.closedLoop.pid(0.1,0,0)
            Configs.AlgaeSubsystem.armConfig.closedLoop.outputRange(-0.5, 0.5)

            # Configure intake motor
            Configs.AlgaeSubsystem.intakeConfig.inverted(True)
            Configs.AlgaeSubsystem.intakeConfig.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
            Configs.AlgaeSubsystem.intakeConfig.smartCurrentLimit(40)

# Call initialization functions
Configs.MAXSwerveModule.initialize()
Configs.CoralSubsystem.configure()
Configs.AlgaeSubsystem.configure()
