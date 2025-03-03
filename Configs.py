import math
from rev import SparkMaxConfig, SparkFlexConfig 
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
            Configs.MAXSwerveModule.drivingConfig.encoder \
                .positionConversionFactor(drivingFactor) \
                .velocityConversionFactor(drivingFactor / 60.0)  # meters and meters per second
            Configs.MAXSwerveModule.drivingConfig.closedLoop \
                .pid(0, 0, 0) \
                .velocityFF(drivingVelocityFeedForward) \
                .outputRange(-1, 1)


            # Configure turning motor settings
            Configs.MAXSwerveModule.turningConfig \
                .smartCurrentLimit(20) \
                .setIdleMode(idleMode=SparkMaxConfig.IdleMode.kBrake)
            Configs.MAXSwerveModule.turningConfig.absoluteEncoder \
                .inverted(True) \
                .positionConversionFactor(turningFactor) \
                .velocityConversionFactor(turningFactor / 60.0)  # radians and radians per second
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
        def initialize():
            # Configure arm motor
            Configs.CoralSubsystem.armConfig \
                .setIdleMode(idleMode=SparkMaxConfig.IdleMode.kCoast) \
                .smartCurrentLimit(40) \
                .voltageCompensation(12)
            Configs.CoralSubsystem.armConfig.closedLoop \
                .setFeedbackSensor(Configs.CoralSubsystem.armConfig.closedLoop.FeedbackSensor.kPrimaryEncoder) \
                .pid(0.1,0,0) \
                .outputRange(-1, 1) \
            #!!!!!!!!The Numbers Might need to get changed!!!!!!!! -----Alex
            Configs.CoralSubsystem.armConfig.closedLoop.maxMotion \
                .maxVelocity(2000) \
                .maxAcceleration(10000) \
                .allowedClosedLoopError(0.25)

            # Configure elevator motor
            Configs.CoralSubsystem.elevatorConfig \
                .setIdleMode(idleMode=SparkFlexConfig.IdleMode.kCoast) \
                .smartCurrentLimit(50) \
                .voltageCompensation(12)
            Configs.CoralSubsystem.elevatorConfig.limitSwitch \
                .reverseLimitSwitchEnabled(True) \
                .reverseLimitSwitchType(Configs.CoralSubsystem.elevatorConfig.limitSwitch.Type.kNormallyOpen)
            Configs.CoralSubsystem.elevatorConfig.closedLoop \
                .outputRange(-1, 1) \
                .pid(0.1,0,0) \
                .setFeedbackSensor(Configs.CoralSubsystem.armConfig.closedLoop.FeedbackSensor.kPrimaryEncoder)
            Configs.CoralSubsystem.elevatorConfig.closedLoop.maxMotion \
                .maxVelocity(4200) \
                .maxAcceleration(6000) \
                .allowedClosedLoopError(0.5)

            # Configure intake motor
            Configs.CoralSubsystem.intakeConfig \
                .inverted(True) \
                .setIdleMode(idleMode=SparkMaxConfig.IdleMode.kBrake) \
                .smartCurrentLimit(40)

    class AlgaeSubsystem:
        intakeConfig = SparkMaxConfig()
        armConfig = SparkFlexConfig()

        @staticmethod
        def initialize():
            # Configure arm motor
            Configs.AlgaeSubsystem.armConfig \
                .smartCurrentLimit(40) \
                .setIdleMode(SparkFlexConfig.IdleMode.kBrake)
            Configs.AlgaeSubsystem.armConfig.closedLoop \
                .setFeedbackSensor(Configs.AlgaeSubsystem.armConfig.closedLoop.FeedbackSensor.kPrimaryEncoder) \
                .pid(0.1,0,0) \
                .outputRange(-0.5, 0.5)
            Configs.AlgaeSubsystem.armConfig.closedLoop.maxMotion \
                .maxVelocity(2000) \
                .maxAcceleration(10000) \
                .allowedClosedLoopError(0.25)

            # Configure intake motor
            Configs.AlgaeSubsystem.intakeConfig \
                .inverted(True) \
                .setIdleMode(SparkMaxConfig.IdleMode.kBrake) \
                .smartCurrentLimit(40)

# Call initialization functions
Configs.MAXSwerveModule.initialize()
Configs.CoralSubsystem.initialize()
Configs.AlgaeSubsystem.initialize()
