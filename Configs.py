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
                .IdleMode.kBrake
            Configs.MAXSwerveModule.drivingConfig.encoder.positionConversionFactor(drivingFactor)  # meters
            Configs.MAXSwerveModule.drivingConfig.encoder.velocityConversionFactor(drivingFactor / 60.0)  # meters per second
            Configs.MAXSwerveModule.drivingConfig.closedLoop \
                .pid(0.45, 0, 0) \
                .velocityFF(drivingVelocityFeedForward) \
                .outputRange(-1, 1)\
                .FeedbackSensor.kPrimaryEncoder

            # Configure turning motor settings
            Configs.MAXSwerveModule.turningConfig \
                .smartCurrentLimit(20) \
                .IdleMode.kBrake
            Configs.MAXSwerveModule.turningConfig.absoluteEncoder.inverted(True) \
                .positionConversionFactor(turningFactor)  # radians
            Configs.MAXSwerveModule.turningConfig.absoluteEncoder.velocityConversionFactor(turningFactor / 60.0)  # radians per second
            Configs.MAXSwerveModule.turningConfig.closedLoop \
                .pid(1, 0, 0) \
                .outputRange(-1, 1) \
                .positionWrappingEnabled(True) \
                .positionWrappingInputRange(0, turningFactor) \
                .FeedbackSensor.kAbsoluteEncoder # this might need to change to Primary encoder for the feedback loop, but I doubt it.

# Call the initialization function
Configs.MAXSwerveModule.initialize()