class PIDGains:
    def __init__(self, p, i, d):
        """Constructor for the PIDGains class."""
        self.p = p
        self.i = i
        self.d = d

    @staticmethod
    def set_spark_max_gains(controller, gains):
        """Sets the PID gains for a SparkMax PID controller."""
        controller.setP(gains.p)
        controller.setI(gains.i)
        controller.setD(gains.d)
