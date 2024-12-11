import wpimath

class GamepadUtils:
    @staticmethod
    def squareInput(value, deadband):
        """Squares the input value with deadband handling."""
        value_with_deadband = wpimath.applyDeadband(value, deadband)
        return (value_with_deadband ** 2) * (1 if value >= 0 else -1)