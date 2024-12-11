import math

class SwerveUtils:
    @staticmethod
    def step_towards(current, target, stepsize):
        """Steps a value towards a target with a specified step size."""
        if abs(current - target) <= stepsize:
            return target
        elif target < current:
            return current - stepsize
        else:
            return current + stepsize

    @staticmethod
    def step_towards_circular(current, target, stepsize):
        """Steps a value (angle) towards a target (angle) taking the shortest path."""
        current = SwerveUtils.wrap_angle(current)
        target = SwerveUtils.wrap_angle(target)

        step_direction = math.copysign(1, target - current)
        difference = abs(current - target)

        if difference <= stepsize:
            return target
        elif difference > math.pi:
            # Handle the special case where you can reach the target in one step while also wrapping
            if abs(current + 2 * math.pi - target) < stepsize or abs(target + 2 * math.pi - current) < stepsize:
                return target
            else:
                return SwerveUtils.wrap_angle(current - step_direction * stepsize)
        else:
            return current + step_direction * stepsize

    @staticmethod
    def angle_difference(angle_a, angle_b):
        """Finds the (unsigned) minimum difference between two angles."""
        difference = abs(angle_a - angle_b)
        return difference if difference <= math.pi else 2 * math.pi - difference

    @staticmethod
    def wrap_angle(angle):
        """Wraps an angle until it lies within the range from 0 to 2*pi (exclusive)."""
        two_pi = 2 * math.pi

        if angle == two_pi:  # Handle floating-point errors at exactly 2*pi
            return 0.0
        elif angle > two_pi:
            return angle - two_pi * math.floor(angle / two_pi)
        elif angle < 0.0:
            return angle + two_pi * (math.floor((-angle) / two_pi) + 1)
        else:
            return angle
