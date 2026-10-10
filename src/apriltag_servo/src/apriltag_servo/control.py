"""ROS-independent, forward-only Ackermann preview calculation."""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Parameters:
    wheelbase: float = 0.839
    max_steering: float = math.radians(50.0)
    max_speed: float = 0.15
    desired_distance: float = 1.0
    distance_deadband: float = 0.05
    distance_gain: float = 0.3
    acceleration: float = 0.15
    minimum_forward_target: float = 0.05

    def validate(self):
        values = vars(self)
        if not all(math.isfinite(v) for v in values.values()):
            raise ValueError('controller parameters must be finite')
        if (self.wheelbase <= 0 or not 0 < self.max_steering < math.pi / 2
                or self.max_speed <= 0 or self.desired_distance <= 0
                or self.distance_deadband < 0 or self.distance_gain <= 0
                or self.acceleration <= 0 or self.minimum_forward_target < 0):
            raise ValueError('invalid Ackermann controller parameters')


@dataclass(frozen=True)
class Command:
    speed: float = 0.0
    steering: float = 0.0
    yaw_rate: float = 0.0
    reason: str = 'stopped'


def calculate(x, y, forward_distance, previous_speed, dt, params):
    """Direct camera forward/left axes; distance is raw AprilTag optical z.

    No mounting translation, height, steering or range compensation is applied.
    Ackermann curvature is a minimal camera-based approximation. Stops bypass
    slew limits; ordinary speed changes are limited by acceleration.
    """
    params.validate()
    if not all(math.isfinite(v) for v in (x, y, forward_distance, previous_speed, dt)):
        return Command(reason='invalid_geometry')
    if forward_distance <= 0 or dt <= 0 or previous_speed < 0:
        return Command(reason='invalid_geometry')
    if x <= params.minimum_forward_target:
        return Command(reason='target_behind_or_too_lateral')
    error = forward_distance - params.desired_distance
    if error <= params.distance_deadband:
        return Command(reason='desired_distance_reached')
    radius_squared = x * x + y * y
    curvature = 2.0 * y / radius_squared
    delta = max(-params.max_steering,
                min(params.max_steering, math.atan(params.wheelbase * curvature)))
    desired_speed = min(params.max_speed, params.distance_gain * error)
    step = params.acceleration * min(dt, 0.1)
    speed = max(0.0, min(params.max_speed,
                        max(previous_speed - step, min(previous_speed + step, desired_speed))))
    yaw_rate = speed * math.tan(delta) / params.wheelbase
    return Command(speed, delta, yaw_rate, 'tracking_preview')
