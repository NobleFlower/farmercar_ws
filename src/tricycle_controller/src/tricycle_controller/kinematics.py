"""ROS-free rear-axle bicycle model for a single front-steering tricycle."""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Limits:
    wheelbase: float = 0.839
    max_steering: float = math.radians(50.0)
    max_forward_speed: float = 0.15
    max_reverse_speed: float = 0.15
    zero_speed_tolerance: float = 1e-6
    lateral_tolerance: float = 1e-6
    yaw_tolerance: float = 1e-6

    def validate(self):
        if not all(math.isfinite(v) for v in vars(self).values()):
            raise ValueError('limits must be finite')
        if (self.wheelbase <= 0 or not 0 < self.max_steering < math.pi / 2
                or self.max_forward_speed <= 0 or self.max_reverse_speed < 0
                or self.zero_speed_tolerance <= 0 or self.lateral_tolerance < 0
                or self.yaw_tolerance < 0):
            raise ValueError('invalid kinematic limits')


@dataclass(frozen=True)
class Command:
    speed: float = 0.0
    steering_angle: float = 0.0
    yaw_rate: float = 0.0
    accepted: bool = False
    saturated: bool = False
    reason: str = 'waiting_for_command'


def convert(vx, vy, yaw_rate, limits):
    """vx m/s, vy m/s, yaw_rate rad/s -> speed m/s, front steering rad.

    Nonzero lateral velocity is rejected rather than discarded. Steering is
    computed with the SIGNED vx: reverse turning must reverse the steering sign.
    Saturation preserves requested curvature when limiting speed, then clips
    front steering and recomputes achievable yaw rate. No actuator dynamics,
    servo angle offsets, camera geometry, odometry or wheel feedback are assumed.
    """
    limits.validate()
    if not all(math.isfinite(v) for v in (vx, vy, yaw_rate)):
        return Command(reason='nonfinite_velocity')
    if abs(vy) > limits.lateral_tolerance:
        return Command(reason='lateral_velocity_not_supported')
    if abs(vx) <= limits.zero_speed_tolerance:
        if abs(yaw_rate) > limits.yaw_tolerance:
            return Command(reason='in_place_rotation_not_supported')
        return Command(accepted=True, reason='stop')
    if vx < 0 and limits.max_reverse_speed == 0:
        return Command(reason='reverse_disabled')
    requested_steering = math.atan(limits.wheelbase * yaw_rate / vx)
    speed = max(-limits.max_reverse_speed, min(limits.max_forward_speed, vx))
    steering = max(-limits.max_steering, min(limits.max_steering, requested_steering))
    achievable_yaw = speed * math.tan(steering) / limits.wheelbase
    saturated = speed != vx or steering != requested_steering
    return Command(speed, steering, achievable_yaw, True, saturated,
                   'limited' if saturated else 'ok')
