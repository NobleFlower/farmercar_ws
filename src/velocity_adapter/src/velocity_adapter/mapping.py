"""Stateless body-frame vector-to-nonholonomic velocity adaptation."""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Parameters:
    heading_gain: float = 0.5  # 1/s: heading error radians -> yaw rate rad/s
    allow_reverse: bool = True
    zero_tolerance: float = 1e-6

    def validate(self):
        if (not math.isfinite(self.heading_gain) or self.heading_gain <= 0
                or not math.isfinite(self.zero_tolerance) or self.zero_tolerance <= 0):
            raise ValueError('heading_gain and zero_tolerance must be finite and positive')


@dataclass(frozen=True)
class Adapted:
    vx: float = 0.0
    yaw_rate: float = 0.0
    heading_error: float = 0.0
    accepted: bool = False
    approximate: bool = False
    reason: str = 'waiting_for_command'


def adapt(vx, vy, yaw_rate, params):
    """Convert requested translation bearing to turning while travelling.

    vy=0: signed vx and yaw_rate pass through (differential input).
    vy!=0: signed norm gives travel speed; nearest forward/reverse heading
    error contributes heading_gain*error to requested yaw rate. This is an
    approximation, NOT an exact realization of body lateral velocity. A
    fixed lateral body-frame request produces a curve rather than pure side slip.
    Downstream tricycle kinematics enforces speed/steering limits.
    """
    params.validate()
    if not all(math.isfinite(v) for v in (vx, vy, yaw_rate)):
        return Adapted(reason='nonfinite_velocity')
    if math.hypot(vx, vy) <= params.zero_tolerance:
        if abs(yaw_rate) > params.zero_tolerance:
            return Adapted(reason='pure_rotation_unreachable')
        return Adapted(accepted=True, reason='stop')
    if vx < -params.zero_tolerance and not params.allow_reverse:
        return Adapted(reason='reverse_disabled')
    if abs(vy) <= params.zero_tolerance:
        return Adapted(vx=vx, yaw_rate=yaw_rate, accepted=True,
                       reason='differential_passthrough')
    direction = -1.0 if vx < -params.zero_tolerance else 1.0
    heading = math.atan2(direction*vy, direction*vx)
    speed = direction*math.hypot(vx, vy)
    mapped_yaw = yaw_rate + params.heading_gain*heading
    if not math.isfinite(speed) or not math.isfinite(mapped_yaw):
        return Adapted(reason='numeric_overflow')
    return Adapted(speed, mapped_yaw, heading, True, True, 'holonomic_heading_approximation')
