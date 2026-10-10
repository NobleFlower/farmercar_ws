"""Existing EE/EF v1 plus an explicit optional v2 firmware contract.

Checksum is byte sum modulo 256, NOT polynomial CRC. Little endian payloads.
"""
import math
import struct
from dataclasses import dataclass


@dataclass(frozen=True)
class Calibration:
    speed_units_per_mps: float = 1.0
    steering_neutral_deg: float = 90.0
    steering_sign: int = 1
    max_speed: float = .15
    max_steering_deg: float = 50.0

    def validate(self):
        if (not all(math.isfinite(v) for v in vars(self).values()) or
                self.speed_units_per_mps <= 0 or self.steering_sign not in (-1, 1) or
                self.max_speed <= 0 or not 0 < self.max_steering_deg < 90 or
                self.steering_neutral_deg-self.max_steering_deg < 0 or
                self.steering_neutral_deg+self.max_steering_deg > 180):
            raise ValueError('invalid protocol calibration')


def packet(header, kind, payload):
    if len(payload) > 64:
        raise ValueError('payload too long')
    data = bytes([header, kind, len(payload)])+payload
    return data+bytes([sum(data) & 255])


def encode_motion(speed, steering_rad, cal, version='legacy', enabled=False, sequence=0, auto_requested=False):
    cal.validate()
    if not math.isfinite(speed) or not math.isfinite(steering_rad):
        raise ValueError('nonfinite command')
    if abs(speed) > cal.max_speed+1e-6 or abs(math.degrees(steering_rad)) > cal.max_steering_deg+1e-6:
        raise ValueError('command exceeds configured chassis limits')
    raw_speed = round(speed*cal.speed_units_per_mps*100)
    raw_angle = round((cal.steering_neutral_deg+cal.steering_sign*math.degrees(steering_rad))*10)
    if not -32768 <= raw_speed <= 32767 or not 0 <= raw_angle <= 1800:
        raise ValueError('command outside wire range')
    payload = struct.pack('<hH', raw_speed, raw_angle)
    if version == 'legacy':
        return packet(0xEE, 1, payload)
    if version == 'extended':
        # bit0 permit drive, bit1 explicit stop, bit2 request automatic mode.
        flags = (1 if enabled else 0) | (2 if not enabled or speed == 0 else 0) | (4 if auto_requested else 0)
        return packet(0xEE, 2, payload+struct.pack('<BH', flags, sequence & 65535))
    raise ValueError('unknown protocol version')


def decode_feedback(frame, cal, legacy_speed_measured=False, legacy_steering_measured=False):
    cal.validate()
    if len(frame) < 4 or frame[0] != 0xEF or len(frame) != frame[2]+4 or frame[-1] != sum(frame[:-1]) & 255:
        raise ValueError('invalid feedback frame/checksum')
    if (frame[1], frame[2]) not in ((1, 4), (2, 10)):
        raise ValueError('unsupported feedback type/length')
    raw_speed, raw_angle = struct.unpack('<hH', frame[3:7])
    steer = (raw_angle/10-cal.steering_neutral_deg)/cal.steering_sign
    if not 0 <= raw_angle <= 1800 or abs(steer) > cal.max_steering_deg+.1:
        raise ValueError('invalid measured steering')
    result = dict(protocol_version=frame[1], speed=raw_speed/(100*cal.speed_units_per_mps),
                  steering_angle=math.radians(steer), speed_is_measured=legacy_speed_measured,
                  steering_is_measured=legacy_steering_measured, status_authoritative=False,
                  controller_ready=False, emergency_stop=False, control_mode=0,
                  fault_flags=0, ack_valid=False, ack_sequence=0)
    if frame[1] == 2:
        mode, flags, faults, ack = struct.unpack('<BBHH', frame[7:13])
        if mode not in (0, 1, 2) or flags & ~15:
            raise ValueError('invalid extended status')
        result.update(status_authoritative=True, controller_ready=bool(flags & 1),
                      emergency_stop=bool(flags & 2), speed_is_measured=bool(flags & 4),
                      steering_is_measured=bool(flags & 8), control_mode=mode,
                      fault_flags=faults, ack_valid=True, ack_sequence=ack)
    return result


class FeedbackParser:
    """Bounded stream parser; accepts fragmented/coalesced frames and resyncs."""
    def __init__(self):
        self.buffer = bytearray()
        self.errors = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while self.buffer:
            try:
                index = self.buffer.index(0xEF)
            except ValueError:
                self.buffer.clear()
                break
            del self.buffer[:index]
            if len(self.buffer) < 3:
                break
            kind, length = self.buffer[1], self.buffer[2]
            if (kind, length) not in ((1, 4), (2, 10)):
                del self.buffer[0]
                self.errors += 1
                continue
            total = length+4
            if len(self.buffer) < total:
                break
            frame = bytes(self.buffer[:total])
            if frame[-1] != sum(frame[:-1]) & 255:
                del self.buffer[0]
                self.errors += 1
                continue
            frames.append(frame)
            del self.buffer[:total]
        return frames
