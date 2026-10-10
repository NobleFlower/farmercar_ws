#!/usr/bin/env python3
"""Run with workspace sourced; never connects to a ROS master or hardware."""
import importlib.util
import math
from pathlib import Path
import struct
import unittest

path = Path(__file__).resolve().parent / 'vehicle_controller/scripts/servo_preview_adapter.py'
spec = importlib.util.spec_from_file_location('adapter', path)
adapter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(adapter)


class ProtocolTests(unittest.TestCase):
    def encode(self, v, w, **kw):
        args = dict(wheelbase=.839, max_angle_deg=50, neutral_deg=90, sign=1, speed_units=1)
        args.update(kw)
        return adapter.encode_preview(v, w, **args)

    def test_straight_packet_matches_existing_protocol(self):
        packet, angle, degrees = self.encode(.15, 0)
        self.assertEqual(packet[:3], [0xEE, 1, 4])
        self.assertEqual(struct.unpack('<hH', bytes(packet[3:7])), (15, 900))
        self.assertEqual(packet[-1], sum(packet[:-1]) & 255)
        self.assertEqual(angle, 0)

    def test_turning_kinematics_and_sign(self):
        for angle in (-50, -20, 20, 50):
            w = .15 * math.tan(math.radians(angle)) / .839
            packet, decoded_angle, protocol = self.encode(.15, w)
            self.assertAlmostEqual(decoded_angle, angle)
            self.assertAlmostEqual(protocol, 90 + angle)
        _, angle, protocol = self.encode(.15, .15 * math.tan(math.radians(20))/.839, sign=-1)
        self.assertAlmostEqual(protocol, 70)

    def test_stop_packet_is_zero_speed_centered(self):
        packet, _, _ = self.encode(0, 0)
        self.assertEqual(struct.unpack('<hH', bytes(packet[3:7])), (0, 900))

    def test_unachievable_invalid_commands_rejected(self):
        for v, w in ((0, .2), (-.1, 0), (.1, 1), (float('nan'), 0), (.1, float('inf'))):
            with self.assertRaises(ValueError):
                self.encode(v, w)


if __name__ == '__main__':
    unittest.main()
