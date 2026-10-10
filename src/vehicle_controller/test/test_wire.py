#!/usr/bin/env python3
import math
import os
import pty
import struct
import time
import unittest
from vehicle_controller.wire import Calibration, FeedbackParser, decode_feedback, encode_motion, packet
from vehicle_controller.transport import SerialTransport


class WireTests(unittest.TestCase):
    def setUp(self):
        self.cal = Calibration()

    def test_known_legacy_command_bytes(self):
        self.assertEqual(encode_motion(0, 0, self.cal).hex(' '), 'ee 01 04 00 00 84 03 7a')
        self.assertEqual(encode_motion(.1, math.radians(10), self.cal).hex(' '), 'ee 01 04 0a 00 e8 03 e8')
        frame = encode_motion(-.1, math.radians(-10), self.cal)
        self.assertEqual(struct.unpack('<hH', frame[3:7]), (-10, 800))

    def test_extended_motion_flags_and_sequence(self):
        frame = encode_motion(.1, 0, self.cal, 'extended', True, 65535, True)
        self.assertEqual(frame[:3], bytes([0xEE, 2, 7]))
        self.assertEqual(struct.unpack('<hHBH', frame[3:-1]), (10, 900, 5, 65535))
        stop = encode_motion(0, 0, self.cal, 'extended', False, 65536, True)
        self.assertEqual(struct.unpack('<hHBH', stop[3:-1]), (0, 900, 6, 0))

    def test_feedback_measurement_semantics(self):
        legacy = packet(0xEF, 1, struct.pack('<hH', -10, 1000))
        state = decode_feedback(legacy, self.cal)
        self.assertAlmostEqual(state['speed'], -.1)
        self.assertAlmostEqual(state['steering_angle'], math.radians(10))
        self.assertFalse(state['speed_is_measured'])
        self.assertFalse(state['steering_is_measured'])
        self.assertFalse(state['status_authoritative'])
        extended = packet(0xEF, 2, struct.pack('<hHBBHH', 12, 800, 2, 13, 0, 17))
        state = decode_feedback(extended, self.cal)
        self.assertTrue(state['controller_ready'])
        self.assertFalse(state['emergency_stop'])
        self.assertTrue(state['speed_is_measured'])
        self.assertTrue(state['steering_is_measured'])
        self.assertEqual(state['ack_sequence'], 17)

    def test_calibration_direction_and_speed_scale(self):
        cal = Calibration(speed_units_per_mps=2, steering_sign=-1)
        frame = encode_motion(.1, math.radians(10), cal)
        self.assertEqual(struct.unpack('<hH', frame[3:7]), (20, 800))
        rx = packet(0xEF, 1, frame[3:7])
        state = decode_feedback(rx, cal)
        self.assertAlmostEqual(state['speed'], .1)
        self.assertAlmostEqual(state['steering_angle'], math.radians(10))

    def test_stream_fragmentation_corruption_and_resync(self):
        first = packet(0xEF, 1, struct.pack('<hH', 0, 900))
        second = packet(0xEF, 2, struct.pack('<hHBBHH', 0, 900, 2, 13, 0, 7))
        parser = FeedbackParser()
        self.assertEqual(parser.feed(b'garbage'+first[:2]), [])
        self.assertEqual(parser.feed(first[2:5]), [])
        self.assertEqual(parser.feed(first[5:]+second), [first, second])
        bad = bytearray(first)
        bad[-1] ^= 1
        self.assertEqual(parser.feed(bytes(bad)+b'\xef\xff\xff'+second), [second])
        self.assertGreater(parser.errors, 0)
        self.assertLessEqual(len(parser.buffer), 14)

    def test_invalid_range_status_checksum_rejected(self):
        frames = (packet(0xEF, 1, struct.pack('<hH', 0, 1800)),
                  packet(0xEF, 2, struct.pack('<hHBBHH', 0, 900, 8, 13, 0, 0)),
                  packet(0xEF, 2, struct.pack('<hHBBHH', 0, 900, 2, 255, 0, 0)),
                  b'\xef\x01\x04\x00\x00\x84\x03\x00')
        for frame in frames:
            with self.assertRaises(ValueError):
                decode_feedback(frame, self.cal)
        for speed, steer in ((.3, 0), (0, math.radians(51)), (float('nan'), 0)):
            with self.assertRaises(ValueError):
                encode_motion(speed, steer, self.cal)

    def test_posix_serial_roundtrip_on_fake_pty_only(self):
        master, slave = pty.openpty()
        transport = SerialTransport(os.ttyname(slave), 115200)
        try:
            tx = encode_motion(.1, 0, self.cal)
            transport.write(tx)
            self.assertEqual(os.read(master, 64), tx)
            rx = packet(0xEF, 1, struct.pack('<hH', 0, 900))
            os.write(master, rx)
            deadline = time.monotonic()+1
            received = b''
            while time.monotonic()<deadline and received != rx:
                received += transport.read()
                time.sleep(.005)
            self.assertEqual(received, rx)
        finally:
            transport.close()
            os.close(master)
            os.close(slave)


if __name__ == '__main__':
    unittest.main()
