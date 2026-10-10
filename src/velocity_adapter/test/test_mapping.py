#!/usr/bin/env python3
import math
import unittest
from velocity_adapter.mapping import Parameters, adapt


class MappingTests(unittest.TestCase):
    def setUp(self):
        self.p = Parameters()

    def test_differential_forward_reverse_passthrough(self):
        for v in (-.1, .1):
            for w in (-.05, 0, .05):
                out = adapt(v, 0, w, self.p)
                self.assertTrue(out.accepted)
                self.assertFalse(out.approximate)
                self.assertEqual((out.vx, out.yaw_rate), (v, w))

    def test_diagonal_is_speed_norm_plus_heading_yaw(self):
        out = adapt(.1, .1, .03, self.p)
        self.assertTrue(out.accepted)
        self.assertTrue(out.approximate)
        self.assertAlmostEqual(out.vx, math.sqrt(.02))
        self.assertAlmostEqual(out.heading_error, math.pi/4)
        self.assertAlmostEqual(out.yaw_rate, .03+.5*math.pi/4)

    def test_lateral_becomes_travelling_turn_not_sideways(self):
        for y in (-.1, .1):
            out = adapt(0, y, 0, self.p)
            self.assertTrue(out.accepted)
            self.assertTrue(out.approximate)
            self.assertEqual(out.vx, .1)
            self.assertAlmostEqual(out.yaw_rate, math.copysign(.5*math.pi/2, y))

    def test_reverse_heading_is_nearest_reverse_axis(self):
        out = adapt(-.1, .1, 0, self.p)
        self.assertLess(out.vx, 0)
        self.assertAlmostEqual(out.heading_error, -math.pi/4)
        self.assertLess(out.yaw_rate, 0)

    def test_gain_and_yaw_feedforward(self):
        out = adapt(.1, .1, -.1, Parameters(heading_gain=.2))
        self.assertAlmostEqual(out.yaw_rate, -.1+.2*math.pi/4)

    def test_pure_rotation_is_not_silently_turned_into_forward_motion(self):
        out = adapt(0, 0, .1, self.p)
        self.assertFalse(out.accepted)
        self.assertEqual(out.reason, 'pure_rotation_unreachable')
        self.assertEqual((out.vx, out.yaw_rate), (0, 0))

    def test_zero_request_and_tiny_lateral(self):
        self.assertEqual(adapt(0, 0, 0, self.p).reason, 'stop')
        self.assertFalse(adapt(.1, 1e-8, .05, self.p).approximate)

    def test_reverse_can_be_disabled(self):
        out = adapt(-.1, .1, 0, Parameters(allow_reverse=False))
        self.assertFalse(out.accepted)
        self.assertEqual(out.reason, 'reverse_disabled')

    def test_nonfinite_and_numeric_overflow_stop(self):
        for values in ((math.nan, 0, 0), (.1, math.inf, 0), (.1, 0, math.nan),
                       (1.7e308, 1.7e308, 0)):
            out = adapt(*values, self.p)
            self.assertFalse(out.accepted)
            self.assertEqual(out.vx, 0)

    def test_invalid_gain_rejected(self):
        for p in (Parameters(heading_gain=0), Parameters(heading_gain=math.nan),
                  Parameters(zero_tolerance=0)):
            with self.assertRaises(ValueError):
                adapt(.1, .1, 0, p)


if __name__ == '__main__':
    unittest.main()
