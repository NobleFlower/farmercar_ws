#!/usr/bin/env python3
import math
import unittest
from apriltag_servo.control import Parameters, calculate


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.p = Parameters()

    def test_forward_straight_and_acceleration_bound(self):
        cmd = calculate(3.5, 0.0, 2.9, 0.0, 0.05, self.p)
        self.assertGreater(cmd.speed, 0)
        self.assertLessEqual(cmd.speed, self.p.acceleration * 0.05)
        self.assertEqual(cmd.yaw_rate, 0.0)
        self.assertEqual(cmd.steering, 0.0)

    def test_turn_sign_and_ackermann_relation(self):
        for y in (-0.8, 0.8):
            cmd = calculate(3.0, y, 2.9, 0.15, 0.05, self.p)
            self.assertEqual(math.copysign(1, cmd.steering), math.copysign(1, y))
            self.assertAlmostEqual(cmd.yaw_rate, cmd.speed * math.tan(cmd.steering) / 0.839)
            self.assertLessEqual(abs(cmd.steering), math.radians(50))

    def test_sharp_turn_saturates_steering(self):
        cmd = calculate(0.1, 0.1, 2.9, 0.15, 0.05, self.p)
        self.assertAlmostEqual(cmd.steering, math.radians(50))

    def test_no_reverse_or_in_place_for_close_behind_and_lateral_targets(self):
        for x, y, distance in ((3.0, 1.0, 0.5), (-2.0, 1.0, 2.9), (0.0, 3.0, 3.0)):
            cmd = calculate(x, y, distance, 0.15, 0.05, self.p)
            self.assertEqual((cmd.speed, cmd.steering, cmd.yaw_rate), (0, 0, 0))

    def test_distance_uses_raw_optical_z_without_offsets(self):
        cmd = calculate(1.04, 2.3, 1.04, 0.15, 0.05, self.p)
        self.assertEqual(cmd.speed, 0)
        self.assertEqual(cmd.reason, 'desired_distance_reached')

    def test_invalid_measurements_stop_immediately(self):
        for values in ((math.nan, 0, 3, .15, .05), (3, math.inf, 3, .15, .05),
                       (3, 0, 0, .15, .05), (3, 0, 3, .15, 0)):
            cmd = calculate(*values, self.p)
            self.assertEqual((cmd.speed, cmd.yaw_rate), (0, 0))

    def test_invalid_parameters_fail(self):
        for params in (Parameters(wheelbase=0), Parameters(max_steering=math.pi/2),
                       Parameters(max_speed=-1), Parameters(acceleration=math.nan)):
            with self.assertRaises(ValueError):
                params.validate()

    def test_delayed_tick_does_not_accelerate_unboundedly(self):
        cmd = calculate(3.0, 0.0, 2.9, 0.0, 20.0, self.p)
        self.assertLessEqual(cmd.speed, self.p.acceleration * 0.1)


if __name__ == '__main__':
    unittest.main()
