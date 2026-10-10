#!/usr/bin/env python3
import math
import unittest
from dataclasses import replace
from tricycle_controller.kinematics import Limits, convert


class KinematicsTests(unittest.TestCase):
    def setUp(self):
        self.limits = Limits()

    def test_forward_and_reverse_straight(self):
        for v in (-.1, .1):
            cmd = convert(v, 0, 0, self.limits)
            self.assertTrue(cmd.accepted)
            self.assertFalse(cmd.saturated)
            self.assertEqual((cmd.speed, cmd.steering_angle, cmd.yaw_rate), (v, 0, 0))

    def test_signed_inverse_and_forward_kinematics(self):
        # Both reverse/forward, left/right: w sign is the body yaw, not wheel steer.
        for v in (-.1, .1):
            for w in (-.08, .08):
                cmd = convert(v, 0, w, self.limits)
                self.assertTrue(cmd.accepted)
                self.assertFalse(cmd.saturated)
                self.assertAlmostEqual(cmd.steering_angle, math.atan(.839*w/v))
                self.assertAlmostEqual(cmd.speed*math.tan(cmd.steering_angle)/.839, w)
                self.assertAlmostEqual(cmd.yaw_rate, w)

    def test_speed_limit_preserves_curvature(self):
        cmd = convert(1, 0, .5, self.limits)
        self.assertTrue(cmd.saturated)
        self.assertAlmostEqual(cmd.speed, .15)
        self.assertAlmostEqual(cmd.yaw_rate, .075)
        self.assertAlmostEqual(cmd.steering_angle, math.atan(.839*.5))

    def test_steering_limit_reports_achievable_yaw(self):
        for v in (-.1, .1):
            for w in (-2, 2):
                cmd = convert(v, 0, w, self.limits)
                self.assertTrue(cmd.saturated)
                self.assertAlmostEqual(abs(cmd.steering_angle), math.radians(50))
                self.assertAlmostEqual(abs(cmd.yaw_rate), .1*math.tan(math.radians(50))/.839)
                self.assertEqual(math.copysign(1, cmd.yaw_rate), math.copysign(1, w))

    def test_rejects_independent_lateral_and_in_place(self):
        for vx, vy, w, reason in ((.1, .02, 0, 'lateral_velocity_not_supported'),
                                  (0, 0, .1, 'in_place_rotation_not_supported'),
                                  (1e-8, 0, -.1, 'in_place_rotation_not_supported')):
            cmd = convert(vx, vy, w, self.limits)
            self.assertFalse(cmd.accepted)
            self.assertEqual(cmd.reason, reason)
            self.assertEqual((cmd.speed, cmd.steering_angle, cmd.yaw_rate), (0, 0, 0))

    def test_stop_and_tolerance(self):
        cmd = convert(0, 0, 0, self.limits)
        self.assertTrue(cmd.accepted)
        self.assertEqual(cmd.reason, 'stop')
        cmd = convert(.1, 1e-8, 0, self.limits)
        self.assertTrue(cmd.accepted)

    def test_optional_reverse_disable(self):
        cmd = convert(-.1, 0, 0, replace(self.limits, max_reverse_speed=0))
        self.assertFalse(cmd.accepted)
        self.assertEqual(cmd.reason, 'reverse_disabled')

    def test_nonfinite_input_stops(self):
        for values in ((math.nan, 0, 0), (.1, math.inf, 0), (.1, 0, -math.inf)):
            cmd = convert(*values, self.limits)
            self.assertFalse(cmd.accepted)
            self.assertEqual(cmd.speed, 0)

    def test_configuration_validation(self):
        for limits in (Limits(wheelbase=0), Limits(max_steering=math.pi/2),
                       Limits(max_forward_speed=-1), Limits(lateral_tolerance=-1),
                       Limits(wheelbase=math.nan), Limits(max_reverse_speed=-1)):
            with self.assertRaises(ValueError):
                convert(.1, 0, 0, limits)

    def test_small_nonzero_speed_is_numerically_bounded(self):
        cmd = convert(.00001, 0, 1, self.limits)
        self.assertTrue(cmd.accepted)
        self.assertTrue(cmd.saturated)
        self.assertLess(abs(cmd.yaw_rate), .00002)
        self.assertLessEqual(abs(cmd.steering_angle), self.limits.max_steering)


if __name__ == '__main__':
    unittest.main()
