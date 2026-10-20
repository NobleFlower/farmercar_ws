#!/usr/bin/env python3
import math
import unittest

from apriltag_navigation.voice_navigation import (
    Detection,
    NavigationParameters,
    VoiceCommand,
    VoiceFrameParser,
    VoiceNavigationStateMachine,
    lane_from_tag_name,
)


TAGS = {
    "road11", "road12", "road13", "road14",
    "road21", "road22", "road23", "road24",
}


def tag(name, z=2.0, x=0.0):
    return Detection(0, name, x, 0.0, z, 0.0)


class ParserTests(unittest.TestCase):
    def test_fragmented_and_coalesced_frames(self):
        parser = VoiceFrameParser()
        self.assertEqual(parser.feed(b"\xab\xab"), [])
        commands = parser.feed(b"\xab\x01\xde\xde\xde\x02\xad\xad\xad\x02")
        self.assertEqual([item.kind for item in commands],
                         ["forward", "cruise", "go_aisle"])
        self.assertEqual(commands[-1].target_lane, 2)

    def test_resynchronises_after_noise(self):
        parser = VoiceFrameParser()
        commands = parser.feed(b"\x00\x01\xab\xab\xab\x00")
        self.assertEqual(commands[0].kind, "stop")
        self.assertGreater(parser.errors, 0)

    def test_lane_is_first_name_digit(self):
        self.assertEqual(lane_from_tag_name("road12"), 1)
        self.assertEqual(lane_from_tag_name("road23"), 2)
        self.assertEqual(lane_from_tag_name("apriltag_link"), 0)


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.params = NavigationParameters()
        self.nav = VoiceNavigationStateMachine(self.params, TAGS)

    def voice(self, kind, lane=0, now=1.0):
        self.nav.on_voice_command(VoiceCommand(kind, lane), now)

    def detect(self, *items, now=1.0):
        self.nav.on_detections(items, now)
        return self.nav.tick(now)

    def test_low_speed_forward_and_reverse_track_current_lane(self):
        self.voice("forward")
        command = self.detect(tag("road12", 2.0, -0.2))
        self.assertTrue(command.active)
        self.assertGreater(command.speed_mps, 0)
        self.assertGreater(command.steering_rad, 0)
        self.assertEqual(command.lane, 1)

        self.voice("reverse")
        command = self.detect(tag("road12", 2.0, 0.0))
        self.assertLess(command.speed_mps, 0)
        self.assertEqual(command.phase, "lane_reverse")

    def test_stop_and_low_speed_mode_cancel_motion(self):
        self.voice("forward")
        self.detect(tag("road12"))
        self.voice("stop")
        command = self.detect(tag("road12"))
        self.assertFalse(command.active)
        self.assertEqual(command.speed_mps, 0)
        self.voice("cruise")
        self.voice("low_speed")
        self.assertFalse(self.detect(tag("road12")).active)

    def test_lane_one_crosses_to_lane_two(self):
        self.voice("go_aisle", 2)
        self.detect(tag("road13", 2.0))
        command = self.detect(tag("road14", 1.1))
        self.assertEqual(command.phase, "go_terminal")
        command = self.detect(tag("road14", 1.0))
        self.assertEqual(command.phase, "cross_turn")
        self.assertAlmostEqual(command.steering_rad, math.radians(45))
        self.assertAlmostEqual(command.speed_mps, 1.0)
        command = self.detect(tag("road23", 4.0))
        self.assertEqual(command.phase, "approach_target")
        command = self.detect(tag("road23", 0.5))
        self.assertFalse(command.active)
        self.assertEqual(command.lane, 2)
        self.assertEqual(command.reason, "cross_target_distance_reached")

    def test_lane_two_crosses_to_lane_one(self):
        self.voice("go_aisle", 1)
        self.detect(tag("road22", 2.0))
        command = self.detect(tag("road21", 1.0))
        self.assertEqual(command.phase, "cross_turn")
        command = self.detect(tag("road12", 3.0))
        self.assertEqual(command.phase, "approach_target")
        command = self.detect(tag("road12", 0.5))
        self.assertFalse(command.active)
        self.assertEqual(command.lane, 1)

    def test_missing_cross_tag_does_not_move_blindly(self):
        nav = VoiceNavigationStateMachine(self.params, {"road12", "road21"})
        nav.on_voice_command(VoiceCommand("go_aisle", 2), 1.0)
        nav.on_detections((tag("road12", 2.0),), 1.0)
        command = nav.tick(1.1)
        self.assertFalse(command.active)
        self.assertIn("missing_terminal_tag_config_road14", command.reason)

    def test_turn_timeout_is_fail_safe(self):
        self.voice("go_aisle", 2)
        self.detect(tag("road14", 1.0))
        command = self.nav.tick(12.0)
        self.assertFalse(command.active)
        self.assertEqual(command.reason, "cross_turn_timeout")

    def test_cross_turn_continues_while_no_tag_is_visible(self):
        self.voice("go_aisle", 2)
        self.detect(tag("road14", 1.0))
        command = self.nav.tick(2.0)
        self.assertTrue(command.active)
        self.assertEqual(command.phase, "cross_turn")
        self.assertAlmostEqual(command.speed_mps, 1.0)
        self.assertAlmostEqual(command.steering_rad, math.radians(45.0))

    def test_repeated_voice_frame_does_not_restart_crossing(self):
        self.voice("go_aisle", 2, now=1.0)
        self.detect(tag("road14", 1.0), now=1.0)
        self.voice("go_aisle", 2, now=11.5)
        command = self.nav.tick(11.6)
        self.assertFalse(command.active)
        self.assertEqual(command.reason, "cross_turn_timeout")

    def test_cruise_continues_after_crossing(self):
        self.voice("cruise")
        self.detect(tag("road12", 2.0))
        command = self.detect(tag("road14", 1.0))
        self.assertEqual(command.phase, "cross_turn")
        self.detect(tag("road23", 2.0))
        command = self.detect(tag("road23", 0.5))
        self.assertEqual(command.phase, "go_terminal")
        self.assertEqual(self.nav.target_lane, 1)
        self.assertEqual(command.lane, 2)

    def test_stale_detection_stops(self):
        self.voice("forward", now=1.0)
        self.detect(tag("road12"), now=1.0)
        command = self.nav.tick(2.0)
        self.assertFalse(command.active)
        self.assertEqual(command.reason, "tag_timeout")


if __name__ == "__main__":
    unittest.main()
