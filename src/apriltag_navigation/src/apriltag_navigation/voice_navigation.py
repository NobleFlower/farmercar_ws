"""ROS-independent voice command parsing and lane-navigation state machine."""
import math
import re
from dataclasses import dataclass


LANE_TAG_RE = re.compile(r"^road([0-9]+)[0-9]+$")


@dataclass(frozen=True)
class NavigationParameters:
    wheelbase: float = 0.839
    max_steering_deg: float = 50.0
    low_speed_forward_mps: float = 0.3
    low_speed_reverse_mps: float = 0.3
    cruise_speed_mps: float = 0.3
    approach_speed_mps: float = 0.3
    cross_turn_speed_mps: float = 1.0
    cross_turn_steering_deg: float = 45.0
    terminal_approach_distance_m: float = 1.0
    cross_target_stop_distance_m: float = 0.5
    target_pass_distance_m: float = 0.5
    max_tracking_distance_m: float = 12.0
    tag_timeout_s: float = 0.5
    turn_timeout_s: float = 10.0

    def validate(self):
        values = vars(self)
        if not all(math.isfinite(value) for value in values.values()):
            raise ValueError("navigation parameters must be finite")
        if (self.wheelbase <= 0 or not 0 < self.max_steering_deg < 90
                or self.low_speed_forward_mps <= 0
                or self.low_speed_reverse_mps <= 0
                or self.cruise_speed_mps <= 0
                or self.approach_speed_mps <= 0
                or self.cross_turn_speed_mps <= 0
                or not 0 < self.cross_turn_steering_deg
                or self.cross_turn_steering_deg > self.max_steering_deg
                or self.terminal_approach_distance_m <= 0
                or self.cross_target_stop_distance_m <= 0
                or self.target_pass_distance_m < 0
                or self.max_tracking_distance_m <= self.target_pass_distance_m
                or self.tag_timeout_s <= 0
                or self.turn_timeout_s <= 0):
            raise ValueError("invalid navigation parameters")


@dataclass(frozen=True)
class Detection:
    tag_id: int
    name: str
    x: float
    y: float
    z: float
    stamp_s: float


@dataclass(frozen=True)
class VoiceCommand:
    kind: str
    target_lane: int = 0
    raw: bytes = b""


@dataclass(frozen=True)
class MotionCommand:
    speed_mps: float = 0.0
    steering_rad: float = 0.0
    yaw_rate_radps: float = 0.0
    reason: str = "waiting_for_command"
    active: bool = False
    lane: int = 0
    phase: str = "idle"
    target_name: str = ""


class VoiceFrameParser:
    """Resynchronising parser for the existing fixed four-byte voice frames."""
    HEADERS = {
        (0xAB, 0xAB, 0xAB): "motion",
        (0xCD, 0xCD, 0xCD): "service",
        (0xDE, 0xDE, 0xDE): "mode",
        (0xAD, 0xAD, 0xAD): "aisle",
    }

    def __init__(self):
        self.buffer = bytearray()
        self.errors = 0

    def feed(self, data):
        self.buffer.extend(data)
        commands = []
        while len(self.buffer) >= 4:
            prefix = tuple(self.buffer[:3])
            if prefix not in self.HEADERS:
                del self.buffer[0]
                self.errors += 1
                continue
            raw = bytes(self.buffer[:4])
            command = self._decode(raw)
            commands.append(command)
            del self.buffer[:4]
        return commands

    @staticmethod
    def _decode(raw):
        prefix = tuple(raw[:3])
        code = raw[3]
        kinds = {
            ((0xAB, 0xAB, 0xAB), 0x01): "forward",
            ((0xAB, 0xAB, 0xAB), 0x10): "reverse",
            ((0xAB, 0xAB, 0xAB), 0x00): "stop",
            ((0xDE, 0xDE, 0xDE), 0x00): "low_speed",
            ((0xDE, 0xDE, 0xDE), 0x01): "intermittent",
            ((0xDE, 0xDE, 0xDE), 0x02): "cruise",
        }
        kind = kinds.get((prefix, code), "unknown")
        if prefix == (0xAD, 0xAD, 0xAD) and code in (1, 2):
            kind = "go_aisle"
        return VoiceCommand(kind, code if kind == "go_aisle" else 0, raw)


def lane_from_tag_name(name):
    match = LANE_TAG_RE.match(name)
    if not match:
        return 0
    return int(match.group(1))


class VoiceNavigationStateMachine:
    """Implements low-speed tracking, cross-lane commands and serpentine cruise."""

    def __init__(self, parameters, tag_names):
        parameters.validate()
        self.params = parameters
        self.tag_names = set(tag_names)
        self.mode = "low_speed"
        self.phase = "idle"
        self.lane = 0
        self.target_lane = 0
        self.target_name = ""
        self.phase_started = 0.0
        self.last_detection_update = 0.0
        self.detections = ()
        self.last_reason = "waiting_for_command"
        self.parser_errors = 0

    def on_detections(self, detections, now):
        self.detections = tuple(detections)
        self.last_detection_update = now
        if self.lane == 0:
            self._infer_lane()

    def on_voice_command(self, command, now):
        if command.kind == "stop":
            self.phase = "idle"
            self.target_name = ""
            self.target_lane = 0
            self.lane = 0
            self.mode = "low_speed"
            self.last_reason = "voice_stop"
        elif command.kind in ("low_speed", "intermittent"):
            self.mode = "low_speed"
            self.phase = "idle"
            self.target_name = ""
            self.target_lane = 0
            self.lane = 0
            self.last_reason = "low_speed_mode"
        elif command.kind == "forward":
            if self.mode == "low_speed" and self.phase == "lane_forward":
                self.last_reason = "voice_forward"
                return
            self.mode = "low_speed"
            self.phase = "lane_forward"
            self.target_name = ""
            self.target_lane = 0
            self.phase_started = now
            self.last_reason = "voice_forward"
        elif command.kind == "reverse":
            if self.mode == "low_speed" and self.phase == "lane_reverse":
                self.last_reason = "voice_reverse"
                return
            self.mode = "low_speed"
            self.phase = "lane_reverse"
            self.target_name = ""
            self.target_lane = 0
            self.phase_started = now
            self.last_reason = "voice_reverse"
        elif command.kind == "cruise":
            if self.mode == "cruise" and self.phase != "idle":
                self.last_reason = "cruise_requested"
                return
            self.mode = "cruise"
            self.phase = "go_terminal"
            self.target_lane = 0
            self.target_name = ""
            self.phase_started = now
            self.last_reason = "cruise_requested"
        elif command.kind == "go_aisle":
            if (self.mode == "aisle" and command.target_lane == self.target_lane
                    and self.phase != "idle"):
                self.last_reason = "go_aisle_requested"
                return
            self.mode = "aisle"
            self.target_lane = command.target_lane
            self.phase = "go_terminal" if self.lane else "waiting_for_lane"
            self.target_name = ""
            self.phase_started = now
            self.last_reason = "go_aisle_requested"
        else:
            self.phase = "idle"
            self.target_name = ""
            self.target_lane = 0
            self.last_reason = "unknown_voice_command"

    def tick(self, now):
        if self.phase == "idle":
            return self._stop(self.last_reason)
        if self.phase == "cross_turn":
            if now - self.phase_started > self.params.turn_timeout_s:
                self.phase = "idle"
                self.last_reason = "cross_turn_timeout"
                return self._stop(self.last_reason)
            # Crossing can temporarily leave every tag outside the field of
            # view. Only the bounded turn timeout applies in this phase.
            return self._turn_across_lane(now)
        if not self._detections_fresh(now):
            return self._stop("tag_timeout")
        if self.lane == 0:
            self._infer_lane()
        if self.phase == "waiting_for_lane":
            if self.lane:
                self._begin_cross_lane_or_cruise(now)
            if self.phase != "go_terminal":
                return self._stop(self.last_reason)
        if self.lane == 0:
            return self._stop("waiting_for_lane_tag")
        if self.phase in ("lane_forward", "lane_reverse"):
            return self._track_lane(now)
        if self.phase == "go_terminal":
            return self._navigate_terminal(now)
        if self.phase == "approach_target":
            return self._approach_target(now)
        return self._stop("invalid_navigation_phase")

    def _infer_lane(self):
        candidates = [item for item in self.detections
                      if lane_from_tag_name(item.name) > 0]
        if candidates:
            self.lane = lane_from_tag_name(
                min(candidates, key=lambda item: item.z).name)

    def _detections_fresh(self, now):
        return (self.detections and
                now - self.last_detection_update <= self.params.tag_timeout_s)

    def _normal_detections(self):
        result = []
        for item in self.detections:
            if (item.name in self.tag_names and math.isfinite(item.x)
                    and math.isfinite(item.y) and math.isfinite(item.z)
                    and item.z > self.params.target_pass_distance_m
                    and item.z <= self.params.max_tracking_distance_m):
                result.append(item)
        return result

    def _nearest_name(self, name):
        candidates = [item for item in self.detections
                      if item.name == name and math.isfinite(item.z) and item.z > 0]
        return min(candidates, key=lambda item: item.z) if candidates else None

    def _nearest_lane_tag(self):
        candidates = [item for item in self._normal_detections()
                      if lane_from_tag_name(item.name) == self.lane]
        return min(candidates, key=lambda item: item.z) if candidates else None

    def _track_lane(self, now):
        target = self._nearest_lane_tag()
        if target is None:
            return self._stop("no_visible_lane_tag")
        self.target_name = target.name
        speed = (self.params.low_speed_forward_mps
                 if self.phase == "lane_forward"
                 else -self.params.low_speed_reverse_mps)
        return self._track(target, speed, "tracking_lane_forward"
                           if speed > 0 else "tracking_lane_reverse")

    def _begin_cross_lane_or_cruise(self, now):
        if self.mode == "aisle":
            if self.target_lane == self.lane:
                self.phase = "idle"
                self.last_reason = "already_in_target_lane"
                return
            if self.target_lane not in (1, 2):
                self.phase = "idle"
                self.last_reason = "unsupported_target_lane"
                return
        self.phase = "go_terminal"
        self.phase_started = now
        self.target_name = ""
        self.last_reason = "crossing_to_lane_%d" % self.target_lane if self.mode == "aisle" else "cruising"

    @staticmethod
    def _terminal_for_lane(lane):
        return "road14" if lane == 1 else "road21" if lane == 2 else ""

    @staticmethod
    def _bridge_tag_for_crossing(source_lane):
        # Both specified lane changes are left turns at opposite lane ends.
        return "road23" if source_lane == 1 else "road12" if source_lane == 2 else ""

    def _navigate_terminal(self, now):
        if self.mode == "cruise":
            self.target_lane = 2 if self.lane == 1 else 1 if self.lane == 2 else 0
        if self.mode == "aisle" and self.lane == self.target_lane:
            self.phase = "idle"
            self.last_reason = "already_in_target_lane"
            return self._stop(self.last_reason)
        terminal_name = self._terminal_for_lane(self.lane)
        if not terminal_name:
            return self._stop("unsupported_source_lane")
        if terminal_name not in self.tag_names:
            return self._stop("missing_terminal_tag_config_%s" % terminal_name)
        terminal = self._nearest_name(terminal_name)
        if terminal and terminal.z <= self.params.terminal_approach_distance_m:
            self.phase = "cross_turn"
            self.phase_started = now
            self.target_name = self._bridge_tag_for_crossing(self.lane)
            self.last_reason = "terminal_reached_start_left_turn"
            return self._fixed_turn(self.last_reason)
        target = terminal or self._nearest_lane_tag()
        if target is None:
            return self._stop("waiting_for_lane_navigation_tag")
        self.target_name = target.name
        return self._track(target, self.params.cruise_speed_mps, "driving_to_terminal")

    def _turn_across_lane(self, now):
        bridge_name = self._bridge_tag_for_crossing(self.lane)
        if not bridge_name:
            self.phase = "idle"
            self.last_reason = "unsupported_source_lane"
            return self._stop(self.last_reason)
        if bridge_name not in self.tag_names:
            self.phase = "idle"
            self.last_reason = "missing_cross_target_tag_config_%s" % bridge_name
            return self._stop(self.last_reason)
        if self._detections_fresh(now) and self._nearest_name(bridge_name):
            self.phase = "approach_target"
            self.target_name = bridge_name
            self.last_reason = "cross_target_detected"
            return self._approach_target(now)
        self.target_name = bridge_name
        return self._fixed_turn("turning_left_until_cross_target")

    def _approach_target(self, now):
        target = self._nearest_name(self.target_name)
        if target is None:
            return self._stop("cross_target_lost")
        if target.z <= self.params.cross_target_stop_distance_m:
            self.lane = self.target_lane
            self.phase = "idle"
            self.last_reason = "cross_target_distance_reached"
            if self.mode == "cruise":
                # Continue the serpentine route automatically.
                self.target_lane = 2 if self.lane == 1 else 1 if self.lane == 2 else 0
                self.phase = "go_terminal"
                self.phase_started = now
                self.target_name = ""
                self.last_reason = "cruise_continuing_after_cross"
                return self._stop(self.last_reason)
            return self._stop(self.last_reason)
        return self._track(target, self.params.approach_speed_mps, "approaching_cross_target")

    def _track(self, target, speed, reason):
        radius_squared = target.x * target.x + target.z * target.z
        if radius_squared <= 1e-9:
            return self._stop("invalid_tag_geometry")
        # Camera optical x is right-positive; vehicle-left is -x.
        lateral = -target.x
        curvature = 2.0 * lateral / radius_squared
        steering = math.atan(self.params.wheelbase * curvature)
        max_steering = math.radians(self.params.max_steering_deg)
        steering = max(-max_steering, min(max_steering, steering))
        yaw_rate = speed * math.tan(steering) / self.params.wheelbase
        return MotionCommand(speed, steering, yaw_rate, reason, True,
                             self.lane, self.phase, self.target_name)

    def _fixed_turn(self, reason):
        steering = math.radians(self.params.cross_turn_steering_deg)
        speed = self.params.cross_turn_speed_mps
        yaw_rate = speed * math.tan(steering) / self.params.wheelbase
        return MotionCommand(speed, steering, yaw_rate, reason, True,
                             self.lane, self.phase, self.target_name)

    def _stop(self, reason):
        return MotionCommand(reason=reason, active=False, lane=self.lane,
                             phase=self.phase, target_name=self.target_name)
