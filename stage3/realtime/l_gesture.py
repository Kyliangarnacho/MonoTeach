"""Conservative POINT -> L -> POINT command from existing hand landmarks.

The former rule confused a naturally extended thumb with an L because it only
asked whether the thumb itself was straight and open.  This version measures
whether the thumb tip moves sideways away from the index-finger axis and only
confirms a command after a capture-time dwell.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from stage2.hand_observation import HandObservation, NormalizedLandmark

WRIST = 0
THUMB_TIP = 4
INDEX_MCP, INDEX_PIP, INDEX_DIP, INDEX_TIP = 5, 6, 7, 8
MIDDLE_MCP, MIDDLE_PIP, MIDDLE_DIP, MIDDLE_TIP = 9, 10, 11, 12
RING_MCP, RING_PIP, RING_DIP, RING_TIP = 13, 14, 15, 16
PINKY_MCP, PINKY_PIP, PINKY_DIP, PINKY_TIP = 17, 18, 19, 20


class HandControlGesture(str, Enum):
    POINT = "POINT"
    L_COMMAND = "L_COMMAND"
    OTHER = "OTHER"
    UNAVAILABLE = "UNAVAILABLE"


@dataclass(frozen=True)
class LGestureConfig:
    """Scale-normalized thresholds for an intentionally obvious L command."""
    finger_straight_angle_deg: float = 150.0
    thumb_axis_distance_enter_palm_scales: float = 1.15
    thumb_axis_distance_exit_palm_scales: float = 0.85
    command_dwell_ms: float = 400.0
    point_rearm_ms: float = 300.0
    dropout_grace_ms: float = 180.0
    minimum_palm_scale: float = 1e-5

    def __post_init__(self) -> None:
        angle = float(self.finger_straight_angle_deg)
        enter, exit_ = float(self.thumb_axis_distance_enter_palm_scales), float(self.thumb_axis_distance_exit_palm_scales)
        if not 90.0 < angle < 180.0:
            raise ValueError("finger_straight_angle_deg must lie between 90 and 180 degrees.")
        if not math.isfinite(enter) or not math.isfinite(exit_) or not 0.0 < exit_ < enter:
            raise ValueError("L axis-distance thresholds must satisfy 0 < exit < enter.")
        for name in ("command_dwell_ms", "point_rearm_ms", "dropout_grace_ms", "minimum_palm_scale"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive.")
            object.__setattr__(self, name, value)
        object.__setattr__(self, "finger_straight_angle_deg", angle)
        object.__setattr__(self, "thumb_axis_distance_enter_palm_scales", enter)
        object.__setattr__(self, "thumb_axis_distance_exit_palm_scales", exit_)


def _xy(point: NormalizedLandmark) -> tuple[float, float]:
    return float(point[0]), float(point[1])


def _distance(first: NormalizedLandmark, second: NormalizedLandmark) -> float:
    ax, ay = _xy(first); bx, by = _xy(second)
    return math.hypot(ax - bx, ay - by)


def _joint_angle_deg(first: NormalizedLandmark, joint: NormalizedLandmark, third: NormalizedLandmark) -> float:
    ax, ay = _xy(first); bx, by = _xy(joint); cx, cy = _xy(third)
    left, right = (ax - bx, ay - by), (cx - bx, cy - by)
    denominator = math.hypot(*left) * math.hypot(*right)
    if denominator <= 1e-12:
        return 0.0
    cosine = max(-1.0, min(1.0, (left[0] * right[0] + left[1] * right[1]) / denominator))
    return math.degrees(math.acos(cosine))


def _finger_extended(landmarks: tuple[NormalizedLandmark, ...], mcp: int, pip: int, dip: int, tip: int, angle: float) -> bool:
    return _joint_angle_deg(landmarks[mcp], landmarks[pip], landmarks[dip]) >= angle and _joint_angle_deg(landmarks[pip], landmarks[dip], landmarks[tip]) >= angle


def normalized_thumb_axis_distance(observation: HandObservation, config: LGestureConfig = LGestureConfig()) -> float | None:
    """Thumb-tip distance to the index MCP-to-tip line, divided by palm scale."""
    if not observation.detected or len(observation.landmarks_norm) <= PINKY_TIP:
        return None
    points = observation.landmarks_norm
    palm_scale = _distance(points[WRIST], points[MIDDLE_MCP])
    if palm_scale < config.minimum_palm_scale:
        return None
    mx, my = _xy(points[INDEX_MCP]); tx, ty = _xy(points[INDEX_TIP]); hx, hy = _xy(points[THUMB_TIP])
    axis_x, axis_y = tx - mx, ty - my
    axis_length = math.hypot(axis_x, axis_y)
    if axis_length <= 1e-12:
        return None
    perpendicular_distance = abs((hx - mx) * axis_y - (hy - my) * axis_x) / axis_length
    return perpendicular_distance / palm_scale


def _classify_with_axis_threshold(
    observation: HandObservation, config: LGestureConfig, axis_threshold: float
) -> HandControlGesture:
    """Classify the hand geometry using one specified thumb-distance threshold."""
    feature = normalized_thumb_axis_distance(observation, config)
    if feature is None:
        return HandControlGesture.UNAVAILABLE
    landmarks = observation.landmarks_norm
    index_extended = _finger_extended(landmarks, INDEX_MCP, INDEX_PIP, INDEX_DIP, INDEX_TIP, config.finger_straight_angle_deg)
    folded_others = not any(_finger_extended(landmarks, mcp, pip, dip, tip, config.finger_straight_angle_deg) for mcp, pip, dip, tip in ((MIDDLE_MCP, MIDDLE_PIP, MIDDLE_DIP, MIDDLE_TIP), (RING_MCP, RING_PIP, RING_DIP, RING_TIP), (PINKY_MCP, PINKY_PIP, PINKY_DIP, PINKY_TIP)))
    if not (index_extended and folded_others):
        return HandControlGesture.OTHER
    return HandControlGesture.L_COMMAND if feature >= axis_threshold else HandControlGesture.POINT


def classify_l_gesture(observation: HandObservation, config: LGestureConfig = LGestureConfig()) -> HandControlGesture:
    """Classify an instantaneous *entry candidate*; detector dwell confirms commands."""
    return _classify_with_axis_threshold(observation, config, config.thumb_axis_distance_enter_palm_scales)


class LGestureToggleDetector:
    """Confirm deliberate L only after real-time dwell; candidates are harmless."""
    def __init__(self, config: LGestureConfig = LGestureConfig()) -> None:
        if not isinstance(config, LGestureConfig):
            raise TypeError("config must be an LGestureConfig.")
        self.config = config
        self.last_candidate = HandControlGesture.UNAVAILABLE
        self.last_gesture = HandControlGesture.UNAVAILABLE
        self._candidate_since_ms: float | None = None
        self._candidate_paused_ms = 0.0
        self._dropout_since_ms: float | None = None
        self._last_timestamp_ms = 0.0
        self._point_since_ms: float | None = None
        self._armed = False
        self._command_active = False

    @property
    def armed(self) -> bool:
        return self._armed

    @property
    def command_active(self) -> bool:
        return self._command_active

    @property
    def candidate_active(self) -> bool:
        """True while an L is being confirmed or deliberately held.

        This is intentionally broader than ``command_active``.  During the
        400 ms confirmation interval we must not let the moving fingertip
        become an ordinary workspace sample or a one-frame outside barrier.
        """
        return self._command_active or self._candidate_since_ms is not None

    @property
    def candidate_elapsed_ms(self) -> float:
        if self._candidate_since_ms is None:
            return 0.0
        reference = self._dropout_since_ms if self._dropout_since_ms is not None else self._last_timestamp_ms
        return max(0.0, reference - self._candidate_since_ms - self._candidate_paused_ms)

    @property
    def dropout_grace_remaining_ms(self) -> float:
        if self._dropout_since_ms is None:
            return 0.0
        return max(0.0, self.config.dropout_grace_ms - (self._last_timestamp_ms - self._dropout_since_ms))

    def update(self, observation: HandObservation) -> bool:
        # Once an L is confirmed, use the lower exit threshold.  This is the
        # hysteresis band: landmark noise between 0.85 and 1.15 palm scales
        # cannot repeatedly turn the command on and off.
        threshold = (
            self.config.thumb_axis_distance_exit_palm_scales
            if self._command_active
            else self.config.thumb_axis_distance_enter_palm_scales
        )
        candidate = _classify_with_axis_threshold(observation, self.config, threshold)
        self.last_candidate = candidate
        now_ms = float(observation.timestamp_ms)
        self._last_timestamp_ms = now_ms
        if candidate is HandControlGesture.UNAVAILABLE:
            # A single missed landmark frame is common while forming an L.
            # It pauses candidate dwell rather than counting invisible time;
            # a longer loss still ends the gesture normally.
            if self.candidate_active:
                if self._dropout_since_ms is None:
                    self._dropout_since_ms = now_ms
                if now_ms - self._dropout_since_ms <= self.config.dropout_grace_ms:
                    self.last_gesture = HandControlGesture.L_COMMAND if self._command_active else HandControlGesture.POINT
                    return False
            self._reset_l_state()
            self.last_gesture = HandControlGesture.UNAVAILABLE
            self._point_since_ms = None
            return False
        if self._dropout_since_ms is not None:
            # Remove invisible time from the continuous-L dwell requirement.
            self._candidate_paused_ms += now_ms - self._dropout_since_ms
            self._dropout_since_ms = None
        if candidate is HandControlGesture.POINT:
            self._reset_l_state(); self.last_gesture = HandControlGesture.POINT
            if self._point_since_ms is None: self._point_since_ms = now_ms
            if now_ms - self._point_since_ms >= self.config.point_rearm_ms: self._armed = True
            return False
        self._point_since_ms = None
        if candidate is not HandControlGesture.L_COMMAND:
            self._reset_l_state(); self.last_gesture = candidate
            return False
        if self._command_active:
            # The same held command remains a hold, not a second toggle
            # candidate.  A POINT below the exit threshold releases it.
            self.last_gesture = HandControlGesture.L_COMMAND
            return False
        if self._candidate_since_ms is None:
            self._candidate_since_ms = now_ms; self._candidate_paused_ms = 0.0
        if self._armed and self.candidate_elapsed_ms >= self.config.command_dwell_ms:
            self._armed = False; self._command_active = True; self.last_gesture = HandControlGesture.L_COMMAND; self._candidate_since_ms = None; self._candidate_paused_ms = 0.0
            return True
        self.last_gesture = HandControlGesture.POINT
        return False

    def _reset_l_state(self) -> None:
        self._candidate_since_ms = None
        self._candidate_paused_ms = 0.0
        self._dropout_since_ms = None
        self._command_active = False
