"""Landmark-only pen-state toggle for live MonoTeach recording."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import dist

from .hand_observation import HandObservation


WRIST = 0
THUMB_TIP = 4
MIDDLE_FINGER_MCP = 9
MIDDLE_FINGER_TIP = 12


class PenState(str, Enum):
    """Task-level pen contact semantics, independent from camera tracking."""

    UP = "UP"
    DOWN = "DOWN"


@dataclass(frozen=True)
class PinchToggleConfig:
    """Scale-normalized hysteresis and debounce parameters for one pinch."""

    press_threshold: float = 0.35
    release_threshold: float = 0.55
    stable_frame_count: int = 3
    minimum_palm_scale: float = 1e-6

    def __post_init__(self) -> None:
        if not 0.0 < self.press_threshold < self.release_threshold:
            raise ValueError("press_threshold must be positive and below release_threshold.")
        if self.stable_frame_count < 1:
            raise ValueError("stable_frame_count must be at least one.")
        if self.minimum_palm_scale <= 0.0:
            raise ValueError("minimum_palm_scale must be positive.")


@dataclass(frozen=True)
class PenTransition:
    """One debounced pinch event and its resulting semantic pen state."""

    previous_state: PenState
    pen_state: PenState
    stroke_id: int | None

    @property
    def begins_stroke(self) -> bool:
        return self.previous_state is PenState.UP and self.pen_state is PenState.DOWN

    @property
    def ends_stroke(self) -> bool:
        return self.previous_state is PenState.DOWN and self.pen_state is PenState.UP


def normalized_pinch_distance(
    observation: HandObservation,
    config: PinchToggleConfig = PinchToggleConfig(),
) -> float | None:
    """Return thumb-tip/middle-tip distance divided by a palm-scale baseline."""
    if not observation.detected or len(observation.landmarks_norm) <= MIDDLE_FINGER_TIP:
        return None

    landmarks = observation.landmarks_norm
    palm_scale = dist(landmarks[WRIST], landmarks[MIDDLE_FINGER_MCP])
    if palm_scale < config.minimum_palm_scale:
        return None
    return dist(landmarks[THUMB_TIP], landmarks[MIDDLE_FINGER_TIP]) / palm_scale


class PinchToggleDetector:
    """Emit exactly one event per stable press-release pinch cycle."""

    def __init__(self, config: PinchToggleConfig = PinchToggleConfig()) -> None:
        self.config = config
        self._latched = False
        self._press_frames = 0
        self._release_frames = 0
        self.last_normalized_distance: float | None = None

    def update(self, observation: HandObservation) -> bool:
        """Return True only when a new stable pinch press is completed."""
        distance = normalized_pinch_distance(observation, self.config)
        self.last_normalized_distance = distance
        if distance is None:
            self._press_frames = 0
            self._release_frames = 0
            return False

        if self._latched:
            self._release_frames = (
                self._release_frames + 1
                if distance >= self.config.release_threshold
                else 0
            )
            if self._release_frames >= self.config.stable_frame_count:
                self._latched = False
                self._release_frames = 0
            return False

        self._press_frames = (
            self._press_frames + 1
            if distance <= self.config.press_threshold
            else 0
        )
        if self._press_frames < self.config.stable_frame_count:
            return False

        self._latched = True
        self._press_frames = 0
        return True


class PenStateController:
    """Map pinch events to UP/DOWN and monotonically numbered semantic strokes."""

    def __init__(self) -> None:
        self._state = PenState.UP
        self._active_stroke_id: int | None = None
        self._last_stroke_id = 0

    @property
    def state(self) -> PenState:
        return self._state

    @property
    def active_stroke_id(self) -> int | None:
        return self._active_stroke_id

    @property
    def display_stroke_id(self) -> int:
        return self._active_stroke_id or self._last_stroke_id

    def toggle(self) -> PenTransition:
        """Apply one valid pinch event; UP starts and DOWN ends a stroke."""
        previous = self._state
        if self._state is PenState.UP:
            self._last_stroke_id += 1
            self._active_stroke_id = self._last_stroke_id
            self._state = PenState.DOWN
        else:
            self._active_stroke_id = None
            self._state = PenState.UP
        return PenTransition(previous, self._state, self._active_stroke_id)
