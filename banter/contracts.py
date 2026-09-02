"""Immutable Task 1 contracts from hand perception to one-shot events.

``HandFrame`` preserves one camera frame's 0/1/2 raw hand observations.
``GestureEvidence`` is a repeatable per-frame classifier fact.  Only
``GestureEvent`` crosses the temporal confirmation boundary; downstream
Grammar must never need to infer a pose from raw landmarks again.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real

from stage2.hand_observation import HandObservation


EVENT_SCHEMA_VERSION = "banter_gesture_event_v1"


class HandKey(str, Enum):
    """Stable role usable by later two-hand grammar, not a tracker index."""

    LEFT = "LEFT"
    RIGHT = "RIGHT"
    UNKNOWN = "UNKNOWN"


class GestureKind(str, Enum):
    """Small, intentionally obvious Task 1 vocabulary."""

    POINT_ONE = "POINT_ONE"
    VICTORY_TWO = "VICTORY_TWO"
    OPEN_PALM_FIVE = "OPEN_PALM_FIVE"
    THUMBS_UP = "THUMBS_UP"
    FIST = "FIST"
    MIDDLE_FINGER = "MIDDLE_FINGER"
    UNKNOWN = "UNKNOWN"


class GestureSource(str, Enum):
    """How an evidence label was obtained; the initial implementation is canned."""

    MEDIAPIPE_CANNED = "MEDIAPIPE_CANNED"
    LANDMARK_RULE = "LANDMARK_RULE"


def _finite_nonnegative(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a finite non-negative number.")
    numeric = float(value)
    if not math.isfinite(numeric) or numeric < 0.0:
        raise ValueError(f"{name} must be a finite non-negative number.")
    return numeric


def _score(value: object, name: str, *, optional: bool = False) -> float | None:
    if value is None and optional:
        return None
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a finite score between zero and one.")
    numeric = float(value)
    if not math.isfinite(numeric) or not 0.0 <= numeric <= 1.0:
        raise ValueError(f"{name} must be a finite score between zero and one.")
    return numeric


def _nonnegative_int(value: object, name: str, *, minimum: int = 0) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < minimum:
        raise ValueError(f"{name} must be an integer at least {minimum}.")
    return value


@dataclass(frozen=True)
class HandFrame:
    """One camera frame with zero, one, or two detected hands.

    The tuple order is MediaPipe's current result order only.  It must not be
    used as a persistent identity; later components use ``HandKey`` instead.
    """

    frame_index: int
    capture_t_ms: float
    available_t_ms: float
    hands: tuple[HandObservation, ...]

    def __post_init__(self) -> None:
        _nonnegative_int(self.frame_index, "frame_index")
        capture_t_ms = _finite_nonnegative(self.capture_t_ms, "capture_t_ms")
        available_t_ms = _finite_nonnegative(self.available_t_ms, "available_t_ms")
        if available_t_ms < capture_t_ms:
            raise ValueError("available_t_ms must not precede capture_t_ms.")
        if not isinstance(self.hands, tuple) or len(self.hands) > 2:
            raise ValueError("hands must be a tuple containing zero to two observations.")
        for hand in self.hands:
            if not isinstance(hand, HandObservation) or not hand.detected:
                raise ValueError("HandFrame hands must contain detected HandObservation values.")
            if not math.isclose(float(hand.timestamp_ms), capture_t_ms, abs_tol=1e-6):
                raise ValueError("Each hand observation timestamp must match capture_t_ms.")
        object.__setattr__(self, "capture_t_ms", capture_t_ms)
        object.__setattr__(self, "available_t_ms", available_t_ms)


@dataclass(frozen=True)
class GestureEvidence:
    """One hand's instantaneous classifier output; it is not an event."""

    frame_index: int
    capture_t_ms: float
    available_t_ms: float
    hand: HandObservation
    hand_key: HandKey
    gesture: GestureKind
    recognizer_score: float | None
    source: GestureSource = GestureSource.MEDIAPIPE_CANNED
    rejection_reason: str | None = None

    def __post_init__(self) -> None:
        _nonnegative_int(self.frame_index, "frame_index")
        capture_t_ms = _finite_nonnegative(self.capture_t_ms, "capture_t_ms")
        available_t_ms = _finite_nonnegative(self.available_t_ms, "available_t_ms")
        if available_t_ms < capture_t_ms:
            raise ValueError("available_t_ms must not precede capture_t_ms.")
        if not isinstance(self.hand, HandObservation) or not self.hand.detected:
            raise ValueError("GestureEvidence requires one detected HandObservation.")
        if not math.isclose(float(self.hand.timestamp_ms), capture_t_ms, abs_tol=1e-6):
            raise ValueError("hand.timestamp_ms must match capture_t_ms.")
        if not isinstance(self.hand_key, HandKey):
            raise TypeError("hand_key must be a HandKey.")
        if not isinstance(self.gesture, GestureKind):
            raise TypeError("gesture must be a GestureKind.")
        if not isinstance(self.source, GestureSource):
            raise TypeError("source must be a GestureSource.")
        score = _score(self.recognizer_score, "recognizer_score", optional=True)
        if self.gesture is GestureKind.UNKNOWN and score is not None:
            raise ValueError("UNKNOWN evidence must not claim a recognizer score.")
        if self.gesture is not GestureKind.UNKNOWN and score is None:
            raise ValueError("Known gesture evidence requires recognizer_score.")
        if self.rejection_reason is not None and (not isinstance(self.rejection_reason, str) or not self.rejection_reason):
            raise TypeError("rejection_reason must be a non-empty string or None.")
        object.__setattr__(self, "capture_t_ms", capture_t_ms)
        object.__setattr__(self, "available_t_ms", available_t_ms)
        object.__setattr__(self, "recognizer_score", score)


@dataclass(frozen=True)
class GesturePerception:
    """Derived Task 1 output for one frame, retaining raw and classified truth."""

    frame: HandFrame
    evidence: tuple[GestureEvidence, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.frame, HandFrame):
            raise TypeError("frame must be a HandFrame.")
        if not isinstance(self.evidence, tuple) or len(self.evidence) != len(self.frame.hands):
            raise ValueError("evidence must contain exactly one item for each detected hand.")
        for item, hand in zip(self.evidence, self.frame.hands):
            if not isinstance(item, GestureEvidence) or item.hand != hand:
                raise ValueError("Each evidence item must correspond to the matching frame hand.")
            if item.frame_index != self.frame.frame_index:
                raise ValueError("Evidence frame_index must match HandFrame.")


@dataclass(frozen=True)
class GestureEvent:
    """A single dwell-confirmed perception fact for later Gesture Grammar."""

    event_id: int
    gesture: GestureKind
    hand_key: HandKey
    source_frame_index: int
    evidence_started_capture_t_ms: float
    confirmed_capture_t_ms: float
    available_t_ms: float
    dwell_ms: float
    recognizer_score: float
    observed_hand_count: int
    source: GestureSource = GestureSource.MEDIAPIPE_CANNED
    schema_version: str = EVENT_SCHEMA_VERSION

    def __post_init__(self) -> None:
        _nonnegative_int(self.event_id, "event_id", minimum=1)
        if self.gesture is GestureKind.UNKNOWN or not isinstance(self.gesture, GestureKind):
            raise ValueError("GestureEvent requires one known GestureKind.")
        if self.hand_key not in {HandKey.LEFT, HandKey.RIGHT}:
            raise ValueError("GestureEvent requires LEFT or RIGHT hand_key.")
        _nonnegative_int(self.source_frame_index, "source_frame_index")
        started = _finite_nonnegative(self.evidence_started_capture_t_ms, "evidence_started_capture_t_ms")
        confirmed = _finite_nonnegative(self.confirmed_capture_t_ms, "confirmed_capture_t_ms")
        available = _finite_nonnegative(self.available_t_ms, "available_t_ms")
        dwell = _finite_nonnegative(self.dwell_ms, "dwell_ms")
        if confirmed < started or available < confirmed:
            raise ValueError("GestureEvent timestamps are not chronologically valid.")
        if dwell > confirmed - started + 1e-6:
            raise ValueError("dwell_ms cannot exceed the capture-time confirmation interval.")
        score = _score(self.recognizer_score, "recognizer_score")
        _nonnegative_int(self.observed_hand_count, "observed_hand_count", minimum=1)
        if self.observed_hand_count > 2:
            raise ValueError("observed_hand_count must be one or two.")
        if not isinstance(self.source, GestureSource):
            raise TypeError("source must be a GestureSource.")
        if self.schema_version != EVENT_SCHEMA_VERSION:
            raise ValueError(f"Unsupported GestureEvent schema_version: {self.schema_version!r}")
        object.__setattr__(self, "evidence_started_capture_t_ms", started)
        object.__setattr__(self, "confirmed_capture_t_ms", confirmed)
        object.__setattr__(self, "available_t_ms", available)
        object.__setattr__(self, "dwell_ms", dwell)
        object.__setattr__(self, "recognizer_score", score)

    def to_dict(self) -> dict[str, object]:
        """Return JSON-safe audit data without raw landmarks or image frames."""
        return {
            "schema_version": self.schema_version,
            "event_id": self.event_id,
            "gesture": self.gesture.value,
            "hand_key": self.hand_key.value,
            "source_frame_index": self.source_frame_index,
            "evidence_started_capture_t_ms": self.evidence_started_capture_t_ms,
            "confirmed_capture_t_ms": self.confirmed_capture_t_ms,
            "available_t_ms": self.available_t_ms,
            "dwell_ms": self.dwell_ms,
            "recognizer_score": self.recognizer_score,
            "observed_hand_count": self.observed_hand_count,
            "source": self.source.value,
        }
