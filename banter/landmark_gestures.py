"""Conservative static-gesture recognition from one MediaPipe hand landmark set.

This module deliberately recognizes only the small Banter vocabulary.  Any
finger pattern that is not an unambiguous match is rejected as ``UNKNOWN``;
in particular, numeric three/four and a raised middle finger are not coerced
into one of the supported commands.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Sequence

from .contracts import GestureKind


Landmark = tuple[float, float, float]


class FingerState(str, Enum):
    EXTENDED = "EXTENDED"
    FOLDED = "FOLDED"
    AMBIGUOUS = "AMBIGUOUS"


@dataclass(frozen=True)
class LandmarkGestureDecision:
    """One pure geometric classification result for a detected hand."""

    gesture: GestureKind
    score: float | None
    rejection_reason: str | None = None

    def __post_init__(self) -> None:
        if self.gesture is GestureKind.UNKNOWN:
            if self.score is not None:
                raise ValueError("UNKNOWN landmark decisions must not have a score.")
            if not self.rejection_reason:
                raise ValueError("UNKNOWN landmark decisions need a rejection reason.")
        elif self.score is None or not 0.0 <= self.score <= 1.0:
            raise ValueError("Known landmark decisions need a score in [0, 1].")


_FINGER_CHAINS = {
    "thumb": (1, 2, 3, 4),
    "index": (5, 6, 7, 8),
    "middle": (9, 10, 11, 12),
    "ring": (13, 14, 15, 16),
    "pinky": (17, 18, 19, 20),
}

# The two thresholds leave an intentional rejection band.  The score combines
# chain straightness and endpoint reach, both scale-independent in 3-D.
_EXTENDED_MIN = 0.84
_FOLDED_MAX = 0.60
_EPSILON = 1e-8

# Thumb chains behave differently from the four fingers: a thumb folded
# across a fist can still be geometrically straight.  Openness is therefore
# evaluated against the palm, not inferred from joint straightness alone.
_THUMB_CHAIN_MIN = 0.72
_THUMB_TIP_TO_INDEX_MIN = 0.70
_THUMB_TIP_TO_PALM_MIN = 0.60
_THUMB_UPWARD_MIN = 0.45
# OPEN_PALM_FIVE intentionally accepts a natural, slightly bent/splayed
# thumb.  It has its own lighter gate; the stricter gate above remains the
# safety boundary for V/POINT versus numeric three and for THUMBS_UP.
_THUMB_FIVE_CHAIN_MIN = 0.58
_THUMB_FIVE_TIP_TO_INDEX_MIN = 0.62


def _sub(a: Landmark, b: Landmark) -> Landmark:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _norm(vector: Landmark) -> float:
    return math.sqrt(sum(component * component for component in vector))


def _dot(a: Landmark, b: Landmark) -> float:
    return sum(left * right for left, right in zip(a, b))


def _distance(a: Landmark, b: Landmark) -> float:
    return _norm(_sub(a, b))


def _mean(points: Sequence[Landmark]) -> Landmark:
    count = float(len(points))
    return tuple(sum(point[axis] for point in points) / count for axis in range(3))  # type: ignore[return-value]


def _finger_score(landmarks: tuple[Landmark, ...], chain: tuple[int, int, int, int]) -> float | None:
    points = tuple(landmarks[index] for index in chain)
    segments = tuple(_sub(later, earlier) for earlier, later in zip(points, points[1:]))
    lengths = tuple(_norm(segment) for segment in segments)
    if min(lengths) <= _EPSILON:
        return None
    straightness = min(
        max(-1.0, min(1.0, _dot(first, second) / (first_length * second_length)))
        for first, second, first_length, second_length in zip(
            segments,
            segments[1:],
            lengths,
            lengths[1:],
        )
    )
    # A straight chain has cosine/reach close to one; a curled chain does not.
    normalized_straightness = (straightness + 1.0) / 2.0
    reach = _norm(_sub(points[-1], points[0])) / sum(lengths)
    return min(normalized_straightness, reach)


def _state(score: float) -> FingerState:
    if score >= _EXTENDED_MIN:
        return FingerState.EXTENDED
    if score <= _FOLDED_MAX:
        return FingerState.FOLDED
    return FingerState.AMBIGUOUS


def _known(kind: GestureKind, *supports: float) -> LandmarkGestureDecision:
    # The least convincing required digit is the rule confidence.  It is a
    # derived geometric margin, not a MediaPipe canned-classifier probability.
    return LandmarkGestureDecision(kind, min(supports))


@dataclass(frozen=True)
class _ThumbFeatures:
    chain_score: float
    tip_to_index_ratio: float
    tip_to_palm_ratio: float
    upward_projection: float


def _thumb_features(landmarks: tuple[Landmark, ...], chain_score: float) -> _ThumbFeatures | None:
    """Measure thumb openness relative to this hand's own palm scale."""
    wrist = landmarks[0]
    index_mcp = landmarks[5]
    middle_mcp = landmarks[9]
    pinky_mcp = landmarks[17]
    thumb_mcp = landmarks[2]
    thumb_tip = landmarks[4]
    palm_scale = max(_distance(wrist, middle_mcp), _distance(index_mcp, pinky_mcp))
    palm_axis = _sub(middle_mcp, wrist)
    palm_axis_length = _norm(palm_axis)
    if palm_scale <= _EPSILON or palm_axis_length <= _EPSILON:
        return None
    palm_center = _mean((wrist, index_mcp, middle_mcp, pinky_mcp))
    upward_projection = _dot(_sub(thumb_tip, thumb_mcp), palm_axis) / (palm_axis_length * palm_scale)
    return _ThumbFeatures(
        chain_score=chain_score,
        tip_to_index_ratio=_distance(thumb_tip, index_mcp) / palm_scale,
        tip_to_palm_ratio=_distance(thumb_tip, palm_center) / palm_scale,
        upward_projection=upward_projection,
    )


def _thumb_is_open(features: _ThumbFeatures) -> bool:
    """Require a straight thumb to be demonstrably away from the palm."""
    return bool(
        features.chain_score >= _THUMB_CHAIN_MIN
        and features.tip_to_index_ratio >= _THUMB_TIP_TO_INDEX_MIN
        and features.tip_to_palm_ratio >= _THUMB_TIP_TO_PALM_MIN
    )


def _thumb_open_confidence(features: _ThumbFeatures) -> float:
    """Return a bounded score only after ``_thumb_is_open`` has passed."""
    return min(
        features.chain_score,
        1.0,
        features.tip_to_index_ratio / _THUMB_TIP_TO_INDEX_MIN,
        features.tip_to_palm_ratio / _THUMB_TIP_TO_PALM_MIN,
    )


def _thumb_is_open_for_five(features: _ThumbFeatures) -> bool:
    """Accept a naturally splayed thumb without weakening command-pose gates."""
    return bool(
        features.chain_score >= _THUMB_FIVE_CHAIN_MIN
        and features.tip_to_index_ratio >= _THUMB_FIVE_TIP_TO_INDEX_MIN
    )


def _thumb_five_confidence(features: _ThumbFeatures) -> float:
    """Return a bounded OPEN_PALM_FIVE thumb score after its lighter gate."""
    return min(
        features.chain_score,
        1.0,
        features.tip_to_index_ratio / _THUMB_FIVE_TIP_TO_INDEX_MIN,
    )


def _is_thumb_up(features: _ThumbFeatures) -> bool:
    """Recognize a thumb-up only with positive palm-relative evidence."""
    return _thumb_is_open(features) and features.upward_projection >= _THUMB_UPWARD_MIN


def classify_landmarks(landmarks: Sequence[Landmark]) -> LandmarkGestureDecision:
    """Classify one 21-landmark hand into a supported gesture or ``UNKNOWN``.

    No caller state is retained here: temporal stability is the responsibility
    of :class:`GestureEventStabilizer`.  Invalid or ambiguous geometry remains
    explicit ``UNKNOWN`` rather than being guessed into a command pose.
    """
    try:
        normalized = tuple((float(x), float(y), float(z)) for x, y, z in landmarks)
    except (TypeError, ValueError):
        return LandmarkGestureDecision(GestureKind.UNKNOWN, None, "invalid_landmarks")
    if len(normalized) != 21 or not all(math.isfinite(value) for point in normalized for value in point):
        return LandmarkGestureDecision(GestureKind.UNKNOWN, None, "invalid_landmarks")

    scores = {name: _finger_score(normalized, chain) for name, chain in _FINGER_CHAINS.items()}
    if any(score is None for score in scores.values()):
        return LandmarkGestureDecision(GestureKind.UNKNOWN, None, "degenerate_landmark_geometry")
    numeric_scores = {name: float(score) for name, score in scores.items()}
    states = {name: _state(score) for name, score in numeric_scores.items()}
    thumb_features = _thumb_features(normalized, numeric_scores["thumb"])
    if thumb_features is None:
        return LandmarkGestureDecision(GestureKind.UNKNOWN, None, "degenerate_palm_geometry")
    thumb_is_open = _thumb_is_open(thumb_features)

    if (
        all(states[name] is FingerState.EXTENDED for name in ("index", "middle", "ring", "pinky"))
        and _thumb_is_open_for_five(thumb_features)
    ):
        return _known(
            GestureKind.OPEN_PALM_FIVE,
            *(numeric_scores[name] for name in ("index", "middle", "ring", "pinky")),
            _thumb_five_confidence(thumb_features),
        )
    if (
        states["index"] is FingerState.EXTENDED
        and states["middle"] is FingerState.EXTENDED
        and states["ring"] is FingerState.FOLDED
        and states["pinky"] is FingerState.FOLDED
        and not thumb_is_open
    ):
        return _known(
            GestureKind.VICTORY_TWO,
            numeric_scores["index"],
            numeric_scores["middle"],
            1.0 - numeric_scores["ring"],
            1.0 - numeric_scores["pinky"],
        )
    if (
        states["middle"] is FingerState.EXTENDED
        and all(states[name] is FingerState.FOLDED for name in ("index", "ring", "pinky"))
        and not thumb_is_open
    ):
        return _known(
            GestureKind.MIDDLE_FINGER,
            numeric_scores["middle"],
            1.0 - numeric_scores["index"],
            1.0 - numeric_scores["ring"],
            1.0 - numeric_scores["pinky"],
        )
    if (
        states["index"] is FingerState.EXTENDED
        and all(states[name] is FingerState.FOLDED for name in ("middle", "ring", "pinky"))
        and not thumb_is_open
    ):
        return _known(
            GestureKind.POINT_ONE,
            numeric_scores["index"],
            1.0 - numeric_scores["middle"],
            1.0 - numeric_scores["ring"],
            1.0 - numeric_scores["pinky"],
        )
    if all(states[name] is FingerState.FOLDED for name in ("index", "middle", "ring", "pinky")):
        if _is_thumb_up(thumb_features):
            return _known(
                GestureKind.THUMBS_UP,
                _thumb_open_confidence(thumb_features),
                min(1.0, thumb_features.upward_projection / _THUMB_UPWARD_MIN),
                *(1.0 - numeric_scores[name] for name in ("index", "middle", "ring", "pinky")),
            )
        return _known(
            GestureKind.FIST,
            *(1.0 - numeric_scores[name] for name in ("index", "middle", "ring", "pinky")),
        )

    if any(state is FingerState.AMBIGUOUS for state in states.values()):
        reason = "ambiguous_landmark_gesture"
    else:
        reason = "unsupported_landmark_gesture"
    return LandmarkGestureDecision(GestureKind.UNKNOWN, None, reason)
