"""Camera-free unit tests for Banter's conservative landmark gesture rules."""

from __future__ import annotations

from banter.contracts import GestureKind
from banter.landmark_gestures import classify_landmarks


_CHAINS = {
    "thumb": (1, 2, 3, 4),
    "index": (5, 6, 7, 8),
    "middle": (9, 10, 11, 12),
    "ring": (13, 14, 15, 16),
    "pinky": (17, 18, 19, 20),
}
_BASE_X = {"thumb": 0.28, "index": 0.40, "middle": 0.50, "ring": 0.60, "pinky": 0.70}


def _landmarks(*, extended: set[str], thumb_up: bool = False) -> tuple[tuple[float, float, float], ...]:
    """Build non-degenerate synthetic chains with obvious folded/straight states."""
    points = [(0.5, 0.85, 0.0) for _ in range(21)]
    for name, chain in _CHAINS.items():
        x = _BASE_X[name]
        if name in extended:
            chain_points = ((x, 0.65, 0.0), (x, 0.50, 0.0), (x, 0.35, 0.0), (x, 0.20, 0.0))
        else:
            # The finger doubles back toward its base, yielding low reach and
            # low chain straightness without zero-length segments.
            chain_points = ((x, 0.65, 0.0), (x, 0.50, 0.0), (x + 0.02, 0.62, 0.0), (x - 0.02, 0.66, 0.0))
        for index, point in zip(chain, chain_points):
            points[index] = point
    if thumb_up:
        x = _BASE_X["thumb"]
        for index, point in zip(_CHAINS["thumb"], ((x, 0.70, 0.0), (x, 0.52, 0.0), (x, 0.34, 0.0), (x, 0.16, 0.0))):
            points[index] = point
    return tuple(points)


def _with_cross_palm_thumb(landmarks: tuple[tuple[float, float, float], ...]) -> tuple[tuple[float, float, float], ...]:
    """Model a folded thumb that is straight in the image but stays near index MCP.

    This matches the C920 false-positive shape: the old chain-only rule called
    it THUMBS_UP because its tip is high and the chain is straight.
    """
    points = list(landmarks)
    for index, point in zip(_CHAINS["thumb"], ((0.28, 0.72, 0.0), (0.31, 0.64, 0.0), (0.35, 0.56, 0.0), (0.40, 0.48, 0.0))):
        points[index] = point
    return tuple(points)


def _with_natural_open_palm_thumb(landmarks: tuple[tuple[float, float, float], ...]) -> tuple[tuple[float, float, float], ...]:
    """A splayed but not maximally straight thumb from a natural five pose."""
    points = list(landmarks)
    for index, point in zip(_CHAINS["thumb"], ((0.28, 0.70, 0.0), (0.30, 0.62, 0.0), (0.26, 0.57, 0.0), (0.22, 0.56, 0.0))):
        points[index] = point
    return tuple(points)


def test_small_supported_vocabulary_has_explicit_landmark_patterns() -> None:
    cases = {
        GestureKind.FIST: _landmarks(extended=set()),
        GestureKind.POINT_ONE: _landmarks(extended={"index"}),
        GestureKind.VICTORY_TWO: _landmarks(extended={"index", "middle"}),
        GestureKind.OPEN_PALM_FIVE: _landmarks(extended=set(_CHAINS)),
        GestureKind.THUMBS_UP: _landmarks(extended={"thumb"}, thumb_up=True),
        GestureKind.MIDDLE_FINGER: _landmarks(extended={"middle"}),
    }
    for expected, landmarks in cases.items():
        decision = classify_landmarks(landmarks)
        assert decision.gesture is expected
        assert decision.score is not None and decision.score >= 0.65
        assert decision.rejection_reason is None


def test_three_four_middle_and_ambiguous_geometry_are_rejected_not_coerced() -> None:
    unsupported = (
        _landmarks(extended={"index", "middle", "ring"}),  # 3
        _landmarks(extended={"thumb", "index", "middle"}),  # common thumb-index-middle 3
        _landmarks(extended={"index", "middle", "ring", "pinky"}),  # 4
        tuple((0.5, 0.5, 0.0) for _ in range(21)),  # degenerate input
    )
    for landmarks in unsupported:
        decision = classify_landmarks(landmarks)
        assert decision.gesture is GestureKind.UNKNOWN
        assert decision.score is None
        assert decision.rejection_reason


def test_cross_palm_thumb_does_not_turn_fist_into_thumb_up_and_keeps_victory() -> None:
    fist = classify_landmarks(_with_cross_palm_thumb(_landmarks(extended=set())))
    assert fist.gesture is GestureKind.FIST
    assert fist.score is not None and fist.score >= 0.65

    victory = classify_landmarks(_with_cross_palm_thumb(_landmarks(extended={"index", "middle"})))
    assert victory.gesture is GestureKind.VICTORY_TWO
    assert victory.score is not None and victory.score >= 0.65

    thumb_up = classify_landmarks(_landmarks(extended={"thumb"}, thumb_up=True))
    assert thumb_up.gesture is GestureKind.THUMBS_UP


def test_naturally_splayed_thumb_is_enough_for_five_but_not_for_four() -> None:
    natural_five = classify_landmarks(
        _with_natural_open_palm_thumb(_landmarks(extended=set(_CHAINS)))
    )
    assert natural_five.gesture is GestureKind.OPEN_PALM_FIVE
    assert natural_five.score is not None and natural_five.score >= 0.65

    four_with_cross_palm_thumb = classify_landmarks(
        _with_cross_palm_thumb(_landmarks(extended={"index", "middle", "ring", "pinky"}))
    )
    assert four_with_cross_palm_thumb.gesture is GestureKind.UNKNOWN
