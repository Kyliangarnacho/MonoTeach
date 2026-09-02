"""Deterministic dwell, hysteresis, release, and two-hand event tests."""

from __future__ import annotations

import pytest

from banter.contracts import GestureEvidence, GestureKind, GesturePerception, HandFrame, HandKey
from banter.gesture_events import GestureEventConfig, GestureEventStabilizer
from stage2.hand_observation import HandObservation


def _hand(timestamp_ms: float, side: HandKey) -> HandObservation:
    return HandObservation(
        timestamp_ms=timestamp_ms,
        detected=True,
        handedness=side.value.title(),
        handedness_score=0.9,
        landmarks_norm=tuple((0.0, 0.0, 0.0) for _ in range(21)),
        index_tip_norm=(0.0, 0.0),
        index_tip_px=(0, 0),
    )


def _perception(
    timestamp_ms: float,
    items: tuple[tuple[HandKey, GestureKind, float], ...] = (),
    *,
    frame_index: int | None = None,
) -> GesturePerception:
    index = int(timestamp_ms) if frame_index is None else frame_index
    hands = tuple(_hand(timestamp_ms, side) for side, _gesture, _score in items)
    evidence = tuple(
        GestureEvidence(
            frame_index=index,
            capture_t_ms=timestamp_ms,
            available_t_ms=timestamp_ms + 2.0,
            hand=hand,
            hand_key=side,
            gesture=gesture,
            recognizer_score=None if gesture is GestureKind.UNKNOWN else score,
        )
        for hand, (side, gesture, score) in zip(hands, items)
    )
    return GesturePerception(
        frame=HandFrame(index, timestamp_ms, timestamp_ms + 2.0, hands),
        evidence=evidence,
    )


def _stabilizer() -> GestureEventStabilizer:
    return GestureEventStabilizer(
        GestureEventConfig(enter_score=0.65, hold_score=0.5, dwell_ms=100.0, release_ms=50.0, dropout_grace_ms=30.0)
    )


def test_held_gesture_publishes_once_then_requires_release_and_fresh_dwell() -> None:
    stabilizer = _stabilizer()
    point = (HandKey.LEFT, GestureKind.POINT_ONE, 0.9)
    assert stabilizer.update(_perception(0.0, (point,))) == ()
    first = stabilizer.update(_perception(100.0, (point,)))
    assert len(first) == 1 and first[0].gesture is GestureKind.POINT_ONE
    assert first[0].dwell_ms == 100.0
    assert stabilizer.update(_perception(200.0, (point,))) == ()

    fist = (HandKey.LEFT, GestureKind.FIST, 0.9)
    assert stabilizer.update(_perception(210.0, (fist,))) == ()
    assert stabilizer.update(_perception(260.0, (fist,))) == ()  # release completes; candidate begins
    assert stabilizer.update(_perception(300.0, (fist,))) == ()
    second = stabilizer.update(_perception(360.0, (fist,)))
    assert len(second) == 1 and second[0].gesture is GestureKind.FIST
    assert second[0].event_id == 2


def test_candidate_short_dropout_pauses_dwell_and_does_not_count_missing_time() -> None:
    stabilizer = _stabilizer()
    point = (HandKey.LEFT, GestureKind.POINT_ONE, 0.9)
    assert stabilizer.update(_perception(0.0, (point,))) == ()
    assert stabilizer.update(_perception(30.0)) == ()
    assert stabilizer.update(_perception(60.0, (point,))) == ()
    event = stabilizer.update(_perception(130.0, (point,)))
    assert len(event) == 1
    assert event[0].dwell_ms == 100.0


def test_active_pose_eventually_rearms_after_a_long_dropout() -> None:
    stabilizer = _stabilizer()
    point = (HandKey.RIGHT, GestureKind.POINT_ONE, 0.9)
    stabilizer.update(_perception(0.0, (point,)))
    assert len(stabilizer.update(_perception(100.0, (point,)))) == 1
    assert stabilizer.update(_perception(120.0)) == ()
    assert stabilizer.update(_perception(160.0)) == ()
    assert stabilizer.update(_perception(210.0)) == ()
    assert stabilizer.update(_perception(220.0, (point,))) == ()
    assert len(stabilizer.update(_perception(320.0, (point,)))) == 1


def test_left_and_right_hands_confirm_independently_in_a_single_frame() -> None:
    stabilizer = _stabilizer()
    both = (
        (HandKey.LEFT, GestureKind.OPEN_PALM_FIVE, 0.9),
        (HandKey.RIGHT, GestureKind.VICTORY_TWO, 0.9),
    )
    assert stabilizer.update(_perception(0.0, both)) == ()
    events = stabilizer.update(_perception(100.0, both))
    assert [(event.hand_key, event.gesture) for event in events] == [
        (HandKey.LEFT, GestureKind.OPEN_PALM_FIVE),
        (HandKey.RIGHT, GestureKind.VICTORY_TWO),
    ]
    assert [event.observed_hand_count for event in events] == [2, 2]


def test_unknown_side_never_enters_the_event_channel_and_time_must_increase() -> None:
    stabilizer = _stabilizer()
    unknown = (HandKey.UNKNOWN, GestureKind.POINT_ONE, 0.9)
    assert stabilizer.update(_perception(0.0, (unknown,))) == ()
    assert stabilizer.update(_perception(100.0, (unknown,))) == ()
    with pytest.raises(ValueError, match="strictly increasing"):
        stabilizer.update(_perception(100.0, (unknown,)))


def test_unknown_gesture_never_enters_the_event_channel() -> None:
    stabilizer = _stabilizer()
    rejected = (HandKey.LEFT, GestureKind.UNKNOWN, 0.0)
    assert stabilizer.update(_perception(0.0, (rejected,))) == ()
    assert stabilizer.update(_perception(100.0, (rejected,))) == ()
