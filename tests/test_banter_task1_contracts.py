"""Contract tests for the isolated Banter Task 1 boundary."""

from __future__ import annotations

import pytest

from banter.contracts import (
    GestureEvidence,
    GestureEvent,
    GestureKind,
    GestureSource,
    HandFrame,
    HandKey,
)
from stage2.hand_observation import HandObservation


def _hand(timestamp_ms: float) -> HandObservation:
    return HandObservation(
        timestamp_ms=timestamp_ms,
        detected=True,
        handedness="Left",
        handedness_score=0.9,
        landmarks_norm=tuple((0.0, 0.0, 0.0) for _ in range(21)),
        index_tip_norm=(0.0, 0.0),
        index_tip_px=(0, 0),
    )


def test_hand_frame_represent_zero_one_and_two_hands_without_a_fake_no_hand() -> None:
    assert HandFrame(0, 10.0, 11.0, ()).hands == ()
    first, second = _hand(10.0), _hand(10.0)
    assert HandFrame(0, 10.0, 11.0, (first,)).hands == (first,)
    assert HandFrame(0, 10.0, 11.0, (first, second)).hands == (first, second)


def test_contract_rejects_more_than_two_hands_and_unknown_score() -> None:
    hand = _hand(10.0)
    with pytest.raises(ValueError, match="zero to two"):
        HandFrame(0, 10.0, 11.0, (hand, hand, hand))
    with pytest.raises(ValueError, match="UNKNOWN evidence"):
        GestureEvidence(
            frame_index=0,
            capture_t_ms=10.0,
            available_t_ms=11.0,
            hand=hand,
            hand_key=HandKey.LEFT,
            gesture=GestureKind.UNKNOWN,
            recognizer_score=0.5,
        )


def test_event_audit_payload_is_json_safe_and_excludes_landmarks() -> None:
    event = GestureEvent(
        event_id=1,
        gesture=GestureKind.POINT_ONE,
        hand_key=HandKey.LEFT,
        source_frame_index=12,
        evidence_started_capture_t_ms=100.0,
        confirmed_capture_t_ms=500.0,
        available_t_ms=503.0,
        dwell_ms=400.0,
        recognizer_score=0.91,
        observed_hand_count=2,
        source=GestureSource.MEDIAPIPE_CANNED,
    )
    payload = event.to_dict()
    assert payload["schema_version"] == "banter_gesture_event_v1"
    assert payload["gesture"] == "POINT_ONE"
    assert "landmarks" not in payload
