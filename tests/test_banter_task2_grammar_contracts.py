"""Contract tests for Task 2 derived Gesture Grammar facts."""

from __future__ import annotations

import pytest

from banter.contracts import GestureEvent, GestureKind, HandKey
from banter.grammar_contracts import (
    ChordRule,
    GesturePhraseEvent,
    GrammarUpdate,
    PhraseForm,
    SequenceRule,
)


def _event(event_id: int, gesture: GestureKind, hand_key: HandKey, t_ms: float) -> GestureEvent:
    return GestureEvent(
        event_id=event_id,
        gesture=gesture,
        hand_key=hand_key,
        source_frame_index=event_id,
        evidence_started_capture_t_ms=max(0.0, t_ms - 100.0),
        confirmed_capture_t_ms=t_ms,
        available_t_ms=t_ms + 5.0,
        dwell_ms=min(100.0, t_ms),
        recognizer_score=0.9,
        observed_hand_count=1,
    )


def test_rules_reject_unknown_gestures_and_invalid_windows() -> None:
    with pytest.raises(ValueError, match="known GestureKind"):
        ChordRule("BAD", GestureKind.UNKNOWN, GestureKind.FIST, 100.0)
    with pytest.raises(ValueError, match="at least two"):
        SequenceRule("BAD", (GestureKind.FIST,), 100.0, 200.0)
    with pytest.raises(ValueError, match="at least max_step_gap"):
        SequenceRule("BAD", (GestureKind.FIST, GestureKind.VICTORY_TWO), 300.0, 200.0)


def test_phrase_json_retains_only_event_provenance_and_derived_timing() -> None:
    left = _event(1, GestureKind.FIST, HandKey.LEFT, 100.0)
    right = _event(2, GestureKind.VICTORY_TWO, HandKey.RIGHT, 130.0)
    phrase = GesturePhraseEvent(
        phrase_id=1,
        name="POWER_VICTORY",
        form=PhraseForm.CHORD,
        source_events=(left, right),
        available_t_ms=140.0,
    )
    payload = phrase.to_dict()
    assert phrase.started_capture_t_ms == 100.0
    assert phrase.completed_capture_t_ms == 130.0
    assert payload["source_event_ids"] == (1, 2)
    assert payload["source_tokens"][0]["gesture"] == "FIST"
    assert "landmarks" not in payload


def test_phrase_cannot_claim_availability_before_its_source_events() -> None:
    event = _event(1, GestureKind.FIST, HandKey.LEFT, 100.0)
    other = _event(2, GestureKind.VICTORY_TWO, HandKey.RIGHT, 110.0)
    with pytest.raises(ValueError, match="must not precede"):
        GesturePhraseEvent(1, "POWER_VICTORY", PhraseForm.CHORD, (event, other), 100.0)
    with pytest.raises(TypeError, match="both be tuples"):
        GrammarUpdate(tokens=[], phrases=())  # type: ignore[arg-type]
