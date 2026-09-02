"""Offline Task 4 verify; it never opens a camera or commands a robot."""

from __future__ import annotations

from .contracts import GestureEvent, GestureKind, HandKey
from .gesture_grammar import GestureGrammarEngine
from .grammar_contracts import GrammarUpdate
from .interaction_memory import InteractionMemory
from .personality_behavior import PersonaBehaviorEngine


def _token(event_id: int, gesture: GestureKind, hand_key: HandKey, t_ms: float) -> GestureEvent:
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


def main() -> int:
    """Check concrete behavior names, prefix delay, provenance, and one-shot admission."""
    print("[1/3] Checking a concrete Token behavior without continuous intensity...")
    memory = InteractionMemory("verify-persona", clock_ms=lambda: 20_000.0)
    engine = PersonaBehaviorEngine("verify-persona", clock_ms=lambda: 30_000.0)
    thumbs = engine.update(memory.update(GrammarUpdate((_token(1, GestureKind.THUMBS_UP, HandKey.LEFT, 100.0),), ())))
    assert thumbs.behavior is not None
    assert (thumbs.behavior.behavior_name, thumbs.behavior.variant) == ("THUMBS_UP_ACK", "DEFAULT")
    assert not hasattr(thumbs.behavior, "intensity")

    print("[2/3] Checking right-first chord provenance and phrase-specific behavior...")
    memory = InteractionMemory("verify-chord", clock_ms=lambda: 20_000.0)
    engine = PersonaBehaviorEngine("verify-chord", clock_ms=lambda: 30_000.0)
    grammar = GestureGrammarEngine(clock_ms=lambda: 10_000.0)
    right = _token(2, GestureKind.VICTORY_TWO, HandKey.RIGHT, 200.0)
    left = _token(3, GestureKind.FIST, HandKey.LEFT, 250.0)
    first = engine.update(memory.update(grammar.update((right,))))
    assert first.behavior is None
    chord = grammar.update((left,))
    power = engine.update(memory.update(chord))
    assert [phrase.source_event_ids for phrase in chord.phrases] == [(2, 3)]
    assert power.behavior is not None
    assert power.behavior.behavior_name == "POWER_POSE_REPLY"
    assert power.behavior.variant == "DEFAULT"

    print("[3/3] Checking wake-prefixed Sequence produces one phrase-specific behavior...")
    sequence_memory = InteractionMemory("verify-sequence", clock_ms=lambda: 20_000.0)
    sequence_engine = PersonaBehaviorEngine("verify-sequence", clock_ms=lambda: 30_000.0)
    sequence_grammar = GestureGrammarEngine(clock_ms=lambda: 10_000.0)
    for token in (
        _token(1, GestureKind.OPEN_PALM_FIVE, HandKey.LEFT, 100.0),
        _token(2, GestureKind.FIST, HandKey.LEFT, 500.0),
        _token(3, GestureKind.VICTORY_TWO, HandKey.LEFT, 900.0),
    ):
        update = sequence_engine.update(sequence_memory.update(sequence_grammar.update((token,))))
    assert update.behavior is not None
    assert update.behavior.behavior_name == "TRIUMPH_FLOURISH"
    assert update.behavior.variant == "DEFAULT"
    print("PASS: Task 4 concrete behavior routing, prefix buffering, and canonical provenance verified offline.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
