"""Offline verify for Banter Task 2 Gesture Grammar; it never opens a camera."""

from __future__ import annotations

from .contracts import GestureEvent, GestureKind, HandKey
from .gesture_grammar import GestureGrammarEngine


def _token(event_id: int, gesture: GestureKind, hand_key: HandKey, confirmed_t_ms: float) -> GestureEvent:
    return GestureEvent(
        event_id=event_id,
        gesture=gesture,
        hand_key=hand_key,
        source_frame_index=event_id,
        evidence_started_capture_t_ms=max(0.0, confirmed_t_ms - 100.0),
        confirmed_capture_t_ms=confirmed_t_ms,
        available_t_ms=confirmed_t_ms + 5.0,
        dwell_ms=min(100.0, confirmed_t_ms),
        recognizer_score=0.9,
        observed_hand_count=1,
    )


def main() -> int:
    """Check the default Chord and Sequence definitions with synthetic Tokens."""
    print("[1/2] Checking LEFT:FIST + RIGHT:VICTORY_TWO chord...")
    grammar = GestureGrammarEngine(clock_ms=lambda: 10_000.0)
    chord = grammar.update((
        _token(1, GestureKind.FIST, HandKey.LEFT, 100.0),
        _token(2, GestureKind.VICTORY_TWO, HandKey.RIGHT, 180.0),
    ))
    assert [phrase.name for phrase in chord.phrases] == ["POWER_VICTORY"]
    assert chord.phrases[0].source_event_ids == (1, 2)

    print("[2/2] Checking same-hand OPEN_PALM_FIVE -> FIST -> VICTORY_TWO sequence...")
    grammar = GestureGrammarEngine(clock_ms=lambda: 10_000.0)
    assert grammar.update((_token(1, GestureKind.OPEN_PALM_FIVE, HandKey.LEFT, 100.0),)).phrases == ()
    assert grammar.update((_token(2, GestureKind.FIST, HandKey.LEFT, 900.0),)).phrases == ()
    sequence = grammar.update((_token(3, GestureKind.VICTORY_TWO, HandKey.LEFT, 1_700.0),))
    assert [phrase.name for phrase in sequence.phrases] == ["CHALLENGE_TRIUMPH"]
    assert sequence.phrases[0].source_event_ids == (1, 2, 3)
    print("PASS: Task 2 Token provenance, Chord, and same-hand Sequence verified offline.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
