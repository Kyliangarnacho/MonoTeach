"""Offline Task 3 verify; it never opens a camera or reads landmarks."""

from __future__ import annotations

from .contracts import GestureEvent, GestureKind, HandKey
from .gesture_grammar import GestureGrammarEngine
from .grammar_contracts import GrammarUpdate, PhraseForm
from .interaction_memory import InteractionMemory


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
    """Check cumulative facts, independent lanes, and explicit session reset."""
    print("[1/3] Checking repeated Token totals and capture-time streaks...")
    memory = InteractionMemory("verify-session", clock_ms=lambda: 20_000.0)
    first = memory.update(GrammarUpdate((_token(1, GestureKind.FIST, HandKey.LEFT, 100.0),), ()))
    second = memory.update(GrammarUpdate((_token(2, GestureKind.FIST, HandKey.LEFT, 300.0),), ()))
    changed = memory.update(GrammarUpdate((_token(3, GestureKind.OPEN_PALM_FIVE, HandKey.LEFT, 600.0),), ()))
    assert first.token_total(HandKey.LEFT, GestureKind.FIST) == 1
    assert second.token_total(HandKey.LEFT, GestureKind.FIST) == 2
    assert changed.token_total(HandKey.LEFT, GestureKind.FIST) == 2
    assert changed.left_token_streak is not None
    assert changed.left_token_streak.key.gesture is GestureKind.OPEN_PALM_FIVE

    print("[2/3] Checking Task 2 Phrase provenance stays separate from Tokens...")
    grammar = GestureGrammarEngine(clock_ms=lambda: 10_000.0)
    update = grammar.update((
        _token(4, GestureKind.FIST, HandKey.LEFT, 1_000.0),
        _token(5, GestureKind.VICTORY_TWO, HandKey.RIGHT, 1_100.0),
    ))
    phrase_snapshot = memory.update(update)
    assert phrase_snapshot.total_phrase_count == 1
    assert phrase_snapshot.phrase_total(PhraseForm.CHORD, "POWER_VICTORY") == 1
    assert phrase_snapshot.total_token_count == 5
    assert phrase_snapshot.phrase_streak is not None and phrase_snapshot.phrase_streak.count == 1

    print("[3/3] Checking empty input and explicit session reset...")
    assert memory.update(GrammarUpdate((), ())) is phrase_snapshot
    reset = memory.reset("verify-session-reset")
    assert reset.revision == reset.total_token_count == reset.total_phrase_count == 0
    assert reset.session_id == "verify-session-reset"
    print("PASS: Task 3 canonical interaction context, totals, streaks, provenance, and reset verified offline.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
