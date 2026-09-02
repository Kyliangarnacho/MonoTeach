"""Deterministic reducer/store tests for Task 3 interaction memory."""

from __future__ import annotations

import pytest

from banter.contracts import GestureEvent, GestureKind, HandKey
from banter.grammar_contracts import GesturePhraseEvent, GrammarUpdate, PhraseForm
from banter.interaction_memory import InteractionMemory, fold_interaction_memory
from banter.memory_contracts import InteractionMemoryConfig


def _token(event_id: int, gesture: GestureKind, hand: HandKey, t_ms: float) -> GestureEvent:
    return GestureEvent(
        event_id=event_id,
        gesture=gesture,
        hand_key=hand,
        source_frame_index=event_id,
        evidence_started_capture_t_ms=max(0.0, t_ms - 50.0),
        confirmed_capture_t_ms=t_ms,
        available_t_ms=t_ms + 5.0,
        dwell_ms=min(50.0, t_ms),
        recognizer_score=0.9,
        observed_hand_count=1,
    )


def _phrase(phrase_id: int, name: str, sources: tuple[GestureEvent, ...], available_t_ms: float) -> GesturePhraseEvent:
    return GesturePhraseEvent(phrase_id, name, PhraseForm.CHORD, sources, available_t_ms)


def test_totals_streaks_and_previous_snapshot_are_immutable() -> None:
    memory = InteractionMemory("session-a", clock_ms=lambda: 10_000.0)
    first = memory.update(GrammarUpdate((_token(1, GestureKind.FIST, HandKey.LEFT, 100.0),), ()))
    second = memory.update(GrammarUpdate((_token(2, GestureKind.FIST, HandKey.LEFT, 400.0),), ()))
    third = memory.update(GrammarUpdate((_token(3, GestureKind.OPEN_PALM_FIVE, HandKey.LEFT, 800.0),), ()))

    assert first.total_token_count == 1
    assert first.left_token_streak is not None and first.left_token_streak.count == 1
    assert second.token_total(HandKey.LEFT, GestureKind.FIST) == 2
    assert second.left_token_streak is not None and second.left_token_streak.count == 2
    assert third.token_total(HandKey.LEFT, GestureKind.FIST) == 2
    assert third.left_token_streak is not None
    assert third.left_token_streak.key.gesture is GestureKind.OPEN_PALM_FIVE
    assert third.left_token_streak.count == 1


def test_left_right_and_phrase_streak_lanes_do_not_interrupt_each_other() -> None:
    memory = InteractionMemory("session-b", clock_ms=lambda: 10_000.0)
    left_one = _token(1, GestureKind.FIST, HandKey.LEFT, 100.0)
    right = _token(2, GestureKind.THUMBS_UP, HandKey.RIGHT, 200.0)
    left_two = _token(3, GestureKind.FIST, HandKey.LEFT, 300.0)
    snapshot = memory.update(GrammarUpdate((left_one,), ()))
    snapshot = memory.update(GrammarUpdate((right,), ()))
    snapshot = memory.update(GrammarUpdate((left_two,), ()))
    assert snapshot.left_token_streak is not None and snapshot.left_token_streak.count == 2
    assert snapshot.right_token_streak is not None and snapshot.right_token_streak.count == 1

    phrase_one = _phrase(1, "POWER_VICTORY", (left_one, right), 400.0)
    snapshot = memory.update(GrammarUpdate((), (phrase_one,)))
    phrase_two_sources = (_token(4, GestureKind.FIST, HandKey.LEFT, 500.0), _token(5, GestureKind.VICTORY_TWO, HandKey.RIGHT, 550.0))
    snapshot = memory.update(GrammarUpdate(phrase_two_sources, (_phrase(2, "POWER_VICTORY", phrase_two_sources, 600.0),)))
    assert snapshot.phrase_total(PhraseForm.CHORD, "POWER_VICTORY") == 2
    assert snapshot.phrase_streak is not None and snapshot.phrase_streak.count == 2
    assert snapshot.left_token_streak is not None and snapshot.left_token_streak.count == 3


def test_streak_timeout_and_recent_eviction_do_not_reduce_cumulative_totals() -> None:
    config = InteractionMemoryConfig(recent_token_capacity=3, recent_phrase_capacity=1, token_streak_gap_ms=100.0)
    memory = InteractionMemory("session-c", config, clock_ms=lambda: 10_000.0)
    for event_id, t_ms in enumerate((100.0, 150.0, 300.0, 350.0), start=1):
        snapshot = memory.update(GrammarUpdate((_token(event_id, GestureKind.FIST, HandKey.LEFT, t_ms),), ()))
    assert tuple(event.event_id for event in snapshot.recent_tokens) == (2, 3, 4)
    assert snapshot.total_token_count == snapshot.token_total(HandKey.LEFT, GestureKind.FIST) == 4
    assert snapshot.left_token_streak is not None and snapshot.left_token_streak.count == 2


def test_empty_update_is_identity_and_does_not_read_clock() -> None:
    calls: list[str] = []
    memory = InteractionMemory("session-d", clock_ms=lambda: calls.append("clock") or 100.0)
    before = memory.snapshot
    assert memory.update(GrammarUpdate((), ())) is before
    assert calls == []
    assert before.revision == before.interaction_turn_count == 0


def test_invalid_batch_is_atomic_and_duplicate_provenance_is_rejected() -> None:
    memory = InteractionMemory("session-e", clock_ms=lambda: 10_000.0)
    first = _token(1, GestureKind.FIST, HandKey.LEFT, 100.0)
    second = _token(2, GestureKind.VICTORY_TWO, HandKey.RIGHT, 130.0)
    phrase = _phrase(1, "POWER_VICTORY", (first, second), 150.0)
    accepted = memory.update(GrammarUpdate((first, second), (phrase,)))
    duplicate = GesturePhraseEvent(2, "POWER_VICTORY", PhraseForm.CHORD, (first, second), 160.0)
    with pytest.raises(ValueError, match="duplicate Phrase provenance"):
        memory.update(GrammarUpdate((), (duplicate,)))
    assert memory.snapshot is accepted

    invalid = _token(3, GestureKind.FIST, HandKey.LEFT, 90.0)
    with pytest.raises(ValueError, match="non-decreasing Token"):
        memory.update(GrammarUpdate((invalid,), ()))
    assert memory.snapshot is accepted


def test_provenance_content_idempotency_post_fold_availability_and_reset() -> None:
    calls: list[str] = []
    memory = InteractionMemory("old", clock_ms=lambda: calls.append("post-fold") or 1_000.0)
    token = _token(1, GestureKind.FIST, HandKey.LEFT, 100.0)
    snapshot = memory.update(GrammarUpdate((token,), ()))
    assert calls == ["post-fold"]
    assert snapshot.last_input_available_t_ms == 105.0
    assert snapshot.available_t_ms == 1_000.0
    draft = fold_interaction_memory(memory.snapshot, GrammarUpdate((), ()), memory.config)
    assert draft is memory.snapshot

    wrong_content = _token(1, GestureKind.OPEN_PALM_FIVE, HandKey.LEFT, 100.0)
    bad_phrase = _phrase(1, "POWER_VICTORY", (wrong_content, _token(2, GestureKind.VICTORY_TWO, HandKey.RIGHT, 130.0)), 150.0)
    with pytest.raises(ValueError, match="different Token content"):
        memory.update(GrammarUpdate((_token(2, GestureKind.VICTORY_TWO, HandKey.RIGHT, 130.0),), (bad_phrase,)))
    reset = memory.reset("new")
    assert reset.session_id == "new"
    assert reset.revision == reset.total_token_count == reset.total_phrase_count == 0
    assert reset.phrase_fingerprints == frozenset()
    assert memory.update(GrammarUpdate((_token(1, GestureKind.FIST, HandKey.LEFT, 100.0),), ())).revision == 1
