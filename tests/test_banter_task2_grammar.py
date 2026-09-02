"""Deterministic Chord and wake-sequence tests for Banter Task 2."""

from __future__ import annotations

import pytest

from banter.contracts import GestureEvent, GestureKind, HandKey
from banter.gesture_grammar import GestureGrammarConfig, GestureGrammarEngine
from banter.grammar_contracts import PhraseForm, SequenceRule


def _event(event_id: int, gesture: GestureKind, hand_key: HandKey, t_ms: float) -> GestureEvent:
    return GestureEvent(event_id, gesture, hand_key, event_id, max(0.0, t_ms - 50.0), t_ms, t_ms + 5.0,
                        min(50.0, t_ms), 0.9, 1)


def _engine() -> GestureGrammarEngine:
    return GestureGrammarEngine(clock_ms=lambda: 10_000.0)


def test_default_sequences_are_open_wake_rules_with_natural_windows() -> None:
    rules = GestureGrammarConfig().sequences
    assert [rule.name for rule in rules] == ["CHALLENGE_TRIUMPH", "YOU_GOT_IT", "HYPE_CONFIRM"]
    assert all(rule.gestures[0] is GestureKind.OPEN_PALM_FIVE for rule in rules)
    assert all(rule.max_step_gap_ms == 2_500.0 and rule.max_total_ms == 6_000.0 for rule in rules)


def test_chord_matches_and_canonicalizes_both_confirmation_orders() -> None:
    left_first = _engine().update((_event(39, GestureKind.FIST, HandKey.LEFT, 100.0),
                                   _event(40, GestureKind.VICTORY_TWO, HandKey.RIGHT, 120.0)))
    assert left_first.phrases[0].form is PhraseForm.CHORD
    assert left_first.phrases[0].source_event_ids == (39, 40)

    engine = _engine()
    assert engine.update((_event(39, GestureKind.VICTORY_TWO, HandKey.RIGHT, 100.0),)).phrases == ()
    right_first = engine.update((_event(40, GestureKind.FIST, HandKey.LEFT, 120.0),))
    assert right_first.phrases[0].source_event_ids == (39, 40)


@pytest.mark.parametrize(
    ("gestures", "name"),
    (
        ((GestureKind.OPEN_PALM_FIVE, GestureKind.FIST, GestureKind.VICTORY_TWO), "CHALLENGE_TRIUMPH"),
        ((GestureKind.OPEN_PALM_FIVE, GestureKind.POINT_ONE, GestureKind.THUMBS_UP), "YOU_GOT_IT"),
        ((GestureKind.OPEN_PALM_FIVE, GestureKind.VICTORY_TWO, GestureKind.THUMBS_UP), "HYPE_CONFIRM"),
    ),
)
def test_each_default_wake_sequence_matches_exact_same_hand_suffix(gestures, name: str) -> None:
    engine = _engine()
    for event_id, gesture in enumerate(gestures[:-1], start=1):
        assert engine.update((_event(event_id, gesture, HandKey.LEFT, event_id * 900.0),)).phrases == ()
    update = engine.update((_event(3, gestures[-1], HandKey.LEFT, 2_700.0),))
    assert [phrase.name for phrase in update.phrases] == [name]
    assert update.phrases[0].source_event_ids == (1, 2, 3)


def test_wrong_order_timeout_and_custom_rule_remain_deterministic() -> None:
    engine = _engine()
    for event_id, gesture, t_ms in ((1, GestureKind.OPEN_PALM_FIVE, 0.0), (2, GestureKind.VICTORY_TWO, 500.0),
                                    (3, GestureKind.FIST, 800.0)):
        assert engine.update((_event(event_id, gesture, HandKey.RIGHT, t_ms),)).phrases == ()
    slow = _engine()
    slow.update((_event(1, GestureKind.OPEN_PALM_FIVE, HandKey.LEFT, 0.0),))
    slow.update((_event(2, GestureKind.FIST, HandKey.LEFT, 2_501.0),))
    assert slow.update((_event(3, GestureKind.VICTORY_TWO, HandKey.LEFT, 2_700.0),)).phrases == ()

    config = GestureGrammarConfig(chords=(), sequences=(SequenceRule("DOUBLE_POINT", (GestureKind.POINT_ONE, GestureKind.POINT_ONE), 100.0, 200.0),))
    custom = GestureGrammarEngine(config, clock_ms=lambda: 1_000.0)
    assert custom.update((_event(1, GestureKind.POINT_ONE, HandKey.LEFT, 100.0),)).phrases == ()
    assert custom.update((_event(2, GestureKind.POINT_ONE, HandKey.LEFT, 150.0),)).phrases[0].name == "DOUBLE_POINT"
