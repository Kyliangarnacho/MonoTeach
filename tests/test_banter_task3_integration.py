"""Synthetic Task 1 → Task 2 → Task 3 interaction-context integration."""

from __future__ import annotations

from banter.contracts import GestureEvidence, GestureKind, GesturePerception, HandFrame, HandKey
from banter.gesture_events import GestureEventConfig, GestureEventStabilizer
from banter.gesture_grammar import GestureGrammarEngine
from banter.grammar_contracts import GrammarUpdate, PhraseForm
from banter.interaction_memory import InteractionMemory
from stage2.hand_observation import HandObservation


def _perception(
    frame_index: int, capture_t_ms: float, items: tuple[tuple[HandKey, GestureKind], ...]
) -> GesturePerception:
    hands = tuple(
        HandObservation(
            timestamp_ms=capture_t_ms,
            detected=True,
            handedness=hand_key.value.title(),
            handedness_score=0.9,
            landmarks_norm=tuple((0.0, 0.0, 0.0) for _ in range(21)),
            index_tip_norm=(0.0, 0.0),
            index_tip_px=(0, 0),
        )
        for hand_key, _gesture in items
    )
    evidence = tuple(
        GestureEvidence(
            frame_index=frame_index,
            capture_t_ms=capture_t_ms,
            available_t_ms=capture_t_ms + 3.0,
            hand=hand,
            hand_key=hand_key,
            gesture=gesture,
            recognizer_score=None if gesture is GestureKind.UNKNOWN else 0.9,
        )
        for hand, (hand_key, gesture) in zip(hands, items)
    )
    return GesturePerception(HandFrame(frame_index, capture_t_ms, capture_t_ms + 3.0, hands), evidence)


def test_task1_task2_task3_repeat_timeout_and_phrase_story() -> None:
    stabilizer = GestureEventStabilizer(
        GestureEventConfig(dwell_ms=50.0, release_ms=10.0, dropout_grace_ms=10.0)
    )
    grammar = GestureGrammarEngine(clock_ms=lambda: 10_000.0)
    memory = InteractionMemory("synthetic-story", clock_ms=lambda: 20_000.0)

    frame_index = 0

    def feed(t_ms: float, items: tuple[tuple[HandKey, GestureKind], ...]):
        nonlocal frame_index
        tokens = stabilizer.update(_perception(frame_index, t_ms, items))
        frame_index += 1
        return memory.update(grammar.update(tokens))

    left_fist = ((HandKey.LEFT, GestureKind.FIST),)
    left_unknown = ((HandKey.LEFT, GestureKind.UNKNOWN),)
    feed(0.0, left_fist)
    first = feed(60.0, left_fist)
    feed(70.0, left_unknown)
    feed(90.0, left_unknown)
    feed(100.0, left_fist)
    second = feed(160.0, left_fist)
    feed(170.0, left_unknown)
    feed(190.0, left_unknown)
    left_palm = ((HandKey.LEFT, GestureKind.OPEN_PALM_FIVE),)
    feed(200.0, left_palm)
    palm = feed(260.0, left_palm)
    feed(270.0, left_unknown)
    feed(290.0, left_unknown)
    feed(6_000.0, left_fist)
    timed_out_fist = feed(6_060.0, left_fist)

    assert first.token_total(HandKey.LEFT, GestureKind.FIST) == 1
    assert second.token_total(HandKey.LEFT, GestureKind.FIST) == 2
    assert palm.token_total(HandKey.LEFT, GestureKind.FIST) == 2
    assert palm.left_token_streak is not None
    assert palm.left_token_streak.key.gesture is GestureKind.OPEN_PALM_FIVE
    assert timed_out_fist.token_total(HandKey.LEFT, GestureKind.FIST) == 3
    assert timed_out_fist.left_token_streak is not None and timed_out_fist.left_token_streak.count == 1

    both_unknown = ((HandKey.LEFT, GestureKind.UNKNOWN), (HandKey.RIGHT, GestureKind.UNKNOWN))
    chord = ((HandKey.LEFT, GestureKind.FIST), (HandKey.RIGHT, GestureKind.VICTORY_TWO))
    feed(6_070.0, both_unknown)
    feed(6_090.0, both_unknown)
    feed(6_200.0, chord)
    first_phrase = feed(6_260.0, chord)
    feed(6_270.0, both_unknown)
    feed(6_290.0, both_unknown)
    feed(6_400.0, chord)
    second_phrase = feed(6_460.0, chord)

    assert first_phrase.phrase_total(PhraseForm.CHORD, "POWER_VICTORY") == 1
    assert second_phrase.phrase_total(PhraseForm.CHORD, "POWER_VICTORY") == 2
    assert second_phrase.phrase_streak is not None and second_phrase.phrase_streak.count == 2
    assert second_phrase.total_phrase_count == 2
    assert second_phrase.interaction_turn_count > second_phrase.total_phrase_count
