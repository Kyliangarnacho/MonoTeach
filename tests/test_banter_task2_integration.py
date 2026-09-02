"""Task 1 stabilizer to Task 2 Grammar integration without a camera."""

from __future__ import annotations

from banter.contracts import GestureEvidence, GestureKind, GesturePerception, HandFrame, HandKey
from banter.gesture_events import GestureEventConfig, GestureEventStabilizer
from banter.gesture_grammar import GestureGrammarEngine
from stage2.hand_observation import HandObservation


def _perception(
    frame_index: int,
    timestamp_ms: float,
    items: tuple[tuple[HandKey, GestureKind], ...],
) -> GesturePerception:
    hands = tuple(
        HandObservation(
            timestamp_ms=timestamp_ms,
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
            capture_t_ms=timestamp_ms,
            available_t_ms=timestamp_ms + 3.0,
            hand=hand,
            hand_key=hand_key,
            gesture=gesture,
            recognizer_score=None if gesture is GestureKind.UNKNOWN else 0.9,
        )
        for hand, (hand_key, gesture) in zip(hands, items)
    )
    return GesturePerception(
        frame=HandFrame(frame_index, timestamp_ms, timestamp_ms + 3.0, hands),
        evidence=evidence,
    )


def _stabilizer() -> GestureEventStabilizer:
    return GestureEventStabilizer(
        GestureEventConfig(
            enter_score=0.65,
            hold_score=0.50,
            dwell_ms=50.0,
            release_ms=10.0,
            dropout_grace_ms=10.0,
        )
    )


def test_task1_event_batches_drive_task2_chord_and_sequence_without_landmarks() -> None:
    chord_stabilizer = _stabilizer()
    grammar = GestureGrammarEngine(clock_ms=lambda: 10_000.0)
    chord_items = ((HandKey.LEFT, GestureKind.FIST), (HandKey.RIGHT, GestureKind.VICTORY_TWO))
    assert chord_stabilizer.update(_perception(0, 0.0, chord_items)) == ()
    chord_tokens = chord_stabilizer.update(_perception(1, 60.0, chord_items))
    chord = grammar.update(chord_tokens)
    assert [phrase.name for phrase in chord.phrases] == ["POWER_VICTORY"]

    sequence_stabilizer = _stabilizer()
    grammar = GestureGrammarEngine(clock_ms=lambda: 10_000.0)
    events = []
    for frame_index, timestamp_ms, gesture in (
            (0, 0.0, GestureKind.OPEN_PALM_FIVE),
            (1, 60.0, GestureKind.OPEN_PALM_FIVE),
            (2, 70.0, GestureKind.FIST),
            (3, 85.0, GestureKind.FIST),
            (4, 135.0, GestureKind.FIST),
        (5, 145.0, GestureKind.VICTORY_TWO),
        (6, 160.0, GestureKind.VICTORY_TWO),
        (7, 210.0, GestureKind.VICTORY_TWO),
    ):
        tokens = sequence_stabilizer.update(
            _perception(frame_index, timestamp_ms, ((HandKey.LEFT, gesture),))
        )
        events.extend(grammar.update(tokens).phrases)
    assert [phrase.name for phrase in events] == ["CHALLENGE_TRIUMPH"]


def test_rejected_landmark_gesture_never_becomes_a_task2_token() -> None:
    stabilizer = _stabilizer()
    grammar = GestureGrammarEngine(clock_ms=lambda: 10_000.0)
    rejected = ((HandKey.LEFT, GestureKind.UNKNOWN),)
    assert stabilizer.update(_perception(0, 0.0, rejected)) == ()
    assert grammar.update(stabilizer.update(_perception(1, 60.0, rejected))).tokens == ()
