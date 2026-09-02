"""Small deterministic Token / Chord / Sequence matcher for Banter Task 2."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import time
from typing import Callable

from .contracts import GestureEvent, GestureKind, HandKey
from .grammar_contracts import (
    ChordRule,
    GesturePhraseEvent,
    GrammarUpdate,
    PhraseForm,
    SequenceRule,
)


def _monotonic_ms() -> float:
    return time.monotonic() * 1_000.0


@dataclass(frozen=True)
class GestureGrammarConfig:
    """Small declarative Token / Chord / wake-sequence vocabulary."""

    chords: tuple[ChordRule, ...] = (
        ChordRule("POWER_VICTORY", GestureKind.FIST, GestureKind.VICTORY_TWO, 450.0),
    )
    sequences: tuple[SequenceRule, ...] = (
        SequenceRule("CHALLENGE_TRIUMPH", (GestureKind.OPEN_PALM_FIVE, GestureKind.FIST, GestureKind.VICTORY_TWO), 2_500.0, 6_000.0),
        SequenceRule("YOU_GOT_IT", (GestureKind.OPEN_PALM_FIVE, GestureKind.POINT_ONE, GestureKind.THUMBS_UP), 2_500.0, 6_000.0),
        SequenceRule("HYPE_CONFIRM", (GestureKind.OPEN_PALM_FIVE, GestureKind.VICTORY_TWO, GestureKind.THUMBS_UP), 2_500.0, 6_000.0),
    )

    def __post_init__(self) -> None:
        if not isinstance(self.chords, tuple) or not isinstance(self.sequences, tuple):
            raise TypeError("chords and sequences must be tuples.")
        names: set[str] = set()
        for rule in (*self.chords, *self.sequences):
            if not isinstance(rule, (ChordRule, SequenceRule)):
                raise TypeError("Grammar rules must be ChordRule or SequenceRule values.")
            if rule.name in names:
                raise ValueError(f"Grammar rule names must be unique: {rule.name!r}")
            names.add(rule.name)


class GestureGrammarEngine:
    """Recognize higher-order phrases without consuming or rewriting Tokens.

    ``update`` is called once for each Task 1 stabilizer output tuple.  Chords
    use close confirmation times from opposite hands.  Sequences are exact
    suffixes of each hand's own recent Token history, so other-hand activity
    cannot reorder or break them.
    """

    def __init__(
        self,
        config: GestureGrammarConfig = GestureGrammarConfig(),
        *,
        clock_ms: Callable[[], float] = _monotonic_ms,
    ) -> None:
        if not isinstance(config, GestureGrammarConfig):
            raise TypeError("config must be a GestureGrammarConfig.")
        if not callable(clock_ms):
            raise TypeError("clock_ms must be callable.")
        self.config = config
        self._clock_ms = clock_ms
        self._recent_for_chords: deque[GestureEvent] = deque()
        self._recent_by_hand = {HandKey.LEFT: deque(), HandKey.RIGHT: deque()}
        self._claimed_chord_event_ids: set[int] = set()
        self._last_event_id = 0
        self._last_confirmed_capture_t_ms: float | None = None
        self._next_phrase_id = 1

    def reset(self) -> None:
        """Discard grammar-local history before a new interaction session."""
        self._recent_for_chords.clear()
        for history in self._recent_by_hand.values():
            history.clear()
        self._claimed_chord_event_ids.clear()
        self._last_event_id = 0
        self._last_confirmed_capture_t_ms = None
        self._next_phrase_id = 1

    def clear_transient_history(self) -> None:
        """Forget unfinished chord/sequence context without resetting IDs.

        An interaction-arm boundary must not let an event from before the
        boundary combine with a later event after it.  Phrase identity and
        input ordering remain monotonic for the surrounding session.
        """
        self._recent_for_chords.clear()
        for history in self._recent_by_hand.values():
            history.clear()
        self._claimed_chord_event_ids.clear()

    def update(self, tokens: tuple[GestureEvent, ...]) -> GrammarUpdate:
        """Accept one ordered Task 1 batch and derive any newly completed phrases."""
        self._validate_batch(tokens)
        candidates: list[tuple[str, PhraseForm, tuple[GestureEvent, ...]]] = []
        for token in tokens:
            self._prune_chord_history(token.confirmed_capture_t_ms)
            self._recent_for_chords.append(token)
            chord = self._match_chord_for(token)
            if chord is not None:
                candidates.append(chord)

            self._append_sequence_token(token)
            candidates.extend(self._match_sequences_for(token))

        if tokens:
            self._last_event_id = tokens[-1].event_id
            self._last_confirmed_capture_t_ms = tokens[-1].confirmed_capture_t_ms
        if not candidates:
            return GrammarUpdate(tokens=tokens, phrases=())

        candidates.sort(
            key=lambda item: (
                max(event.confirmed_capture_t_ms for event in item[2]),
                0 if item[1] is PhraseForm.CHORD else 1,
                item[0],
                tuple(event.event_id for event in item[2]),
            )
        )
        available_t_ms = float(self._clock_ms())
        if not math.isfinite(available_t_ms) or available_t_ms <= 0.0:
            raise ValueError("Grammar clock must return a finite positive timestamp in milliseconds.")
        phrases = tuple(
            GesturePhraseEvent(
                phrase_id=self._next_phrase_id + index,
                name=name,
                form=form,
                source_events=source_events,
                available_t_ms=available_t_ms,
            )
            for index, (name, form, source_events) in enumerate(candidates)
        )
        self._next_phrase_id += len(phrases)
        return GrammarUpdate(tokens=tokens, phrases=phrases)

    def _validate_batch(self, tokens: object) -> None:
        if not isinstance(tokens, tuple):
            raise TypeError("GestureGrammarEngine.update requires the Task 1 event tuple.")
        if len(tokens) > 2:
            raise ValueError("One Task 1 event batch contains at most two Tokens.")
        previous_id = self._last_event_id
        previous_capture = self._last_confirmed_capture_t_ms
        for token in tokens:
            if not isinstance(token, GestureEvent):
                raise TypeError("tokens must contain GestureEvent values only.")
            if token.event_id <= previous_id:
                raise ValueError("Gesture Grammar requires strictly increasing event_id values.")
            if previous_capture is not None and token.confirmed_capture_t_ms < previous_capture:
                raise ValueError("Gesture Grammar requires non-decreasing confirmed capture timestamps.")
            previous_id = token.event_id
            previous_capture = token.confirmed_capture_t_ms

    def _prune_chord_history(self, now_capture_t_ms: float) -> None:
        if not self.config.chords:
            self._recent_for_chords.clear()
            self._claimed_chord_event_ids.clear()
            return
        horizon = max(rule.max_confirm_delta_ms for rule in self.config.chords)
        cutoff = now_capture_t_ms - horizon
        while self._recent_for_chords and self._recent_for_chords[0].confirmed_capture_t_ms < cutoff:
            self._recent_for_chords.popleft()
        live_ids = {event.event_id for event in self._recent_for_chords}
        self._claimed_chord_event_ids.intersection_update(live_ids)

    def _match_chord_for(
        self, token: GestureEvent
    ) -> tuple[str, PhraseForm, tuple[GestureEvent, ...]] | None:
        if token.event_id in self._claimed_chord_event_ids:
            return None
        for rule in self.config.chords:
            counterpart = self._closest_chord_counterpart(token, rule)
            if counterpart is None:
                continue
            left, right = (
                (token, counterpart)
                if token.hand_key is HandKey.LEFT
                else (counterpart, token)
            )
            self._claimed_chord_event_ids.update({left.event_id, right.event_id})
            # Source tuple order is canonical provenance order, while the
            # ChordRule and each event's hand_key retain LEFT/RIGHT semantics.
            return rule.name, PhraseForm.CHORD, tuple(sorted((left, right), key=lambda event: event.event_id))
        return None

    def _closest_chord_counterpart(
        self, token: GestureEvent, rule: ChordRule
    ) -> GestureEvent | None:
        if token.hand_key is HandKey.LEFT:
            required_hand, required_gesture = HandKey.RIGHT, rule.right_gesture
            if token.gesture is not rule.left_gesture:
                return None
        else:
            required_hand, required_gesture = HandKey.LEFT, rule.left_gesture
            if token.gesture is not rule.right_gesture:
                return None
        candidates = [
            event
            for event in self._recent_for_chords
            if event.event_id != token.event_id
            and event.event_id not in self._claimed_chord_event_ids
            and event.hand_key is required_hand
            and event.gesture is required_gesture
            and abs(event.confirmed_capture_t_ms - token.confirmed_capture_t_ms)
            <= rule.max_confirm_delta_ms
        ]
        return min(
            candidates,
            key=lambda event: (
                abs(event.confirmed_capture_t_ms - token.confirmed_capture_t_ms),
                event.event_id,
            ),
            default=None,
        )

    def _append_sequence_token(self, token: GestureEvent) -> None:
        history = self._recent_by_hand[token.hand_key]
        history.append(token)
        if not self.config.sequences:
            history.clear()
            return
        maximum_total = max(rule.max_total_ms for rule in self.config.sequences)
        cutoff = token.confirmed_capture_t_ms - maximum_total
        while history and history[0].confirmed_capture_t_ms < cutoff:
            history.popleft()
        maximum_length = max(len(rule.gestures) for rule in self.config.sequences)
        while len(history) > maximum_length:
            history.popleft()

    def _match_sequences_for(
        self, token: GestureEvent
    ) -> tuple[tuple[str, PhraseForm, tuple[GestureEvent, ...]], ...]:
        history = tuple(self._recent_by_hand[token.hand_key])
        matches: list[tuple[str, PhraseForm, tuple[GestureEvent, ...]]] = []
        for rule in self.config.sequences:
            if len(history) < len(rule.gestures):
                continue
            source_events = history[-len(rule.gestures):]
            if tuple(event.gesture for event in source_events) != rule.gestures:
                continue
            gaps = [
                later.confirmed_capture_t_ms - earlier.confirmed_capture_t_ms
                for earlier, later in zip(source_events, source_events[1:])
            ]
            total = source_events[-1].confirmed_capture_t_ms - source_events[0].confirmed_capture_t_ms
            if all(gap <= rule.max_step_gap_ms for gap in gaps) and total <= rule.max_total_ms:
                matches.append((rule.name, PhraseForm.SEQUENCE, source_events))
        return tuple(matches)
