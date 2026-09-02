"""Deterministic Task 3 fold plus stateful post-fold availability publisher."""

from __future__ import annotations

from dataclasses import replace
import math
import time
from typing import Callable

from .contracts import GestureEvent, HandKey
from .grammar_contracts import GesturePhraseEvent, GrammarUpdate
from .memory_contracts import (
    InteractionMemoryConfig,
    InteractionMemorySnapshot,
    PhraseCountEntry,
    PhraseFingerprint,
    PhraseMemoryKey,
    PhraseStreak,
    TokenCountEntry,
    TokenMemoryKey,
    TokenStreak,
)


def _monotonic_ms() -> float:
    return time.monotonic() * 1_000.0


def empty_snapshot(session_id: str) -> InteractionMemorySnapshot:
    """Create the explicit empty state for a new local interaction session."""
    return InteractionMemorySnapshot(session_id=session_id)


def fold_interaction_memory(
    previous: InteractionMemorySnapshot,
    update: GrammarUpdate,
    config: InteractionMemoryConfig,
) -> InteractionMemorySnapshot:
    """Purely fold a complete Task 2 batch without sampling a clock.

    A non-empty result intentionally keeps ``available_t_ms`` at its upstream
    input availability. The outer ``InteractionMemory`` samples a post-fold
    clock before publishing it. Empty updates return the same object by
    identity.
    """
    if not isinstance(previous, InteractionMemorySnapshot):
        raise TypeError("previous must be an InteractionMemorySnapshot.")
    if not isinstance(update, GrammarUpdate):
        raise TypeError("update must be a GrammarUpdate.")
    if not isinstance(config, InteractionMemoryConfig):
        raise TypeError("config must be an InteractionMemoryConfig.")
    if not update.tokens and not update.phrases:
        return previous
    _validate_update(previous, update)

    token_counts = {entry.key: entry.count for entry in previous.token_counts}
    phrase_counts = {entry.key: entry.count for entry in previous.phrase_counts}
    left_streak = previous.left_token_streak
    right_streak = previous.right_token_streak
    phrase_streak = previous.phrase_streak
    for token in update.tokens:
        key = TokenMemoryKey(token.hand_key, token.gesture)
        token_counts[key] = token_counts.get(key, 0) + 1
        current = left_streak if token.hand_key is HandKey.LEFT else right_streak
        next_streak = _next_token_streak(current, key, token.confirmed_capture_t_ms, config.token_streak_gap_ms)
        if token.hand_key is HandKey.LEFT:
            left_streak = next_streak
        else:
            right_streak = next_streak
    for phrase in update.phrases:
        key = PhraseMemoryKey(phrase.form, phrase.name)
        phrase_counts[key] = phrase_counts.get(key, 0) + 1
        phrase_streak = _next_phrase_streak(
            phrase_streak, key, phrase.completed_capture_t_ms, config.phrase_streak_gap_ms
        )

    recent_tokens = (previous.recent_tokens + update.tokens)[-config.recent_token_capacity:]
    recent_phrases = (previous.recent_phrases + update.phrases)[-config.recent_phrase_capacity:]
    input_available_t_ms = max(item.available_t_ms for item in (*update.tokens, *update.phrases))
    last_capture_t_ms = max(
        previous.last_interaction_capture_t_ms or 0.0,
        *[item.confirmed_capture_t_ms for item in update.tokens],
        *[item.completed_capture_t_ms for item in update.phrases],
    )
    return InteractionMemorySnapshot(
        session_id=previous.session_id,
        revision=previous.revision + 1,
        interaction_turn_count=previous.interaction_turn_count + 1,
        total_token_count=previous.total_token_count + len(update.tokens),
        total_phrase_count=previous.total_phrase_count + len(update.phrases),
        recent_tokens=recent_tokens,
        recent_phrases=recent_phrases,
        token_counts=tuple(
            TokenCountEntry(key, count) for key, count in sorted(token_counts.items(), key=_token_key_sort)
        ),
        phrase_counts=tuple(
            PhraseCountEntry(key, count) for key, count in sorted(phrase_counts.items(), key=_phrase_key_sort)
        ),
        left_token_streak=left_streak,
        right_token_streak=right_streak,
        phrase_streak=phrase_streak,
        last_interaction_capture_t_ms=last_capture_t_ms,
        last_input_available_t_ms=input_available_t_ms,
        available_t_ms=input_available_t_ms,
        last_token_event_id=update.tokens[-1].event_id if update.tokens else previous.last_token_event_id,
        last_phrase_id=update.phrases[-1].phrase_id if update.phrases else previous.last_phrase_id,
        phrase_fingerprints=previous.phrase_fingerprints.union(
            _phrase_fingerprint(phrase) for phrase in update.phrases
        ),
    )


class InteractionMemory:
    """Stateful owner that publishes post-fold immutable snapshots.

    It contains no timing policy: all order, aggregation, and streak decisions
    live in the pure fold and use source capture time. The injected monotonic
    clock only stamps when the already-computed snapshot became available.
    """

    def __init__(
        self,
        session_id: str = "banter-session-001",
        config: InteractionMemoryConfig = InteractionMemoryConfig(),
        *,
        clock_ms: Callable[[], float] = _monotonic_ms,
    ) -> None:
        if not isinstance(config, InteractionMemoryConfig):
            raise TypeError("config must be an InteractionMemoryConfig.")
        if not callable(clock_ms):
            raise TypeError("clock_ms must be callable.")
        self.config = config
        self._clock_ms = clock_ms
        self._snapshot = empty_snapshot(session_id)

    @property
    def snapshot(self) -> InteractionMemorySnapshot:
        return self._snapshot

    def update(self, update: GrammarUpdate) -> InteractionMemorySnapshot:
        """Validate/fold atomically, then sample post-fold availability once."""
        draft = fold_interaction_memory(self._snapshot, update, self.config)
        if draft is self._snapshot:
            return draft
        available_t_ms = float(self._clock_ms())
        if not math.isfinite(available_t_ms) or available_t_ms <= 0.0:
            raise ValueError("Memory clock must return a finite positive timestamp in milliseconds.")
        if available_t_ms < draft.last_input_available_t_ms:
            raise ValueError("Memory availability must not precede accepted input availability.")
        self._snapshot = replace(draft, available_t_ms=available_t_ms)
        return self._snapshot

    def reset(self, session_id: str) -> InteractionMemorySnapshot:
        """Explicitly discard all local history and begin a named new session."""
        self._snapshot = empty_snapshot(session_id)
        return self._snapshot


def _validate_update(previous: InteractionMemorySnapshot, update: GrammarUpdate) -> None:
    previous_token_id = previous.last_token_event_id
    previous_capture_t_ms = previous.last_interaction_capture_t_ms
    current_tokens: dict[int, GestureEvent] = {}
    for token in update.tokens:
        if token.event_id <= previous_token_id:
            raise ValueError("Interaction Memory requires strictly increasing Token event_id values.")
        if previous_capture_t_ms is not None and token.confirmed_capture_t_ms < previous_capture_t_ms:
            raise ValueError("Interaction Memory requires non-decreasing Token confirmation capture times.")
        previous_token_id = token.event_id
        previous_capture_t_ms = token.confirmed_capture_t_ms
        current_tokens[token.event_id] = token

    previous_phrase_id = previous.last_phrase_id
    known_tokens = {token.event_id: token for token in previous.recent_tokens}
    known_tokens.update(current_tokens)
    fingerprints = set(previous.phrase_fingerprints)
    previous_phrase_capture_t_ms: float | None = None
    for phrase in update.phrases:
        if phrase.phrase_id <= previous_phrase_id:
            raise ValueError("Interaction Memory requires strictly increasing Phrase phrase_id values.")
        if (
            previous_phrase_capture_t_ms is not None
            and phrase.completed_capture_t_ms < previous_phrase_capture_t_ms
        ):
            raise ValueError("Interaction Memory requires non-decreasing Phrase completion capture times.")
        previous_phrase_id = phrase.phrase_id
        previous_phrase_capture_t_ms = phrase.completed_capture_t_ms
        for source in phrase.source_events:
            known = known_tokens.get(source.event_id)
            if known is None:
                raise ValueError("A Phrase source event must be retained or supplied in the same GrammarUpdate.")
            if known != source:
                raise ValueError("A Phrase source event id cannot refer to different Token content.")
        fingerprint = _phrase_fingerprint(phrase)
        if fingerprint in fingerprints:
            raise ValueError("Interaction Memory rejects duplicate Phrase provenance.")
        fingerprints.add(fingerprint)


def _next_token_streak(
    current: TokenStreak | None, key: TokenMemoryKey, capture_t_ms: float, gap_ms: float
) -> TokenStreak:
    if current is not None and current.key == key and capture_t_ms - current.last_capture_t_ms <= gap_ms:
        return TokenStreak(key, current.count + 1, current.started_capture_t_ms, capture_t_ms)
    return TokenStreak(key, 1, capture_t_ms, capture_t_ms)


def _next_phrase_streak(
    current: PhraseStreak | None, key: PhraseMemoryKey, capture_t_ms: float, gap_ms: float
) -> PhraseStreak:
    if current is not None and current.key == key and capture_t_ms - current.last_capture_t_ms <= gap_ms:
        return PhraseStreak(key, current.count + 1, current.started_capture_t_ms, capture_t_ms)
    return PhraseStreak(key, 1, capture_t_ms, capture_t_ms)


def _phrase_fingerprint(phrase: GesturePhraseEvent) -> PhraseFingerprint:
    return PhraseFingerprint(PhraseMemoryKey(phrase.form, phrase.name), phrase.source_event_ids)


def _token_key_sort(item: tuple[TokenMemoryKey, int]) -> tuple[str, str]:
    return item[0].hand_key.value, item[0].gesture.value


def _phrase_key_sort(item: tuple[PhraseMemoryKey, int]) -> tuple[str, str]:
    return item[0].form.value, item[0].name
