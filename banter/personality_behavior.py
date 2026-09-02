"""Task 4 declarative appraisal, prefix buffering, and non-preemptive behavior admission."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
import time
from typing import Callable

from .contracts import GestureEvent, GestureKind, HandKey
from .gesture_grammar import GestureGrammarConfig
from .grammar_contracts import GesturePhraseEvent, PhraseForm, SequenceRule
from .interaction_memory import empty_snapshot
from .memory_contracts import InteractionMemorySnapshot, PhraseMemoryKey
from .personality_contracts import (
    AffectDelta,
    BehaviorAdmission,
    BehaviorEvent,
    BehaviorRule,
    BehaviorVariantRule,
    PersonaConfig,
    PersonaState,
    Task4Update,
)


def _monotonic_ms() -> float:
    return time.monotonic() * 1_000.0


def _clamp(value: float) -> float:
    return min(1.0, max(0.0, value))


@dataclass(frozen=True)
class _MemoryDelta:
    tokens: tuple[GestureEvent, ...]
    phrases: tuple[GesturePhraseEvent, ...]


@dataclass(frozen=True)
class _Candidate:
    rule: BehaviorRule
    source_tokens: tuple[GestureEvent, ...]
    source_phrases: tuple[GesturePhraseEvent, ...]
    completed_capture_t_ms: float
    streak_count: int

    @property
    def token_ids(self) -> tuple[int, ...]:
        return tuple(sorted({item.event_id for item in self.source_tokens}))

    @property
    def phrase_ids(self) -> tuple[int, ...]:
        return tuple(sorted(item.phrase_id for item in self.source_phrases))


@dataclass(frozen=True)
class _PendingChord:
    token: GestureEvent
    deadline_t_ms: float


@dataclass(frozen=True)
class _PendingSequence:
    tokens: tuple[GestureEvent, ...]
    deadline_t_ms: float


def empty_persona_state(session_id: str, config: PersonaConfig = PersonaConfig()) -> PersonaState:
    """Build the explicit baseline affect for a named Task 4 session."""
    if not isinstance(config, PersonaConfig):
        raise TypeError("config must be a PersonaConfig.")
    profile = config.profile
    return PersonaState(session_id, 0, 0, profile.baseline_friendliness, profile.baseline_playfulness,
                        profile.baseline_energy, profile.baseline_annoyance, None, None)


def appraise_persona(
    previous_persona: PersonaState,
    previous_memory: InteractionMemorySnapshot,
    current_memory: InteractionMemorySnapshot,
    config: PersonaConfig = PersonaConfig(),
) -> PersonaState:
    """Purely fold one adjacent Memory revision through per-stimulus affect rules."""
    if not isinstance(previous_persona, PersonaState) or not isinstance(config, PersonaConfig):
        raise TypeError("previous_persona and config must be PersonaState and PersonaConfig values.")
    delta = _memory_delta(previous_memory, current_memory)
    if (previous_persona.session_id != previous_memory.session_id
            or previous_persona.source_memory_revision != previous_memory.revision):
        raise ValueError("PersonaState must be aligned with the preceding Memory snapshot.")
    capture_t_ms = current_memory.last_interaction_capture_t_ms
    if capture_t_ms is None:
        raise ValueError("A changed Memory snapshot must have an interaction capture timestamp.")
    prior_capture = previous_persona.last_interaction_capture_t_ms
    elapsed = 0.0 if prior_capture is None else max(0.0, capture_t_ms - prior_capture)
    decay = math.pow(0.5, elapsed / config.affect_half_life_ms)
    profile = config.profile
    values = {
        "friendliness": _relax(previous_persona.friendliness, profile.baseline_friendliness, decay),
        "playfulness": _relax(previous_persona.playfulness, profile.baseline_playfulness, decay),
        "energy": _relax(previous_persona.energy, profile.baseline_energy, decay),
        "annoyance": _relax(previous_persona.annoyance, profile.baseline_annoyance, decay),
    }
    repeated: set[tuple[HandKey, GestureKind]] = set()
    for token in delta.tokens:
        rule = config.rule_for(token.gesture)
        if rule is None:
            continue
        _add_delta(values, rule.affect_delta)
        if rule.repeat_streak_threshold is not None:
            streak = current_memory.current_token_streak(token.hand_key)
            if (streak is not None and streak.key.gesture is token.gesture
                    and streak.count >= rule.repeat_streak_threshold):
                repeated.add((token.hand_key, token.gesture))
    for phrase in delta.phrases:
        rule = config.rule_for(PhraseMemoryKey(phrase.form, phrase.name))
        if rule is not None:
            _add_delta(values, rule.affect_delta)
    for hand_key, gesture in repeated:
        rule = config.rule_for(gesture)
        assert rule is not None
        _add_delta(values, rule.repeat_affect_delta)
        del hand_key
    return PersonaState(
        current_memory.session_id,
        current_memory.revision,
        previous_persona.persona_revision + 1,
        _clamp(values["friendliness"]), _clamp(values["playfulness"]),
        _clamp(values["energy"]), _clamp(values["annoyance"]),
        capture_t_ms, current_memory.last_input_available_t_ms, current_memory.available_t_ms,
    )


def select_behavior(
    persona: PersonaState,
    current_memory: InteractionMemorySnapshot,
    previous_memory: InteractionMemorySnapshot,
    config: PersonaConfig = PersonaConfig(),
    *,
    behavior_id: int,
) -> BehaviorEvent:
    """Pure raw-candidate arbitration without prefix buffering or busy admission.

    The stateful engine is the production entry point. This helper remains
    useful for deterministic unit tests of the declarative response table.
    """
    if persona.source_memory_revision != current_memory.revision:
        raise ValueError("PersonaState must be aligned with current Memory.")
    candidates = _direct_candidates(_memory_delta(previous_memory, current_memory), current_memory, config)
    if not candidates:
        raise ValueError("No configured BehaviorRule matches the Memory delta.")
    winner = _choose_candidate(candidates, persona)
    return BehaviorEvent(behavior_id, persona.session_id, persona.source_memory_revision,
                         winner.rule.behavior_name, _variant_for(winner, persona), winner.token_ids,
                         winner.phrase_ids, winner.completed_capture_t_ms, persona.available_t_ms)


class PersonaBehaviorEngine:
    """Own Task 4 routing state only; Memory remains the canonical interaction record."""

    def __init__(
        self,
        session_id: str = "banter-session-001",
        config: PersonaConfig = PersonaConfig(),
        grammar_config: GestureGrammarConfig = GestureGrammarConfig(),
        *,
        clock_ms: Callable[[], float] = _monotonic_ms,
    ) -> None:
        if not isinstance(config, PersonaConfig) or not isinstance(grammar_config, GestureGrammarConfig):
            raise TypeError("config and grammar_config must be PersonaConfig and GestureGrammarConfig values.")
        if not callable(clock_ms):
            raise TypeError("clock_ms must be callable.")
        if any(rule.gestures[0] is not config.sequence_wake_gesture for rule in grammar_config.sequences):
            raise ValueError("Task 4 sequence rules must start with PersonaConfig.sequence_wake_gesture.")
        self.config = config
        self.grammar_config = grammar_config
        self._clock_ms = clock_ms
        self._previous_memory = empty_snapshot(session_id)
        self._persona = empty_persona_state(session_id, config)
        self._last_update = Task4Update(self._persona, None)
        self._next_behavior_id = 1
        self._pending_chords: dict[int, _PendingChord] = {}
        self._pending_sequences: dict[HandKey, _PendingSequence] = {}
        self._consumed_token_ids: set[int] = set()
        self._active_until_capture_t_ms = 0.0
        self._external_active_behavior_id: int | None = None
        self._last_capture_t_ms = 0.0

    @property
    def persona(self) -> PersonaState:
        return self._persona

    @property
    def last_update(self) -> Task4Update:
        return self._last_update

    def update(self, memory: InteractionMemorySnapshot) -> Task4Update:
        """Process one new Memory revision; same revision remains idempotent."""
        if not isinstance(memory, InteractionMemorySnapshot):
            raise TypeError("memory must be an InteractionMemorySnapshot.")
        if memory.session_id != self._previous_memory.session_id:
            raise ValueError("PersonaBehaviorEngine requires an explicit reset for a new session.")
        if memory.revision == self._previous_memory.revision:
            return self._last_update
        delta = _memory_delta(self._previous_memory, memory)
        draft_persona = appraise_persona(self._persona, self._previous_memory, memory, self.config)
        now_capture = memory.last_interaction_capture_t_ms
        if now_capture is None:
            raise ValueError("Changed Memory must carry interaction capture time.")
        self._last_capture_t_ms = max(self._last_capture_t_ms, now_capture)
        candidates, deferred = self._route_delta(delta, memory, now_capture)
        update = self._finalize(draft_persona, candidates, now_capture,
                                BehaviorAdmission.DEFERRED_PREFIX if deferred else BehaviorAdmission.NO_CANDIDATE)
        self._previous_memory = memory
        self._persona = update.persona
        self._last_update = update
        return update

    def advance_time(self, capture_t_ms: float) -> Task4Update:
        """Release expired chord/sequence prefixes on a camera frame with no new Token."""
        now = float(capture_t_ms)
        if not math.isfinite(now) or now < self._last_capture_t_ms:
            raise ValueError("advance_time capture_t_ms must be finite and non-decreasing.")
        self._last_capture_t_ms = now
        candidates = self._expire_pending(now)
        if not candidates:
            return self._last_update
        update = self._finalize(self._persona, candidates, now, BehaviorAdmission.NO_CANDIDATE)
        self._persona = update.persona
        self._last_update = update
        return update

    def reset(self, session_id: str) -> Task4Update:
        """Explicitly start a new named persona session at its profile baseline."""
        self._previous_memory = empty_snapshot(session_id)
        self._persona = empty_persona_state(session_id, self.config)
        self._last_update = Task4Update(self._persona, None)
        self._next_behavior_id = 1
        self._pending_chords.clear()
        self._pending_sequences.clear()
        self._consumed_token_ids.clear()
        self._active_until_capture_t_ms = 0.0
        self._external_active_behavior_id = None
        self._last_capture_t_ms = 0.0
        return self._last_update

    @property
    def external_active_behavior_id(self) -> int | None:
        """The live executor-owned busy lock, if Task 7 has acquired one."""
        return self._external_active_behavior_id

    def acquire_execution_lock(self, behavior_id: int) -> None:
        """Atomically mark the just-emitted behavior as executor-owned busy."""
        if not isinstance(behavior_id, int) or isinstance(behavior_id, bool) or behavior_id < 1:
            raise ValueError("behavior_id must be a positive integer.")
        emitted = self._last_update.behavior
        if emitted is None or emitted.behavior_id != behavior_id:
            raise ValueError("Execution lock must be acquired for the just-emitted BehaviorEvent.")
        if self._external_active_behavior_id is not None:
            raise RuntimeError("An execution lock is already active.")
        self._external_active_behavior_id = behavior_id

    def release_execution_lock(self, behavior_id: int) -> None:
        """Release only the matching executor completion and clear nominal busy."""
        if behavior_id != self._external_active_behavior_id:
            raise ValueError("Execution completion does not match the active BehaviorEvent.")
        self._external_active_behavior_id = None
        # In live mode actual executor completion is authoritative, including
        # the valid case where it precedes Task 4's nominal duration.
        self._active_until_capture_t_ms = 0.0

    def clear_transient_history(self) -> None:
        """Discard deferred phrase prefixes at an ARM/DISARM boundary only."""
        self._pending_chords.clear()
        self._pending_sequences.clear()

    def _route_delta(self, delta: _MemoryDelta, memory: InteractionMemorySnapshot, now: float) -> tuple[list[_Candidate], bool]:
        candidates: list[_Candidate] = []
        phrase_candidates = [self._candidate_for_phrase(item, memory) for item in delta.phrases]
        phrase_candidates = [item for item in phrase_candidates if item is not None]
        claimed = {event.event_id for candidate in phrase_candidates for event in candidate.source_tokens}
        for event_id in claimed:
            self._pending_chords.pop(event_id, None)
        for hand_key, pending in tuple(self._pending_sequences.items()):
            if {item.event_id for item in pending.tokens}.intersection(claimed):
                del self._pending_sequences[hand_key]
        candidates.extend(phrase_candidates)
        candidates.extend(self._expire_pending(now))
        deferred = bool(self._pending_chords or self._pending_sequences)
        for token in delta.tokens:
            if token.event_id in claimed:
                continue
            routed, token_deferred = self._route_token(token, memory)
            candidates.extend(routed)
            deferred = deferred or token_deferred
        return candidates, deferred

    def _route_token(self, token: GestureEvent, memory: InteractionMemorySnapshot) -> tuple[list[_Candidate], bool]:
        pending = self._pending_sequences.get(token.hand_key)
        if pending is not None:
            buffered = pending.tokens + (token,)
            rules = self._sequence_prefix_rules(buffered)
            if rules:
                self._pending_sequences[token.hand_key] = _PendingSequence(buffered, _sequence_deadline(buffered, rules))
                return [], True
            del self._pending_sequences[token.hand_key]
            candidates: list[_Candidate] = []
            for buffered_token in buffered:
                candidate = self._candidate_for_token(buffered_token, memory)
                if candidate is not None:
                    candidates.append(candidate)
            return candidates, False
        rules = self._sequence_prefix_rules((token,))
        if rules:
            self._pending_sequences[token.hand_key] = _PendingSequence((token,), _sequence_deadline((token,), rules))
            return [], True
        candidate = self._candidate_for_token(token, memory)
        if candidate is None:
            return [], False
        deadline = self._chord_deadline(token)
        if deadline is not None:
            self._pending_chords[token.event_id] = _PendingChord(token, deadline)
            return [], True
        return [candidate], False

    def _expire_pending(self, now: float) -> list[_Candidate]:
        candidates: list[_Candidate] = []
        for hand_key, pending in tuple(self._pending_sequences.items()):
            if now > pending.deadline_t_ms:
                del self._pending_sequences[hand_key]
                for token in pending.tokens:
                    candidate = self._candidate_for_token(token, self._previous_memory)
                    if candidate is not None:
                        candidates.append(candidate)
        for event_id, pending in tuple(self._pending_chords.items()):
            if now > pending.deadline_t_ms:
                del self._pending_chords[event_id]
                candidate = self._candidate_for_token(pending.token, self._previous_memory)
                if candidate is not None:
                    candidates.append(candidate)
        return candidates

    def _candidate_for_token(self, token: GestureEvent, memory: InteractionMemorySnapshot) -> _Candidate | None:
        rule = self.config.rule_for(token.gesture)
        if rule is None:
            return None
        streak = memory.current_token_streak(token.hand_key)
        count = streak.count if streak is not None and streak.key.gesture is token.gesture else 1
        return _Candidate(rule, (token,), (), token.confirmed_capture_t_ms, count)

    def _candidate_for_phrase(self, phrase: GesturePhraseEvent, memory: InteractionMemorySnapshot) -> _Candidate | None:
        rule = self.config.rule_for(PhraseMemoryKey(phrase.form, phrase.name))
        if rule is None:
            return None
        streak = memory.current_phrase_streak()
        count = streak.count if streak is not None and streak.key == PhraseMemoryKey(phrase.form, phrase.name) else 1
        return _Candidate(rule, phrase.source_events, (phrase,), phrase.completed_capture_t_ms, count)

    def _sequence_prefix_rules(self, tokens: tuple[GestureEvent, ...]) -> tuple[SequenceRule, ...]:
        gestures = tuple(item.gesture for item in tokens)
        matches: list[SequenceRule] = []
        for rule in self.grammar_config.sequences:
            if len(gestures) >= len(rule.gestures) or rule.gestures[:len(gestures)] != gestures:
                continue
            gaps = [later.confirmed_capture_t_ms - earlier.confirmed_capture_t_ms for earlier, later in zip(tokens, tokens[1:])]
            total = tokens[-1].confirmed_capture_t_ms - tokens[0].confirmed_capture_t_ms
            if all(gap <= rule.max_step_gap_ms for gap in gaps) and total <= rule.max_total_ms:
                matches.append(rule)
        return tuple(matches)

    def _chord_deadline(self, token: GestureEvent) -> float | None:
        windows: list[float] = []
        for rule in self.grammar_config.chords:
            if ((token.hand_key is HandKey.LEFT and token.gesture is rule.left_gesture)
                    or (token.hand_key is HandKey.RIGHT and token.gesture is rule.right_gesture)):
                windows.append(rule.max_confirm_delta_ms)
        return None if not windows else token.confirmed_capture_t_ms + max(windows)

    def _finalize(self, persona: PersonaState, candidates: list[_Candidate], now: float,
                  fallback: BehaviorAdmission) -> Task4Update:
        eligible = [item for item in candidates if not set(item.token_ids).intersection(self._consumed_token_ids)]
        if not eligible:
            return self._publish(persona, None, fallback)
        winner = _choose_candidate(eligible, persona)
        all_token_ids = {token_id for item in eligible for token_id in item.token_ids}
        self._consumed_token_ids.update(all_token_ids)
        if self._external_active_behavior_id is not None or now < self._active_until_capture_t_ms:
            return self._publish(persona, None, BehaviorAdmission.IGNORED_BUSY)
        behavior = BehaviorEvent(
            self._next_behavior_id, persona.session_id, persona.source_memory_revision,
            winner.rule.behavior_name, _variant_for(winner, persona), winner.token_ids, winner.phrase_ids,
            winner.completed_capture_t_ms, persona.available_t_ms,
        )
        self._next_behavior_id += 1
        self._active_until_capture_t_ms = now + winner.rule.nominal_duration_ms
        return self._publish(persona, behavior, BehaviorAdmission.EMITTED)

    def _publish(self, persona: PersonaState, behavior: BehaviorEvent | None,
                 admission: BehaviorAdmission) -> Task4Update:
        available = float(self._clock_ms())
        if not math.isfinite(available) or available <= 0.0:
            raise ValueError("Persona clock must return a finite positive timestamp in milliseconds.")
        if available < persona.available_t_ms:
            raise ValueError("Persona availability must not precede accepted Memory availability.")
        published_persona = replace(persona, available_t_ms=available)
        published_behavior = None if behavior is None else replace(behavior, available_t_ms=available)
        return Task4Update(published_persona, published_behavior, admission)


def _memory_delta(previous: InteractionMemorySnapshot, current: InteractionMemorySnapshot) -> _MemoryDelta:
    if not isinstance(previous, InteractionMemorySnapshot) or not isinstance(current, InteractionMemorySnapshot):
        raise TypeError("previous and current must be InteractionMemorySnapshot values.")
    if current.session_id != previous.session_id:
        raise ValueError("Task 4 cannot compare Memory snapshots from different sessions.")
    if current.revision != previous.revision + 1 or current.interaction_turn_count != previous.interaction_turn_count + 1:
        raise ValueError("Task 4 requires exactly one adjacent Memory revision.")
    tokens = tuple(item for item in current.recent_tokens if item.event_id > previous.last_token_event_id)
    phrases = tuple(item for item in current.recent_phrases if item.phrase_id > previous.last_phrase_id)
    if current.total_token_count - previous.total_token_count != len(tokens):
        raise ValueError("Task 4 requires retained Token provenance for its adjacent Memory revision.")
    if current.total_phrase_count - previous.total_phrase_count != len(phrases):
        raise ValueError("Task 4 requires retained Phrase provenance for its adjacent Memory revision.")
    if not tokens and not phrases:
        raise ValueError("A changed Memory revision must contain a new Token or Phrase fact.")
    return _MemoryDelta(tokens, phrases)


def _direct_candidates(delta: _MemoryDelta, memory: InteractionMemorySnapshot, config: PersonaConfig) -> list[_Candidate]:
    phrases = []
    claimed: set[int] = set()
    for phrase in delta.phrases:
        rule = config.rule_for(PhraseMemoryKey(phrase.form, phrase.name))
        if rule is None:
            continue
        streak = memory.current_phrase_streak()
        count = streak.count if streak is not None and streak.key == PhraseMemoryKey(phrase.form, phrase.name) else 1
        candidate = _Candidate(rule, phrase.source_events, (phrase,), phrase.completed_capture_t_ms, count)
        phrases.append(candidate)
        claimed.update(candidate.token_ids)
    tokens = []
    for token in delta.tokens:
        if token.event_id in claimed:
            continue
        rule = config.rule_for(token.gesture)
        if rule is not None:
            streak = memory.current_token_streak(token.hand_key)
            count = streak.count if streak is not None and streak.key.gesture is token.gesture else 1
            tokens.append(_Candidate(rule, (token,), (), token.confirmed_capture_t_ms, count))
    return phrases + tokens


def _choose_candidate(candidates: list[_Candidate], persona: PersonaState) -> _Candidate:
    return max(candidates, key=lambda item: (item.rule.priority, item.completed_capture_t_ms,
                                              item.rule.behavior_name, _variant_for(item, persona)))


def _variant_for(candidate: _Candidate, persona: PersonaState) -> str:
    matching = [
        item for item in candidate.rule.variants
        if (persona.friendliness >= item.min_friendliness and persona.playfulness >= item.min_playfulness
            and persona.energy >= item.min_energy and persona.annoyance >= item.min_annoyance
            and candidate.streak_count >= item.min_streak_count)
    ]
    return max(matching, key=lambda item: (item.priority, item.variant)).variant if matching else candidate.rule.base_variant


def _sequence_deadline(tokens: tuple[GestureEvent, ...], rules: tuple[SequenceRule, ...]) -> float:
    first, last = tokens[0].confirmed_capture_t_ms, tokens[-1].confirmed_capture_t_ms
    return max(min(last + rule.max_step_gap_ms, first + rule.max_total_ms) for rule in rules)


def _relax(value: float, baseline: float, decay: float) -> float:
    return baseline + (value - baseline) * decay


def _add_delta(values: dict[str, float], delta: AffectDelta) -> None:
    values["friendliness"] += delta.friendliness
    values["playfulness"] += delta.playfulness
    values["energy"] += delta.energy
    values["annoyance"] += delta.annoyance
