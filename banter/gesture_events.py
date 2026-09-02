"""Timestamp-based, per-hand conversion of evidence into one-shot events."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .contracts import (
    GestureEvidence,
    GestureEvent,
    GestureKind,
    GesturePerception,
    HandKey,
)


@dataclass(frozen=True)
class GestureEventConfig:
    """Conservative temporal and score gates for a deliberate static pose."""

    enter_score: float = 0.65
    hold_score: float = 0.50
    dwell_ms: float = 400.0
    release_ms: float = 280.0
    dropout_grace_ms: float = 180.0

    def __post_init__(self) -> None:
        enter, hold = float(self.enter_score), float(self.hold_score)
        if not (0.0 < hold < enter <= 1.0):
            raise ValueError("Gesture score thresholds must satisfy 0 < hold < enter <= 1.")
        for name in ("dwell_ms", "release_ms", "dropout_grace_ms"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive.")
            object.__setattr__(self, name, value)
        object.__setattr__(self, "enter_score", enter)
        object.__setattr__(self, "hold_score", hold)


@dataclass
class _HandState:
    candidate: GestureKind | None = None
    candidate_started_t_ms: float | None = None
    candidate_paused_ms: float = 0.0
    candidate_dropout_since_t_ms: float | None = None
    active: GestureKind | None = None
    active_started_t_ms: float | None = None
    active_dropout_since_t_ms: float | None = None
    release_since_t_ms: float | None = None

    def clear_candidate(self) -> None:
        self.candidate = None
        self.candidate_started_t_ms = None
        self.candidate_paused_ms = 0.0
        self.candidate_dropout_since_t_ms = None

    def clear_active(self) -> None:
        self.active = None
        self.active_started_t_ms = None
        self.active_dropout_since_t_ms = None
        self.release_since_t_ms = None


class GestureEventStabilizer:
    """Publish exactly one event per deliberately held gesture, per hand.

    Evidence with an unknown/ambiguous hand side is intentionally ignored.  It
    remains visible to a demo, but cannot be safely used by later two-hand
    Grammar.
    """

    def __init__(self, config: GestureEventConfig = GestureEventConfig()) -> None:
        if not isinstance(config, GestureEventConfig):
            raise TypeError("config must be a GestureEventConfig.")
        self.config = config
        self._states = {HandKey.LEFT: _HandState(), HandKey.RIGHT: _HandState()}
        self._event_id = 0
        self._last_capture_t_ms: float | None = None

    @property
    def active_gestures(self) -> dict[HandKey, GestureKind | None]:
        """Return a snapshot of Task 1's already-stabilized per-hand poses.

        This deliberately exposes no landmarks or recognition internals.  A
        later control-only gate can use the same dwell/release/dropout result
        as GestureEvent publication instead of duplicating temporal policy.
        """
        return {hand_key: self._states[hand_key].active for hand_key in (HandKey.LEFT, HandKey.RIGHT)}

    def update(self, perception: GesturePerception) -> tuple[GestureEvent, ...]:
        """Consume one strictly later frame and emit zero, one, or two events."""
        if not isinstance(perception, GesturePerception):
            raise TypeError("perception must be a GesturePerception.")
        now_ms = perception.frame.capture_t_ms
        if self._last_capture_t_ms is not None and now_ms <= self._last_capture_t_ms:
            raise ValueError("GestureEventStabilizer requires strictly increasing capture timestamps.")
        self._last_capture_t_ms = now_ms
        per_side: dict[HandKey, GestureEvidence] = {
            item.hand_key: item
            for item in perception.evidence
            if item.hand_key in {HandKey.LEFT, HandKey.RIGHT}
        }
        events: list[GestureEvent] = []
        for hand_key in (HandKey.LEFT, HandKey.RIGHT):
            event = self._update_hand(
                hand_key,
                per_side.get(hand_key),
                len(perception.frame.hands),
                perception.frame.frame_index,
                now_ms,
                perception.frame.available_t_ms,
            )
            if event is not None:
                events.append(event)
        return tuple(events)

    def _update_hand(
        self,
        hand_key: HandKey,
        evidence: GestureEvidence | None,
        observed_hand_count: int,
        frame_index: int,
        now_ms: float,
        available_t_ms: float,
    ) -> GestureEvent | None:
        state = self._states[hand_key]
        if state.active is not None:
            if self._active_matches(state, evidence):
                state.active_dropout_since_t_ms = None
                state.release_since_t_ms = None
                return None
            if evidence is None:
                return self._advance_active_dropout(
                    state, hand_key, evidence, observed_hand_count, frame_index, now_ms, available_t_ms
                )
            # A visible different/low-score pose begins a real release.  It
            # must remain non-active for release_ms before rearming.
            if state.release_since_t_ms is None:
                state.release_since_t_ms = now_ms
                return None
            if now_ms - state.release_since_t_ms < self.config.release_ms:
                return None
            state.clear_active()
            return self._begin_candidate_if_eligible(
                state, hand_key, evidence, observed_hand_count, frame_index, now_ms, available_t_ms
            )

        return self._advance_candidate(
            state, hand_key, evidence, observed_hand_count, frame_index, now_ms, available_t_ms
        )

    def _active_matches(self, state: _HandState, evidence: GestureEvidence | None) -> bool:
        return bool(
            evidence is not None
            and evidence.gesture is state.active
            and evidence.recognizer_score is not None
            and evidence.recognizer_score >= self.config.hold_score
        )

    def _advance_active_dropout(
        self,
        state: _HandState,
        hand_key: HandKey,
        evidence: GestureEvidence | None,
        observed_hand_count: int,
        frame_index: int,
        now_ms: float,
        available_t_ms: float,
    ) -> GestureEvent | None:
        if state.active_dropout_since_t_ms is None:
            state.active_dropout_since_t_ms = now_ms
            return None
        elapsed = now_ms - state.active_dropout_since_t_ms
        if elapsed <= self.config.dropout_grace_ms:
            return None
        if state.release_since_t_ms is None:
            state.release_since_t_ms = state.active_dropout_since_t_ms + self.config.dropout_grace_ms
        if now_ms - state.release_since_t_ms < self.config.release_ms:
            return None
        state.clear_active()
        return self._begin_candidate_if_eligible(
            state, hand_key, evidence, observed_hand_count, frame_index, now_ms, available_t_ms
        )

    def _advance_candidate(
        self,
        state: _HandState,
        hand_key: HandKey,
        evidence: GestureEvidence | None,
        observed_hand_count: int,
        frame_index: int,
        now_ms: float,
        available_t_ms: float,
    ) -> GestureEvent | None:
        if evidence is None or evidence.gesture is GestureKind.UNKNOWN:
            if state.candidate is None:
                return None
            if state.candidate_dropout_since_t_ms is None:
                state.candidate_dropout_since_t_ms = now_ms
                return None
            if now_ms - state.candidate_dropout_since_t_ms <= self.config.dropout_grace_ms:
                return None
            state.clear_candidate()
            return None

        if state.candidate is None:
            return self._begin_candidate_if_eligible(
                state, hand_key, evidence, observed_hand_count, frame_index, now_ms, available_t_ms
            )

        if evidence.gesture is not state.candidate or evidence.recognizer_score is None or evidence.recognizer_score < self.config.hold_score:
            state.clear_candidate()
            return self._begin_candidate_if_eligible(
                state, hand_key, evidence, observed_hand_count, frame_index, now_ms, available_t_ms
            )

        if state.candidate_dropout_since_t_ms is not None:
            state.candidate_paused_ms += now_ms - state.candidate_dropout_since_t_ms
            state.candidate_dropout_since_t_ms = None
        assert state.candidate_started_t_ms is not None
        elapsed = now_ms - state.candidate_started_t_ms - state.candidate_paused_ms
        if elapsed < self.config.dwell_ms:
            return None
        return self._confirm_candidate(
            state, hand_key, evidence, observed_hand_count, frame_index, now_ms, available_t_ms
        )

    def _begin_candidate_if_eligible(
        self,
        state: _HandState,
        hand_key: HandKey,
        evidence: GestureEvidence | None,
        observed_hand_count: int,
        frame_index: int,
        now_ms: float,
        available_t_ms: float,
    ) -> GestureEvent | None:
        if (
            evidence is None
            or evidence.gesture is GestureKind.UNKNOWN
            or evidence.recognizer_score is None
            or evidence.recognizer_score < self.config.enter_score
        ):
            return None
        state.candidate = evidence.gesture
        state.candidate_started_t_ms = now_ms
        state.candidate_paused_ms = 0.0
        state.candidate_dropout_since_t_ms = None
        # A zero dwell config is forbidden, so the initial evidence never
        # publishes immediately; it must survive at least one later frame.
        return None

    def _confirm_candidate(
        self,
        state: _HandState,
        hand_key: HandKey,
        evidence: GestureEvidence,
        observed_hand_count: int,
        frame_index: int,
        now_ms: float,
        available_t_ms: float,
    ) -> GestureEvent:
        assert state.candidate is not None and state.candidate_started_t_ms is not None
        visible_dwell_ms = (
            now_ms - state.candidate_started_t_ms - state.candidate_paused_ms
        )
        self._event_id += 1
        event = GestureEvent(
            event_id=self._event_id,
            gesture=state.candidate,
            hand_key=hand_key,
            source_frame_index=frame_index,
            evidence_started_capture_t_ms=state.candidate_started_t_ms,
            confirmed_capture_t_ms=now_ms,
            available_t_ms=available_t_ms,
            dwell_ms=visible_dwell_ms,
            recognizer_score=evidence.recognizer_score,
            observed_hand_count=observed_hand_count,
            source=evidence.source,
        )
        state.active = state.candidate
        state.active_started_t_ms = state.candidate_started_t_ms
        state.clear_candidate()
        return event
