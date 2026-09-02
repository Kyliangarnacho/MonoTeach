"""Immutable Task 2 contracts for derived Gesture Grammar phrases."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real

from .contracts import GestureEvent, GestureKind


GRAMMAR_SCHEMA_VERSION = "banter_gesture_grammar_v1"


class PhraseForm(str, Enum):
    """The two higher-order expressions deliberately supported in Task 2."""

    CHORD = "CHORD"
    SEQUENCE = "SEQUENCE"


def _positive_ms(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a finite positive duration in milliseconds.")
    numeric = float(value)
    if not math.isfinite(numeric) or numeric <= 0.0:
        raise ValueError(f"{name} must be a finite positive duration in milliseconds.")
    return numeric


def _rule_name(value: object, name: str = "name") -> str:
    if not isinstance(value, str) or not value or not value.isidentifier() or value != value.upper():
        raise ValueError(f"{name} must be a non-empty uppercase identifier.")
    return value


def _known_gesture(value: object, name: str) -> GestureKind:
    if not isinstance(value, GestureKind) or value is GestureKind.UNKNOWN:
        raise ValueError(f"{name} must be a known GestureKind.")
    return value


@dataclass(frozen=True)
class ChordRule:
    """One exact left/right near-simultaneous gesture pair."""

    name: str
    left_gesture: GestureKind
    right_gesture: GestureKind
    max_confirm_delta_ms: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "name", _rule_name(self.name))
        _known_gesture(self.left_gesture, "left_gesture")
        _known_gesture(self.right_gesture, "right_gesture")
        object.__setattr__(
            self,
            "max_confirm_delta_ms",
            _positive_ms(self.max_confirm_delta_ms, "max_confirm_delta_ms"),
        )


@dataclass(frozen=True)
class SequenceRule:
    """One exact, contiguous, same-hand token sequence."""

    name: str
    gestures: tuple[GestureKind, ...]
    max_step_gap_ms: float
    max_total_ms: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "name", _rule_name(self.name))
        if not isinstance(self.gestures, tuple) or len(self.gestures) < 2:
            raise ValueError("gestures must be a tuple containing at least two known GestureKinds.")
        for index, gesture in enumerate(self.gestures):
            _known_gesture(gesture, f"gestures[{index}]")
        step_gap = _positive_ms(self.max_step_gap_ms, "max_step_gap_ms")
        total = _positive_ms(self.max_total_ms, "max_total_ms")
        if total < step_gap:
            raise ValueError("max_total_ms must be at least max_step_gap_ms.")
        object.__setattr__(self, "max_step_gap_ms", step_gap)
        object.__setattr__(self, "max_total_ms", total)


@dataclass(frozen=True)
class GesturePhraseEvent:
    """A derived Chord or Sequence fact with complete Task 1 provenance."""

    phrase_id: int
    name: str
    form: PhraseForm
    source_events: tuple[GestureEvent, ...]
    available_t_ms: float
    schema_version: str = GRAMMAR_SCHEMA_VERSION

    def __post_init__(self) -> None:
        if not isinstance(self.phrase_id, int) or isinstance(self.phrase_id, bool) or self.phrase_id < 1:
            raise ValueError("phrase_id must be a positive integer.")
        object.__setattr__(self, "name", _rule_name(self.name))
        if not isinstance(self.form, PhraseForm):
            raise TypeError("form must be a PhraseForm.")
        if not isinstance(self.source_events, tuple):
            raise TypeError("source_events must be a tuple of GestureEvent values.")
        minimum_sources = 2 if self.form is PhraseForm.CHORD else 2
        if len(self.source_events) < minimum_sources:
            raise ValueError("A GesturePhraseEvent requires at least two source events.")
        source_ids: set[int] = set()
        for event in self.source_events:
            if not isinstance(event, GestureEvent):
                raise TypeError("source_events must contain GestureEvent values only.")
            if event.event_id in source_ids:
                raise ValueError("source_events must not repeat an event_id.")
            source_ids.add(event.event_id)
        if self.form is PhraseForm.CHORD and len(self.source_events) != 2:
            raise ValueError("A CHORD must have exactly two source events.")
        if self.form is PhraseForm.CHORD:
            object.__setattr__(self, "source_events", tuple(sorted(self.source_events, key=lambda event: event.event_id)))
        elif any(
            later.event_id <= earlier.event_id
            for earlier, later in zip(self.source_events, self.source_events[1:])
        ):
            raise ValueError("A SEQUENCE must retain strictly increasing source event ids.")
        available = _positive_ms(self.available_t_ms, "available_t_ms")
        if available < max(event.available_t_ms for event in self.source_events):
            raise ValueError("Phrase available_t_ms must not precede source-event availability.")
        if self.schema_version != GRAMMAR_SCHEMA_VERSION:
            raise ValueError(f"Unsupported GesturePhraseEvent schema_version: {self.schema_version!r}")
        object.__setattr__(self, "available_t_ms", available)

    @property
    def started_capture_t_ms(self) -> float:
        return min(event.confirmed_capture_t_ms for event in self.source_events)

    @property
    def completed_capture_t_ms(self) -> float:
        return max(event.confirmed_capture_t_ms for event in self.source_events)

    @property
    def source_event_ids(self) -> tuple[int, ...]:
        return tuple(event.event_id for event in self.source_events)

    def to_dict(self) -> dict[str, object]:
        """Return a JSON-safe phrase record without raw landmarks or frames."""
        return {
            "schema_version": self.schema_version,
            "phrase_id": self.phrase_id,
            "name": self.name,
            "form": self.form.value,
            "source_event_ids": self.source_event_ids,
            "source_tokens": [
                {
                    "event_id": event.event_id,
                    "gesture": event.gesture.value,
                    "hand_key": event.hand_key.value,
                    "confirmed_capture_t_ms": event.confirmed_capture_t_ms,
                }
                for event in self.source_events
            ],
            "started_capture_t_ms": self.started_capture_t_ms,
            "completed_capture_t_ms": self.completed_capture_t_ms,
            "available_t_ms": self.available_t_ms,
        }


@dataclass(frozen=True)
class GrammarUpdate:
    """One atomic grammar turn: original Tokens plus derived Phrase facts."""

    tokens: tuple[GestureEvent, ...]
    phrases: tuple[GesturePhraseEvent, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.tokens, tuple) or not isinstance(self.phrases, tuple):
            raise TypeError("tokens and phrases must both be tuples.")
        for token in self.tokens:
            if not isinstance(token, GestureEvent):
                raise TypeError("tokens must contain GestureEvent values only.")
        for phrase in self.phrases:
            if not isinstance(phrase, GesturePhraseEvent):
                raise TypeError("phrases must contain GesturePhraseEvent values only.")
