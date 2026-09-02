"""Immutable Task 4 contracts for declarative persona-to-behavior decisions."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real

from .contracts import GestureKind
from .grammar_contracts import PhraseForm
from .memory_contracts import PhraseMemoryKey


PERSONA_SCHEMA_VERSION = "banter_persona_v2"
BEHAVIOR_SCHEMA_VERSION = "banter_behavior_event_v2"


def _finite(value: object, name: str, *, low: float, high: float) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a finite number.")
    numeric = float(value)
    if not math.isfinite(numeric) or not low <= numeric <= high:
        raise ValueError(f"{name} must be between {low} and {high}.")
    return numeric


def _positive_ms(value: object, name: str) -> float:
    numeric = _finite(value, name, low=0.0, high=float("inf"))
    if numeric <= 0.0:
        raise ValueError(f"{name} must be a finite positive duration in milliseconds.")
    return numeric


def _nonnegative_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer.")
    return value


def _positive_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 1:
        raise ValueError(f"{name} must be a positive integer.")
    return value


def _identifier(value: object, name: str) -> str:
    if not isinstance(value, str) or not value or not value.isidentifier() or value != value.upper():
        raise ValueError(f"{name} must be a non-empty uppercase identifier.")
    return value


def _session_id(value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError("session_id must be a non-empty string.")
    return value


@dataclass(frozen=True)
class AffectDelta:
    """A bounded deterministic addition to the four live persona axes."""

    friendliness: float = 0.0
    playfulness: float = 0.0
    energy: float = 0.0
    annoyance: float = 0.0

    def __post_init__(self) -> None:
        for name in ("friendliness", "playfulness", "energy", "annoyance"):
            object.__setattr__(self, name, _finite(getattr(self, name), name, low=-1.0, high=1.0))


@dataclass(frozen=True)
class PersonaProfile:
    """Stable character baseline, deliberately separate from live affect."""

    profile_id: str = "MONO_DEFAULT"
    baseline_friendliness: float = 0.55
    baseline_playfulness: float = 0.45
    baseline_energy: float = 0.50
    baseline_annoyance: float = 0.05

    def __post_init__(self) -> None:
        _identifier(self.profile_id, "profile_id")
        for name in ("baseline_friendliness", "baseline_playfulness", "baseline_energy", "baseline_annoyance"):
            object.__setattr__(self, name, _finite(getattr(self, name), name, low=0.0, high=1.0))


BehaviorStimulus = GestureKind | PhraseMemoryKey

# Variants are intentionally a tiny escalation vocabulary, not per-action
# adjectives.  ``DEFAULT`` means that the behavior's own Macro is sufficient;
# only an explicitly configured high-emotion behavior may select an upgrade.
BEHAVIOR_VARIANT_VOCABULARY = frozenset({"DEFAULT", "FIRM", "FORCEFUL"})


@dataclass(frozen=True)
class BehaviorVariantRule:
    """One discrete named response variant selected by persona/streak thresholds."""

    variant: str
    priority: int = 0
    min_friendliness: float = 0.0
    min_playfulness: float = 0.0
    min_energy: float = 0.0
    min_annoyance: float = 0.0
    min_streak_count: int = 1

    def __post_init__(self) -> None:
        _identifier(self.variant, "variant")
        if self.variant not in BEHAVIOR_VARIANT_VOCABULARY:
            raise ValueError(
                f"variant must be one of {sorted(BEHAVIOR_VARIANT_VOCABULARY)!r}."
            )
        if not isinstance(self.priority, int) or isinstance(self.priority, bool):
            raise TypeError("priority must be an integer.")
        for name in ("min_friendliness", "min_playfulness", "min_energy", "min_annoyance"):
            object.__setattr__(self, name, _finite(getattr(self, name), name, low=0.0, high=1.0))
        _positive_int(self.min_streak_count, "min_streak_count")


@dataclass(frozen=True)
class BehaviorRule:
    """One Token or Phrase's independent behavior, affect, and occupancy policy."""

    stimulus: BehaviorStimulus
    behavior_name: str
    base_variant: str
    affect_delta: AffectDelta = AffectDelta()
    priority: int = 0
    nominal_duration_ms: float = 800.0
    variants: tuple[BehaviorVariantRule, ...] = ()
    repeat_streak_threshold: int | None = None
    repeat_affect_delta: AffectDelta = AffectDelta()

    def __post_init__(self) -> None:
        if isinstance(self.stimulus, GestureKind):
            if self.stimulus is GestureKind.UNKNOWN:
                raise ValueError("BehaviorRule cannot target UNKNOWN GestureKind.")
        elif not isinstance(self.stimulus, PhraseMemoryKey):
            raise TypeError("stimulus must be a known GestureKind or PhraseMemoryKey.")
        _identifier(self.behavior_name, "behavior_name")
        _identifier(self.base_variant, "base_variant")
        if self.base_variant not in BEHAVIOR_VARIANT_VOCABULARY:
            raise ValueError(
                f"base_variant must be one of {sorted(BEHAVIOR_VARIANT_VOCABULARY)!r}."
            )
        if not isinstance(self.affect_delta, AffectDelta) or not isinstance(self.repeat_affect_delta, AffectDelta):
            raise TypeError("affect deltas must be AffectDelta values.")
        if not isinstance(self.priority, int) or isinstance(self.priority, bool):
            raise TypeError("priority must be an integer.")
        object.__setattr__(self, "nominal_duration_ms", _positive_ms(self.nominal_duration_ms, "nominal_duration_ms"))
        if not isinstance(self.variants, tuple) or not all(isinstance(item, BehaviorVariantRule) for item in self.variants):
            raise TypeError("variants must be a tuple of BehaviorVariantRule values.")
        if len({item.variant for item in self.variants}) != len(self.variants):
            raise ValueError("variants must not repeat a variant name.")
        if self.repeat_streak_threshold is not None:
            _positive_int(self.repeat_streak_threshold, "repeat_streak_threshold")


class BehaviorAdmission(str, Enum):
    """Why this Task 4 publication did or did not emit a behavior."""

    EMITTED = "EMITTED"
    DEFERRED_PREFIX = "DEFERRED_PREFIX"
    IGNORED_BUSY = "IGNORED_BUSY"
    NO_CANDIDATE = "NO_CANDIDATE"


@dataclass(frozen=True)
class PersonaConfig:
    """Declarative response table; Token and Phrase have equal rule status."""

    profile: PersonaProfile = PersonaProfile()
    affect_half_life_ms: float = 15_000.0
    sequence_wake_gesture: GestureKind = GestureKind.OPEN_PALM_FIVE
    response_rules: tuple[BehaviorRule, ...] = (
        BehaviorRule(GestureKind.POINT_ONE, "POINT_BACK", "DEFAULT", AffectDelta(playfulness=0.03), 30, 700.0),
        BehaviorRule(GestureKind.VICTORY_TWO, "VICTORY_SALUTE", "DEFAULT", AffectDelta(friendliness=0.05, playfulness=0.10, energy=0.06), 40, 800.0),
        BehaviorRule(GestureKind.OPEN_PALM_FIVE, "OPEN_HAND_WAVE", "DEFAULT", AffectDelta(friendliness=0.04), 25, 700.0),
        BehaviorRule(GestureKind.THUMBS_UP, "THUMBS_UP_ACK", "DEFAULT", AffectDelta(friendliness=0.14, energy=0.04), 45, 750.0),
        BehaviorRule(GestureKind.FIST, "FIST_COUNTER", "DEFAULT", AffectDelta(annoyance=0.05), 50, 900.0,
                     (BehaviorVariantRule("FIRM", 1, min_streak_count=2),
                      BehaviorVariantRule("FORCEFUL", 2, min_annoyance=0.25, min_streak_count=3)),
                     repeat_streak_threshold=3, repeat_affect_delta=AffectDelta(annoyance=0.12, playfulness=0.03)),
        BehaviorRule(GestureKind.MIDDLE_FINGER, "OFFENDED_RETORT", "DEFAULT", AffectDelta(friendliness=-0.08, annoyance=0.24), 70, 1_000.0),
        BehaviorRule(PhraseMemoryKey(PhraseForm.CHORD, "POWER_VICTORY"), "POWER_POSE_REPLY", "DEFAULT", AffectDelta(playfulness=0.18, energy=0.16), 80, 1_100.0),
        BehaviorRule(PhraseMemoryKey(PhraseForm.SEQUENCE, "CHALLENGE_TRIUMPH"), "TRIUMPH_FLOURISH", "DEFAULT", AffectDelta(friendliness=0.05, playfulness=0.20, energy=0.18), 85, 1_200.0),
        BehaviorRule(PhraseMemoryKey(PhraseForm.SEQUENCE, "YOU_GOT_IT"), "PROUD_APPROVAL", "DEFAULT", AffectDelta(friendliness=0.15, playfulness=0.06), 75, 950.0),
        BehaviorRule(PhraseMemoryKey(PhraseForm.SEQUENCE, "HYPE_CONFIRM"), "HYPE_BOUNCE", "DEFAULT", AffectDelta(playfulness=0.16, energy=0.14), 78, 1_000.0),
    )

    def __post_init__(self) -> None:
        if not isinstance(self.profile, PersonaProfile):
            raise TypeError("profile must be a PersonaProfile.")
        object.__setattr__(self, "affect_half_life_ms", _positive_ms(self.affect_half_life_ms, "affect_half_life_ms"))
        if not isinstance(self.sequence_wake_gesture, GestureKind) or self.sequence_wake_gesture is GestureKind.UNKNOWN:
            raise ValueError("sequence_wake_gesture must be a known GestureKind.")
        if not isinstance(self.response_rules, tuple) or not all(isinstance(item, BehaviorRule) for item in self.response_rules):
            raise TypeError("response_rules must be a tuple of BehaviorRule values.")
        if len({item.stimulus for item in self.response_rules}) != len(self.response_rules):
            raise ValueError("response_rules must not repeat a stimulus.")

    def rule_for(self, stimulus: BehaviorStimulus) -> BehaviorRule | None:
        """Return one configured behavior rule without assigning Token/Phrase polarity."""
        return next((rule for rule in self.response_rules if rule.stimulus == stimulus), None)


@dataclass(frozen=True)
class PersonaState:
    """Live, bounded affect state derived only from canonical Memory facts."""

    session_id: str
    source_memory_revision: int
    persona_revision: int
    friendliness: float
    playfulness: float
    energy: float
    annoyance: float
    last_interaction_capture_t_ms: float | None
    last_input_available_t_ms: float | None
    available_t_ms: float = 0.0
    schema_version: str = PERSONA_SCHEMA_VERSION

    def __post_init__(self) -> None:
        object.__setattr__(self, "session_id", _session_id(self.session_id))
        _nonnegative_int(self.source_memory_revision, "source_memory_revision")
        _nonnegative_int(self.persona_revision, "persona_revision")
        for name in ("friendliness", "playfulness", "energy", "annoyance"):
            object.__setattr__(self, name, _finite(getattr(self, name), name, low=0.0, high=1.0))
        available = _finite(self.available_t_ms, "available_t_ms", low=0.0, high=float("inf"))
        for name in ("last_interaction_capture_t_ms", "last_input_available_t_ms"):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, _finite(value, name, low=0.0, high=float("inf")))
        if self.last_input_available_t_ms is not None and available < self.last_input_available_t_ms:
            raise ValueError("Persona availability must not precede its Memory input availability.")
        if self.schema_version != PERSONA_SCHEMA_VERSION:
            raise ValueError(f"Unsupported PersonaState schema_version: {self.schema_version!r}")
        object.__setattr__(self, "available_t_ms", available)

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "session_id": self.session_id,
            "source_memory_revision": self.source_memory_revision,
            "persona_revision": self.persona_revision,
            "friendliness": self.friendliness,
            "playfulness": self.playfulness,
            "energy": self.energy,
            "annoyance": self.annoyance,
            "last_interaction_capture_t_ms": self.last_interaction_capture_t_ms,
            "last_input_available_t_ms": self.last_input_available_t_ms,
            "available_t_ms": self.available_t_ms,
        }


@dataclass(frozen=True)
class BehaviorEvent:
    """One discrete semantic behavior; a future executor maps name/variant to motion."""

    behavior_id: int
    session_id: str
    source_memory_revision: int
    behavior_name: str
    variant: str
    source_token_event_ids: tuple[int, ...]
    source_phrase_ids: tuple[int, ...]
    interaction_capture_t_ms: float
    available_t_ms: float
    schema_version: str = BEHAVIOR_SCHEMA_VERSION

    def __post_init__(self) -> None:
        _positive_int(self.behavior_id, "behavior_id")
        object.__setattr__(self, "session_id", _session_id(self.session_id))
        _positive_int(self.source_memory_revision, "source_memory_revision")
        _identifier(self.behavior_name, "behavior_name")
        _identifier(self.variant, "variant")
        if self.variant not in BEHAVIOR_VARIANT_VOCABULARY:
            raise ValueError(
                f"BehaviorEvent variant must be one of {sorted(BEHAVIOR_VARIANT_VOCABULARY)!r}."
            )
        self._validate_ids(self.source_token_event_ids, "source_token_event_ids")
        self._validate_ids(self.source_phrase_ids, "source_phrase_ids")
        if not self.source_token_event_ids and not self.source_phrase_ids:
            raise ValueError("BehaviorEvent requires Token or Phrase provenance.")
        capture = _finite(self.interaction_capture_t_ms, "interaction_capture_t_ms", low=0.0, high=float("inf"))
        available = _finite(self.available_t_ms, "available_t_ms", low=0.0, high=float("inf"))
        if available < capture:
            raise ValueError("Behavior availability must not precede interaction capture time.")
        if self.schema_version != BEHAVIOR_SCHEMA_VERSION:
            raise ValueError(f"Unsupported BehaviorEvent schema_version: {self.schema_version!r}")
        object.__setattr__(self, "interaction_capture_t_ms", capture)
        object.__setattr__(self, "available_t_ms", available)

    @staticmethod
    def _validate_ids(values: object, name: str) -> None:
        if not isinstance(values, tuple):
            raise TypeError(f"{name} must be a tuple.")
        previous = 0
        for value in values:
            if not isinstance(value, int) or isinstance(value, bool) or value < 1 or value <= previous:
                raise ValueError(f"{name} must contain strictly increasing positive ids.")
            previous = value

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "behavior_id": self.behavior_id,
            "session_id": self.session_id,
            "source_memory_revision": self.source_memory_revision,
            "behavior_name": self.behavior_name,
            "variant": self.variant,
            "source_token_event_ids": self.source_token_event_ids,
            "source_phrase_ids": self.source_phrase_ids,
            "interaction_capture_t_ms": self.interaction_capture_t_ms,
            "available_t_ms": self.available_t_ms,
        }


@dataclass(frozen=True)
class Task4Update:
    """One atomic Task 4 publication with explicit admission outcome."""

    persona: PersonaState
    behavior: BehaviorEvent | None
    admission: BehaviorAdmission = BehaviorAdmission.NO_CANDIDATE

    def __post_init__(self) -> None:
        if not isinstance(self.persona, PersonaState):
            raise TypeError("persona must be a PersonaState.")
        if not isinstance(self.admission, BehaviorAdmission):
            raise TypeError("admission must be a BehaviorAdmission.")
        if self.behavior is None:
            if self.admission is BehaviorAdmission.EMITTED:
                raise ValueError("EMITTED Task4Update requires a BehaviorEvent.")
            return
        if not isinstance(self.behavior, BehaviorEvent):
            raise TypeError("behavior must be a BehaviorEvent or None.")
        if self.admission is not BehaviorAdmission.EMITTED:
            raise ValueError("A BehaviorEvent requires EMITTED admission.")
        if (
            self.behavior.session_id != self.persona.session_id
            or self.behavior.source_memory_revision != self.persona.source_memory_revision
        ):
            raise ValueError("BehaviorEvent must match its PersonaState session and Memory revision.")
