"""Immutable Task 3 contracts for GrammarUpdate → interaction context."""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real

from .contracts import GestureEvent, GestureKind, HandKey
from .grammar_contracts import GesturePhraseEvent, PhraseForm


MEMORY_SCHEMA_VERSION = "banter_interaction_memory_v1"
_CURRENT_LONGEST_PHRASE_PROVENANCE = 3


def _positive_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 1:
        raise ValueError(f"{name} must be a positive integer.")
    return value


def _nonnegative_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer.")
    return value


def _finite_nonnegative(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a finite non-negative number.")
    numeric = float(value)
    if not math.isfinite(numeric) or numeric < 0.0:
        raise ValueError(f"{name} must be a finite non-negative number.")
    return numeric


def _finite_positive(value: object, name: str) -> float:
    numeric = _finite_nonnegative(value, name)
    if numeric <= 0.0:
        raise ValueError(f"{name} must be a finite positive duration in milliseconds.")
    return numeric


def _session_id(value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError("session_id must be a non-empty string.")
    return value


def _rule_name(value: object) -> str:
    if not isinstance(value, str) or not value or not value.isidentifier() or value != value.upper():
        raise ValueError("name must be a non-empty uppercase identifier.")
    return value


@dataclass(frozen=True)
class InteractionMemoryConfig:
    """Bounded retained history and capture-time streak windows."""

    recent_token_capacity: int = 32
    recent_phrase_capacity: int = 16
    token_streak_gap_ms: float = 5_000.0
    phrase_streak_gap_ms: float = 8_000.0

    def __post_init__(self) -> None:
        token_capacity = _positive_int(self.recent_token_capacity, "recent_token_capacity")
        _positive_int(self.recent_phrase_capacity, "recent_phrase_capacity")
        if token_capacity < _CURRENT_LONGEST_PHRASE_PROVENANCE:
            raise ValueError(
                "recent_token_capacity must retain the current longest Phrase provenance "
                f"({_CURRENT_LONGEST_PHRASE_PROVENANCE} Tokens)."
            )
        object.__setattr__(self, "token_streak_gap_ms", _finite_positive(self.token_streak_gap_ms, "token_streak_gap_ms"))
        object.__setattr__(self, "phrase_streak_gap_ms", _finite_positive(self.phrase_streak_gap_ms, "phrase_streak_gap_ms"))


@dataclass(frozen=True)
class TokenMemoryKey:
    """Typed key preserving both hand role and gesture kind."""

    hand_key: HandKey
    gesture: GestureKind

    def __post_init__(self) -> None:
        if self.hand_key not in {HandKey.LEFT, HandKey.RIGHT}:
            raise ValueError("TokenMemoryKey requires LEFT or RIGHT hand_key.")
        if not isinstance(self.gesture, GestureKind) or self.gesture is GestureKind.UNKNOWN:
            raise ValueError("TokenMemoryKey requires a known GestureKind.")


@dataclass(frozen=True)
class PhraseMemoryKey:
    """Typed key retaining both Task 2 phrase form and declarative name."""

    form: PhraseForm
    name: str

    def __post_init__(self) -> None:
        if not isinstance(self.form, PhraseForm):
            raise TypeError("form must be a PhraseForm.")
        object.__setattr__(self, "name", _rule_name(self.name))


@dataclass(frozen=True)
class TokenCountEntry:
    key: TokenMemoryKey
    count: int

    def __post_init__(self) -> None:
        if not isinstance(self.key, TokenMemoryKey):
            raise TypeError("key must be a TokenMemoryKey.")
        _positive_int(self.count, "count")


@dataclass(frozen=True)
class PhraseCountEntry:
    key: PhraseMemoryKey
    count: int

    def __post_init__(self) -> None:
        if not isinstance(self.key, PhraseMemoryKey):
            raise TypeError("key must be a PhraseMemoryKey.")
        _positive_int(self.count, "count")


@dataclass(frozen=True)
class TokenStreak:
    key: TokenMemoryKey
    count: int
    started_capture_t_ms: float
    last_capture_t_ms: float

    def __post_init__(self) -> None:
        if not isinstance(self.key, TokenMemoryKey):
            raise TypeError("key must be a TokenMemoryKey.")
        _positive_int(self.count, "count")
        started = _finite_nonnegative(self.started_capture_t_ms, "started_capture_t_ms")
        last = _finite_nonnegative(self.last_capture_t_ms, "last_capture_t_ms")
        if last < started:
            raise ValueError("TokenStreak last_capture_t_ms cannot precede its start.")
        object.__setattr__(self, "started_capture_t_ms", started)
        object.__setattr__(self, "last_capture_t_ms", last)


@dataclass(frozen=True)
class PhraseStreak:
    key: PhraseMemoryKey
    count: int
    started_capture_t_ms: float
    last_capture_t_ms: float

    def __post_init__(self) -> None:
        if not isinstance(self.key, PhraseMemoryKey):
            raise TypeError("key must be a PhraseMemoryKey.")
        _positive_int(self.count, "count")
        started = _finite_nonnegative(self.started_capture_t_ms, "started_capture_t_ms")
        last = _finite_nonnegative(self.last_capture_t_ms, "last_capture_t_ms")
        if last < started:
            raise ValueError("PhraseStreak last_capture_t_ms cannot precede its start.")
        object.__setattr__(self, "started_capture_t_ms", started)
        object.__setattr__(self, "last_capture_t_ms", last)


@dataclass(frozen=True)
class PhraseFingerprint:
    """Deduplication identity for one non-destructive Task 2 phrase fact."""

    key: PhraseMemoryKey
    source_event_ids: tuple[int, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.key, PhraseMemoryKey):
            raise TypeError("key must be a PhraseMemoryKey.")
        if not isinstance(self.source_event_ids, tuple) or len(self.source_event_ids) < 2:
            raise ValueError("source_event_ids must be a tuple containing at least two event ids.")
        previous_id = 0
        for event_id in self.source_event_ids:
            if not isinstance(event_id, int) or isinstance(event_id, bool) or event_id < 1:
                raise ValueError("source_event_ids must contain positive integer ids.")
            if event_id <= previous_id:
                raise ValueError("source_event_ids must be strictly increasing.")
            previous_id = event_id


@dataclass(frozen=True)
class InteractionMemorySnapshot:
    """Complete immutable Task 3 interaction context for one local session."""

    session_id: str
    revision: int = 0
    interaction_turn_count: int = 0
    total_token_count: int = 0
    total_phrase_count: int = 0
    recent_tokens: tuple[GestureEvent, ...] = ()
    recent_phrases: tuple[GesturePhraseEvent, ...] = ()
    token_counts: tuple[TokenCountEntry, ...] = ()
    phrase_counts: tuple[PhraseCountEntry, ...] = ()
    left_token_streak: TokenStreak | None = None
    right_token_streak: TokenStreak | None = None
    phrase_streak: PhraseStreak | None = None
    last_interaction_capture_t_ms: float | None = None
    last_input_available_t_ms: float | None = None
    available_t_ms: float = 0.0
    last_token_event_id: int = 0
    last_phrase_id: int = 0
    phrase_fingerprints: frozenset[PhraseFingerprint] = frozenset()
    schema_version: str = MEMORY_SCHEMA_VERSION

    def __post_init__(self) -> None:
        object.__setattr__(self, "session_id", _session_id(self.session_id))
        _nonnegative_int(self.revision, "revision")
        _nonnegative_int(self.interaction_turn_count, "interaction_turn_count")
        _nonnegative_int(self.total_token_count, "total_token_count")
        _nonnegative_int(self.total_phrase_count, "total_phrase_count")
        available = _finite_nonnegative(self.available_t_ms, "available_t_ms")
        _nonnegative_int(self.last_token_event_id, "last_token_event_id")
        _nonnegative_int(self.last_phrase_id, "last_phrase_id")
        if self.schema_version != MEMORY_SCHEMA_VERSION:
            raise ValueError(f"Unsupported InteractionMemorySnapshot schema_version: {self.schema_version!r}")
        self._validate_tuple(self.recent_tokens, GestureEvent, "recent_tokens")
        self._validate_tuple(self.recent_phrases, GesturePhraseEvent, "recent_phrases")
        self._validate_tuple(self.token_counts, TokenCountEntry, "token_counts")
        self._validate_tuple(self.phrase_counts, PhraseCountEntry, "phrase_counts")
        if not isinstance(self.phrase_fingerprints, frozenset) or not all(
            isinstance(item, PhraseFingerprint) for item in self.phrase_fingerprints
        ):
            raise TypeError("phrase_fingerprints must be a frozenset of PhraseFingerprint values.")
        self._validate_unique_counts()
        if sum(item.count for item in self.token_counts) != self.total_token_count:
            raise ValueError("total_token_count must equal the Token count entries.")
        if sum(item.count for item in self.phrase_counts) != self.total_phrase_count:
            raise ValueError("total_phrase_count must equal the Phrase count entries.")
        if self.recent_tokens and self.last_token_event_id < self.recent_tokens[-1].event_id:
            raise ValueError("last_token_event_id cannot precede recent_tokens.")
        if self.recent_phrases and self.last_phrase_id < self.recent_phrases[-1].phrase_id:
            raise ValueError("last_phrase_id cannot precede recent_phrases.")
        for streak, hand in ((self.left_token_streak, HandKey.LEFT), (self.right_token_streak, HandKey.RIGHT)):
            if streak is not None and (not isinstance(streak, TokenStreak) or streak.key.hand_key is not hand):
                raise ValueError(f"{hand.value} token streak must match its hand.")
        if self.phrase_streak is not None and not isinstance(self.phrase_streak, PhraseStreak):
            raise TypeError("phrase_streak must be a PhraseStreak or None.")
        last_capture = self._optional_time(self.last_interaction_capture_t_ms, "last_interaction_capture_t_ms")
        input_available = self._optional_time(self.last_input_available_t_ms, "last_input_available_t_ms")
        if input_available is not None and available < input_available:
            raise ValueError("Snapshot availability must not precede accepted input availability.")
        if self.interaction_turn_count == 0 and any((self.total_token_count, self.total_phrase_count, last_capture, input_available)):
            raise ValueError("An empty session cannot contain interaction facts or timestamps.")
        object.__setattr__(self, "available_t_ms", available)

    @staticmethod
    def _validate_tuple(value: object, expected_type: type, name: str) -> None:
        if not isinstance(value, tuple) or not all(isinstance(item, expected_type) for item in value):
            raise TypeError(f"{name} must be a tuple of {expected_type.__name__} values.")

    @staticmethod
    def _optional_time(value: object, name: str) -> float | None:
        if value is None:
            return None
        return _finite_nonnegative(value, name)

    def _validate_unique_counts(self) -> None:
        token_keys = [item.key for item in self.token_counts]
        phrase_keys = [item.key for item in self.phrase_counts]
        if len(set(token_keys)) != len(token_keys) or len(set(phrase_keys)) != len(phrase_keys):
            raise ValueError("Memory count entries must not repeat a key.")

    def token_total(self, hand_key: HandKey, gesture: GestureKind) -> int:
        """Return a cumulative session count; absent keys have total zero."""
        key = TokenMemoryKey(hand_key, gesture)
        return next((item.count for item in self.token_counts if item.key == key), 0)

    def phrase_total(self, form: PhraseForm, name: str) -> int:
        """Return a cumulative session count; absent keys have total zero."""
        key = PhraseMemoryKey(form, name)
        return next((item.count for item in self.phrase_counts if item.key == key), 0)

    def current_token_streak(self, hand_key: HandKey) -> TokenStreak | None:
        if hand_key is HandKey.LEFT:
            return self.left_token_streak
        if hand_key is HandKey.RIGHT:
            return self.right_token_streak
        raise ValueError("current_token_streak requires LEFT or RIGHT hand_key.")

    def current_phrase_streak(self) -> PhraseStreak | None:
        return self.phrase_streak

    def to_dict(self) -> dict[str, object]:
        """Return stable JSON-safe audit data without frames or landmarks."""
        return {
            "schema_version": self.schema_version,
            "session_id": self.session_id,
            "revision": self.revision,
            "interaction_turn_count": self.interaction_turn_count,
            "total_token_count": self.total_token_count,
            "total_phrase_count": self.total_phrase_count,
            "recent_tokens": [item.to_dict() for item in self.recent_tokens],
            "recent_phrases": [item.to_dict() for item in self.recent_phrases],
            "token_counts": [
                {"hand_key": item.key.hand_key.value, "gesture": item.key.gesture.value, "count": item.count}
                for item in self.token_counts
            ],
            "phrase_counts": [
                {"form": item.key.form.value, "name": item.key.name, "count": item.count}
                for item in self.phrase_counts
            ],
            "left_token_streak": self._streak_dict(self.left_token_streak),
            "right_token_streak": self._streak_dict(self.right_token_streak),
            "phrase_streak": self._streak_dict(self.phrase_streak),
            "last_interaction_capture_t_ms": self.last_interaction_capture_t_ms,
            "last_input_available_t_ms": self.last_input_available_t_ms,
            "available_t_ms": self.available_t_ms,
        }

    @staticmethod
    def _streak_dict(streak: TokenStreak | PhraseStreak | None) -> dict[str, object] | None:
        if streak is None:
            return None
        if isinstance(streak, TokenStreak):
            return {
                "hand_key": streak.key.hand_key.value,
                "gesture": streak.key.gesture.value,
                "count": streak.count,
                "started_capture_t_ms": streak.started_capture_t_ms,
                "last_capture_t_ms": streak.last_capture_t_ms,
            }
        return {
            "form": streak.key.form.value,
            "name": streak.key.name,
            "count": streak.count,
            "started_capture_t_ms": streak.started_capture_t_ms,
            "last_capture_t_ms": streak.last_capture_t_ms,
        }
