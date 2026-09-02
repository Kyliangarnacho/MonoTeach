"""Contract coverage for Task 3 canonical immutable interaction context."""

from __future__ import annotations

from dataclasses import FrozenInstanceError

import pytest

from banter.contracts import GestureKind, HandKey
from banter.grammar_contracts import PhraseForm
from banter.demo_interaction_memory import memory_status_lines
from banter.interaction_memory import empty_snapshot
from banter.memory_contracts import (
    InteractionMemoryConfig,
    PhraseMemoryKey,
    PhraseStreak,
    TokenMemoryKey,
    TokenStreak,
)


def test_default_empty_snapshot_has_stable_json_and_zero_queries() -> None:
    snapshot = empty_snapshot("session-a")
    assert snapshot.revision == snapshot.interaction_turn_count == 0
    assert snapshot.total_token_count == snapshot.total_phrase_count == 0
    assert snapshot.recent_tokens == snapshot.recent_phrases == ()
    assert snapshot.token_total(HandKey.LEFT, GestureKind.FIST) == 0
    assert snapshot.phrase_total(PhraseForm.CHORD, "POWER_VICTORY") == 0
    assert snapshot.to_dict() == {
        "schema_version": "banter_interaction_memory_v1",
        "session_id": "session-a",
        "revision": 0,
        "interaction_turn_count": 0,
        "total_token_count": 0,
        "total_phrase_count": 0,
        "recent_tokens": [],
        "recent_phrases": [],
        "token_counts": [],
        "phrase_counts": [],
        "left_token_streak": None,
        "right_token_streak": None,
        "phrase_streak": None,
        "last_interaction_capture_t_ms": None,
        "last_input_available_t_ms": None,
        "available_t_ms": 0.0,
    }


def test_config_keys_and_streaks_reject_invalid_or_mutable_state() -> None:
    with pytest.raises(ValueError, match="longest Phrase provenance"):
        InteractionMemoryConfig(recent_token_capacity=2)
    with pytest.raises(ValueError, match="positive duration"):
        InteractionMemoryConfig(token_streak_gap_ms=0.0)
    with pytest.raises(ValueError, match="LEFT or RIGHT"):
        TokenMemoryKey(HandKey.UNKNOWN, GestureKind.FIST)
    with pytest.raises(ValueError, match="uppercase identifier"):
        PhraseMemoryKey(PhraseForm.CHORD, "not-a-rule")
    with pytest.raises(ValueError, match="cannot precede"):
        TokenStreak(TokenMemoryKey(HandKey.LEFT, GestureKind.FIST), 1, 20.0, 10.0)
    with pytest.raises(ValueError, match="cannot precede"):
        PhraseStreak(PhraseMemoryKey(PhraseForm.CHORD, "POWER_VICTORY"), 1, 20.0, 10.0)
    with pytest.raises(FrozenInstanceError):
        empty_snapshot("session-b").revision = 2  # type: ignore[misc]


def test_memory_demo_status_is_compact_and_uses_only_snapshot_facts() -> None:
    lines = memory_status_lines(empty_snapshot("demo-session"))
    assert lines[0] == "Session: demo-session r0"
    assert lines[1] == "Totals: T=0 P=0"
    assert "None" in lines[-1]
