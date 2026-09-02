"""Task 4 appraisal-only regression coverage."""

from __future__ import annotations

from banter.contracts import GestureEvent, GestureKind, HandKey
from banter.grammar_contracts import GrammarUpdate
from banter.interaction_memory import InteractionMemory
from banter.personality_behavior import appraise_persona, empty_persona_state
from banter.personality_contracts import PersonaConfig


def _token(event_id: int, gesture: GestureKind, hand: HandKey, t_ms: float) -> GestureEvent:
    return GestureEvent(event_id, gesture, hand, event_id, max(0.0, t_ms - 50.0), t_ms, t_ms + 5.0, min(50.0, t_ms), 0.9, 1)


def test_middle_finger_has_its_own_affect_delta_and_capture_time_decay_remains_pure() -> None:
    config = PersonaConfig()
    memory = InteractionMemory("affect", clock_ms=lambda: 100_000.0)
    first = memory.update(GrammarUpdate((_token(1, GestureKind.MIDDLE_FINGER, HandKey.LEFT, 100.0),), ()))
    state = appraise_persona(empty_persona_state("affect", config), memory.reset("affect"), first, config)
    assert state.annoyance > config.profile.baseline_annoyance
    assert state.friendliness < config.profile.baseline_friendliness
