"""Task 7 control-only interaction arm gate regressions."""

from __future__ import annotations

from banter.contracts import GestureEvent, GestureKind, HandKey
from banter.interaction_arm_gate import InteractionArmGate, InteractionArmState, InteractionArmTransition


def _event(event_id: int, gesture: GestureKind, hand: HandKey, t_ms: float) -> GestureEvent:
    return GestureEvent(event_id, gesture, hand, event_id, t_ms - 400.0, t_ms, t_ms + 1.0, 400.0, 0.9, 2)


def _active(left: GestureKind | None, right: GestureKind | None):
    return {HandKey.LEFT: left, HandKey.RIGHT: right}


def test_gate_starts_disarmed_and_double_five_toggles_once_until_stable_release() -> None:
    gate = InteractionArmGate()
    left = _event(1, GestureKind.OPEN_PALM_FIVE, HandKey.LEFT, 1_000.0)
    right = _event(2, GestureKind.OPEN_PALM_FIVE, HandKey.RIGHT, 1_010.0)
    armed = gate.update((left, right), _active(GestureKind.OPEN_PALM_FIVE, GestureKind.OPEN_PALM_FIVE), 1_010.0)
    assert armed.state is InteractionArmState.ARMED
    assert armed.transition is InteractionArmTransition.ARMED
    assert armed.forwarded_batches == () and armed.control_event_ids == (1, 2)

    held = gate.update((), _active(GestureKind.OPEN_PALM_FIVE, GestureKind.OPEN_PALM_FIVE), 1_100.0)
    assert held.state is InteractionArmState.ARMED and held.transition is InteractionArmTransition.NONE

    gate.update((), _active(None, GestureKind.OPEN_PALM_FIVE), 1_500.0)
    left_again = _event(3, GestureKind.OPEN_PALM_FIVE, HandKey.LEFT, 2_000.0)
    disarmed = gate.update((left_again,), _active(GestureKind.OPEN_PALM_FIVE, GestureKind.OPEN_PALM_FIVE), 2_000.0)
    assert disarmed.state is InteractionArmState.DISARMED
    assert disarmed.transition is InteractionArmTransition.DISARMED
    assert disarmed.control_event_ids == (2, 3)


def test_disarmed_drops_business_events_but_single_five_remains_available_when_armed() -> None:
    gate = InteractionArmGate()
    point = _event(1, GestureKind.POINT_ONE, HandKey.LEFT, 1_000.0)
    assert gate.update((point,), _active(GestureKind.POINT_ONE, None), 1_000.0).forwarded_batches == ()

    left = _event(2, GestureKind.OPEN_PALM_FIVE, HandKey.LEFT, 2_000.0)
    right = _event(3, GestureKind.OPEN_PALM_FIVE, HandKey.RIGHT, 2_010.0)
    gate.update((left, right), _active(GestureKind.OPEN_PALM_FIVE, GestureKind.OPEN_PALM_FIVE), 2_010.0)
    gate.update((), _active(None, None), 2_500.0)  # release the control latch

    standalone = _event(4, GestureKind.OPEN_PALM_FIVE, HandKey.LEFT, 3_000.0)
    pending = gate.update((standalone,), _active(GestureKind.OPEN_PALM_FIVE, None), 3_000.0)
    assert pending.forwarded_batches == ()
    released = gate.update((), _active(GestureKind.OPEN_PALM_FIVE, None), 3_700.0)
    assert released.state is InteractionArmState.ARMED
    assert released.forwarded_batches == ((standalone,),)

    # The already-forwarded FIVE cannot later be reclaimed as a control chord.
    late_right = _event(5, GestureKind.OPEN_PALM_FIVE, HandKey.RIGHT, 3_800.0)
    no_toggle = gate.update((late_right,), _active(GestureKind.OPEN_PALM_FIVE, GestureKind.OPEN_PALM_FIVE), 3_800.0)
    assert no_toggle.state is InteractionArmState.ARMED
    assert no_toggle.transition is InteractionArmTransition.NONE


def test_late_second_five_cannot_reclaim_an_expired_business_five() -> None:
    gate = InteractionArmGate()
    left = _event(1, GestureKind.OPEN_PALM_FIVE, HandKey.LEFT, 1_000.0)
    right = _event(2, GestureKind.OPEN_PALM_FIVE, HandKey.RIGHT, 1_010.0)
    gate.update((left, right), _active(GestureKind.OPEN_PALM_FIVE, GestureKind.OPEN_PALM_FIVE), 1_010.0)
    gate.update((), _active(None, None), 1_500.0)
    first = _event(3, GestureKind.OPEN_PALM_FIVE, HandKey.LEFT, 2_000.0)
    gate.update((first,), _active(GestureKind.OPEN_PALM_FIVE, None), 2_000.0)
    # No intervening frame arrives until after the 650 ms ambiguity window.
    # The right FIVE must see the left event as normal business, not toggle.
    late = _event(4, GestureKind.OPEN_PALM_FIVE, HandKey.RIGHT, 2_800.0)
    update = gate.update((late,), _active(GestureKind.OPEN_PALM_FIVE, GestureKind.OPEN_PALM_FIVE), 2_800.0)
    assert update.state is InteractionArmState.ARMED
    assert update.transition is InteractionArmTransition.NONE
    assert update.forwarded_batches == ((first,),)
