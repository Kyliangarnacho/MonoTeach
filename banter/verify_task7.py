"""Offline Task 7 control/admission verify; no camera, TCP, or MATLAB."""

from __future__ import annotations

from .contracts import GestureEvent, GestureKind, HandKey
from .interaction_arm_gate import InteractionArmGate, InteractionArmState


def _five(event_id: int, hand: HandKey, t_ms: float) -> GestureEvent:
    return GestureEvent(event_id, GestureKind.OPEN_PALM_FIVE, hand, event_id, t_ms - 400.0, t_ms, t_ms + 1.0, 400.0, 0.9, 2)


def main() -> int:
    gate = InteractionArmGate()
    left, right = _five(1, HandKey.LEFT, 1_000.0), _five(2, HandKey.RIGHT, 1_010.0)
    update = gate.update((left, right), {HandKey.LEFT: GestureKind.OPEN_PALM_FIVE, HandKey.RIGHT: GestureKind.OPEN_PALM_FIVE}, 1_010.0)
    assert update.state is InteractionArmState.ARMED and not update.forwarded_batches
    gate.update((), {HandKey.LEFT: None, HandKey.RIGHT: GestureKind.OPEN_PALM_FIVE}, 1_500.0)
    update = gate.update((_five(3, HandKey.LEFT, 2_000.0),), {HandKey.LEFT: GestureKind.OPEN_PALM_FIVE, HandKey.RIGHT: GestureKind.OPEN_PALM_FIVE}, 2_000.0)
    assert update.state is InteractionArmState.DISARMED and not update.forwarded_batches
    print("PASS: Task 7 control gate starts DISARMED, toggles only on one-shot double FIVE, and emits no business Token.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
