"""Control-only two-hand FIVE gate placed between Task 1 and Task 2.

The gate never derives a business Token or Phrase.  It keeps Task 1 running
while DISARMED, so its own control chord remains observable, then either
forwards original GestureEvent batches unchanged or drops them before Grammar.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Mapping

from .contracts import GestureEvent, GestureKind, HandKey


class InteractionArmState(str, Enum):
    DISARMED = "DISARMED"
    ARMED = "ARMED"


class InteractionArmTransition(str, Enum):
    NONE = "NONE"
    ARMED = "ARMED"
    DISARMED = "DISARMED"


@dataclass(frozen=True)
class InteractionArmGateConfig:
    """Small ambiguity window that preserves normal single-hand FIVE use."""

    control_pair_window_ms: float = 650.0

    def __post_init__(self) -> None:
        value = float(self.control_pair_window_ms)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("control_pair_window_ms must be finite and positive.")
        object.__setattr__(self, "control_pair_window_ms", value)


@dataclass(frozen=True)
class InteractionGateUpdate:
    state: InteractionArmState
    transition: InteractionArmTransition
    forwarded_batches: tuple[tuple[GestureEvent, ...], ...]
    control_event_ids: tuple[int, ...]


@dataclass
class _PendingBatch:
    tokens: tuple[GestureEvent, ...]
    deadline_t_ms: float | None = None


class InteractionArmGate:
    """Toggle ARMED/DISARMED on stable LEFT:FIVE + RIGHT:FIVE.

    A standalone FIVE is held briefly only while its control meaning is
    ambiguous.  Once it is committed to business forwarding it can no longer
    be retroactively claimed as an arm chord, which prevents downstream
    grammar pollution.
    """

    def __init__(self, config: InteractionArmGateConfig = InteractionArmGateConfig()) -> None:
        if not isinstance(config, InteractionArmGateConfig):
            raise TypeError("config must be an InteractionArmGateConfig.")
        self.config = config
        self._state = InteractionArmState.DISARMED
        self._latched = False
        self._last_capture_t_ms: float | None = None
        self._last_event_id = 0
        self._pending: list[_PendingBatch] = []
        self._latest_five: dict[HandKey, GestureEvent | None] = {HandKey.LEFT: None, HandKey.RIGHT: None}
        self._five_origin: dict[HandKey, str | None] = {HandKey.LEFT: None, HandKey.RIGHT: None}

    @property
    def state(self) -> InteractionArmState:
        return self._state

    def reset(self) -> None:
        self._state = InteractionArmState.DISARMED
        self._latched = False
        self._last_capture_t_ms = None
        self._last_event_id = 0
        self._pending.clear()
        self._latest_five = {HandKey.LEFT: None, HandKey.RIGHT: None}
        self._five_origin = {HandKey.LEFT: None, HandKey.RIGHT: None}

    def update(
        self,
        tokens: tuple[GestureEvent, ...],
        active_gestures: Mapping[HandKey, GestureKind | None],
        capture_t_ms: float,
    ) -> InteractionGateUpdate:
        """Process one Task 1 frame and return ordered business batches."""
        now = float(capture_t_ms)
        if not math.isfinite(now) or now < 0.0:
            raise ValueError("capture_t_ms must be finite and non-negative.")
        if self._last_capture_t_ms is not None and now <= self._last_capture_t_ms:
            raise ValueError("InteractionArmGate requires strictly increasing capture timestamps.")
        if not isinstance(tokens, tuple) or len(tokens) > 2 or not all(isinstance(item, GestureEvent) for item in tokens):
            raise TypeError("tokens must be one Task 1 GestureEvent tuple.")
        previous = self._last_event_id
        for token in tokens:
            if token.event_id <= previous:
                raise ValueError("InteractionArmGate requires strictly increasing event IDs.")
            previous = token.event_id
        self._last_event_id = previous
        self._last_capture_t_ms = now
        self._refresh_active_fives(active_gestures)

        if self._latched and not self._both_active_five(active_gestures):
            self._latched = False

        for token in tokens:
            if token.gesture is GestureKind.OPEN_PALM_FIVE:
                self._latest_five[token.hand_key] = token
                self._five_origin[token.hand_key] = None

        transition = InteractionArmTransition.NONE
        control_ids: tuple[int, ...] = ()
        forwarded: list[tuple[GestureEvent, ...]] = []

        if self._state is InteractionArmState.DISARMED:
            self._pending.clear()
            if not self._latched and tokens and self._eligible_control(active_gestures):
                control_ids = self._toggle()
                transition = InteractionArmTransition.ARMED
            return InteractionGateUpdate(self._state, transition, (), control_ids)

        # Resolve an elapsed earlier FIVE before considering this frame.  A
        # second hand confirmed after the window must not retroactively turn a
        # standalone business FIVE into an ARM/DISARM command.
        forwarded.extend(self._flush_expired(now))

        # ARMED: preserve order by buffering every batch behind an unresolved
        # FIVE.  This avoids forwarding event N+1 while event N is waiting to
        # learn whether it is the first half of the control chord.
        if tokens and not self._pending and not any(item.gesture is GestureKind.OPEN_PALM_FIVE for item in tokens):
            return InteractionGateUpdate(self._state, transition, (tokens,), control_ids)

        if tokens:
            self._pending.append(_PendingBatch(tokens))
            if any(item.gesture is GestureKind.OPEN_PALM_FIVE for item in tokens) and not self._latched:
                first_five = next(item for item in tokens if item.gesture is GestureKind.OPEN_PALM_FIVE)
                if self._pending[0].deadline_t_ms is None:
                    self._pending[0].deadline_t_ms = first_five.confirmed_capture_t_ms + self.config.control_pair_window_ms

        if not self._latched and self._pending and self._eligible_control(active_gestures):
            control_ids = self._toggle()
            transition = InteractionArmTransition.DISARMED
            # The whole unresolved interval is intentionally not business
            # input.  It contains the control source and has never crossed
            # the Task 2 boundary.
            self._pending.clear()
            return InteractionGateUpdate(self._state, transition, (), control_ids)

        forwarded.extend(self._flush_expired(now))
        return InteractionGateUpdate(self._state, transition, tuple(forwarded), control_ids)

    def _flush_expired(self, now: float) -> list[tuple[GestureEvent, ...]]:
        forwarded: list[tuple[GestureEvent, ...]] = []
        while self._pending and self._pending[0].deadline_t_ms is not None and now >= self._pending[0].deadline_t_ms:
            batch = self._pending.pop(0).tokens
            forwarded.append(batch)
            for token in batch:
                if token.gesture is GestureKind.OPEN_PALM_FIVE:
                    self._five_origin[token.hand_key] = "BUSINESS"
            # Subsequent batches were held only for ordering; if they contain
            # no unresolved FIVE they can now flow in the same frame.
            while self._pending and self._pending[0].deadline_t_ms is None:
                batch = self._pending.pop(0).tokens
                forwarded.append(batch)
                for token in batch:
                    if token.gesture is GestureKind.OPEN_PALM_FIVE:
                        self._five_origin[token.hand_key] = "BUSINESS"
        return forwarded

    def _refresh_active_fives(self, active: Mapping[HandKey, GestureKind | None]) -> None:
        for hand_key in (HandKey.LEFT, HandKey.RIGHT):
            if active.get(hand_key) is not GestureKind.OPEN_PALM_FIVE:
                self._latest_five[hand_key] = None
                self._five_origin[hand_key] = None

    @staticmethod
    def _both_active_five(active: Mapping[HandKey, GestureKind | None]) -> bool:
        return all(active.get(hand_key) is GestureKind.OPEN_PALM_FIVE for hand_key in (HandKey.LEFT, HandKey.RIGHT))

    def _eligible_control(self, active: Mapping[HandKey, GestureKind | None]) -> bool:
        if not self._both_active_five(active):
            return False
        if any(self._latest_five[hand_key] is None for hand_key in (HandKey.LEFT, HandKey.RIGHT)):
            return False
        # A FIVE already delivered to business remains a normal business
        # gesture until that hand releases; it cannot be reclaimed later.
        return all(self._five_origin[hand_key] != "BUSINESS" for hand_key in (HandKey.LEFT, HandKey.RIGHT))

    def _toggle(self) -> tuple[int, ...]:
        self._state = (InteractionArmState.ARMED if self._state is InteractionArmState.DISARMED
                       else InteractionArmState.DISARMED)
        self._latched = True
        for hand_key in (HandKey.LEFT, HandKey.RIGHT):
            self._five_origin[hand_key] = "CONTROL"
        return tuple(sorted(self._latest_five[hand_key].event_id for hand_key in (HandKey.LEFT, HandKey.RIGHT)
                            if self._latest_five[hand_key] is not None))
