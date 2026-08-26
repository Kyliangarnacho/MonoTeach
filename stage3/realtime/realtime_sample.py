"""Realtime arrival envelope for an immutable workspace sample."""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real

from stage2.workspace_trajectory import WorkspaceTrajectorySample


def _finite_nonnegative(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a finite non-negative number.")
    numeric_value = float(value)
    if not math.isfinite(numeric_value) or numeric_value < 0.0:
        raise ValueError(f"{name} must be a finite non-negative number.")
    return numeric_value


@dataclass(frozen=True)
class RealtimeWorkspaceSample:
    """One workspace sample released at a deterministic replay-clock instant.

    ``workspace_sample.t_ms`` remains the canonical event timestamp.  The
    separate ``replay_time_ms`` only describes when this source releases it.
    """

    source_index: int
    replay_time_ms: float
    workspace_sample: WorkspaceTrajectorySample

    def __post_init__(self) -> None:
        if (
            not isinstance(self.source_index, int)
            or isinstance(self.source_index, bool)
            or self.source_index < 0
        ):
            raise ValueError("source_index must be a non-negative integer.")
        object.__setattr__(
            self,
            "replay_time_ms",
            _finite_nonnegative(self.replay_time_ms, "replay_time_ms"),
        )
        if not isinstance(self.workspace_sample, WorkspaceTrajectorySample):
            raise TypeError("workspace_sample must be a WorkspaceTrajectorySample.")

    @property
    def t_ms(self) -> float:
        """Return the untouched canonical workspace-event timestamp."""
        return self.workspace_sample.t_ms
