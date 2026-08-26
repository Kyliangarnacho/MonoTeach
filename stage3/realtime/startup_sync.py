"""Start-state synchronization for a future live visual source.

The camera is allowed to keep observing while the arm moves from ``HOME`` to
the demonstrated start pose.  Those observations are *preparation evidence*,
not demonstration waypoints: they must not enter the rolling derivative
window.  This small state machine makes that boundary explicit without
introducing ROS, threads, or a general-purpose message framework.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real

from .stream_contract import RealtimeTaskEvent


class StartSyncPhase(str, Enum):
    """Lifecycle before and during formal trajectory tracking."""

    WAIT_FOR_STABLE_START = "WAIT_FOR_STABLE_START"
    APPROACHING_START = "APPROACHING_START"
    READY = "READY"
    TRACKING = "TRACKING"


def _positive_finite(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a positive finite number.")
    numeric = float(value)
    if not math.isfinite(numeric) or numeric <= 0.0:
        raise ValueError(f"{name} must be a positive finite number.")
    return numeric


@dataclass(frozen=True)
class StartSyncConfig:
    """Small, user-visible dwell gate for the real camera integration."""

    dwell_time_ms: float = 400.0
    stationary_radius_mm: float = 2.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "dwell_time_ms", _positive_finite(self.dwell_time_ms, "dwell_time_ms"))
        object.__setattr__(
            self,
            "stationary_radius_mm",
            _positive_finite(self.stationary_radius_mm, "stationary_radius_mm"),
        )


@dataclass(frozen=True)
class StartSyncDecision:
    """Result of one camera observation or one robot-ready acknowledgement."""

    phase: StartSyncPhase
    planner_accepts_observation: bool
    start_xy_mm: tuple[float, float] | None = None
    reason: str | None = None


@dataclass(frozen=True)
class StartupReplaySchedule:
    """Derived pre-roll for the deterministic virtual-camera demo.

    ``prepare_duration_ms`` is reserved for Home-to-Start.  During that
    interval a virtual camera publishes the first pose repeatedly, just as a
    stationary hand would be observed by a real camera.  These generated
    observations are deliberately not canonical source samples.
    """

    prepare_duration_ms: float = 8_000.0
    prepare_sample_period_ms: float = 100.0

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "prepare_duration_ms",
            _positive_finite(self.prepare_duration_ms, "prepare_duration_ms"),
        )
        object.__setattr__(
            self,
            "prepare_sample_period_ms",
            _positive_finite(self.prepare_sample_period_ms, "prepare_sample_period_ms"),
        )

    def prepare_replay_times_ms(self) -> tuple[float, ...]:
        """Return monotone derived publication instants, including both ends."""
        count = int(math.floor(self.prepare_duration_ms / self.prepare_sample_period_ms))
        times = [index * self.prepare_sample_period_ms for index in range(count + 1)]
        if times[-1] < self.prepare_duration_ms:
            times.append(self.prepare_duration_ms)
        return tuple(times)


class StartSynchronizationController:
    """Gate observations until a stationary start and robot-ready handshake.

    In ``TRACKING`` every observation is accepted by the rolling planner.  In
    all earlier phases observations are retained only in a bounded dwell
    window, so waiting at the start cannot contaminate p/v/a estimation.
    """

    def __init__(self, config: StartSyncConfig | None = None) -> None:
        self._config = StartSyncConfig() if config is None else config
        if not isinstance(self._config, StartSyncConfig):
            raise TypeError("config must be a StartSyncConfig.")
        self._phase = StartSyncPhase.WAIT_FOR_STABLE_START
        self._window: list[RealtimeTaskEvent] = []
        self._start_xy_mm: tuple[float, float] | None = None
        self._last_source_t_ms: float | None = None

    @property
    def phase(self) -> StartSyncPhase:
        return self._phase

    @property
    def start_xy_mm(self) -> tuple[float, float] | None:
        return self._start_xy_mm

    def observe(self, event: RealtimeTaskEvent) -> StartSyncDecision:
        """Consume one source observation without changing the source event."""
        if not isinstance(event, RealtimeTaskEvent):
            raise TypeError("event must be a RealtimeTaskEvent.")
        if self._last_source_t_ms is not None and event.source_t_ms <= self._last_source_t_ms:
            raise ValueError("source_t_ms must strictly increase.")
        self._last_source_t_ms = event.source_t_ms

        if self._phase is StartSyncPhase.TRACKING:
            return StartSyncDecision(self._phase, planner_accepts_observation=True)
        if not event.planning_eligible:
            self._clear_candidate()
            return StartSyncDecision(self._phase, False, reason="invalid_or_outside_start_observation")

        point = (event.x_mm, event.y_mm)
        assert point[0] is not None and point[1] is not None
        point_xy = (float(point[0]), float(point[1]))

        if self._phase is StartSyncPhase.APPROACHING_START:
            if self._start_xy_mm is not None and self._distance(point_xy, self._start_xy_mm) <= self._config.stationary_radius_mm:
                return StartSyncDecision(self._phase, False, self._start_xy_mm)
            self._clear_candidate()
            return StartSyncDecision(self._phase, False, reason="hand_moved_before_robot_ready")

        if self._phase is StartSyncPhase.READY:
            return StartSyncDecision(self._phase, False, self._start_xy_mm)

        self._window.append(event)
        cutoff = event.source_t_ms - self._config.dwell_time_ms
        self._window = [sample for sample in self._window if sample.source_t_ms >= cutoff]
        if self._window[-1].source_t_ms - self._window[0].source_t_ms < self._config.dwell_time_ms:
            return StartSyncDecision(self._phase, False, reason="collecting_stationary_dwell")

        candidate = self._mean_xy(self._window)
        if all(self._distance((float(item.x_mm), float(item.y_mm)), candidate) <= self._config.stationary_radius_mm for item in self._window):
            self._start_xy_mm = candidate
            self._phase = StartSyncPhase.APPROACHING_START
            return StartSyncDecision(self._phase, False, candidate, "stable_start_detected")

        # Keep only the newest sample: an actual movement starts a new dwell.
        self._window = [event]
        return StartSyncDecision(self._phase, False, reason="start_not_stationary")

    def notify_robot_at_start(self) -> StartSyncDecision:
        """Acknowledge a completed Home-to-Start approach after stability held."""
        if self._phase is not StartSyncPhase.APPROACHING_START or self._start_xy_mm is None:
            raise RuntimeError("Robot may acknowledge start only while approaching a stable start pose.")
        self._phase = StartSyncPhase.READY
        self._window.clear()
        return StartSyncDecision(self._phase, False, self._start_xy_mm, "robot_and_hand_ready")

    def begin_tracking(self) -> StartSyncDecision:
        """Create the formal demo epoch; subsequent events enter the planner."""
        if self._phase is not StartSyncPhase.READY:
            raise RuntimeError("Tracking may begin only after robot-ready acknowledgement.")
        self._phase = StartSyncPhase.TRACKING
        return StartSyncDecision(self._phase, True, self._start_xy_mm, "tracking_epoch_started")

    def _clear_candidate(self) -> None:
        self._phase = StartSyncPhase.WAIT_FOR_STABLE_START
        self._window.clear()
        self._start_xy_mm = None

    @staticmethod
    def _distance(first: tuple[float, float], second: tuple[float, float]) -> float:
        return math.hypot(first[0] - second[0], first[1] - second[1])

    @staticmethod
    def _mean_xy(events: list[RealtimeTaskEvent]) -> tuple[float, float]:
        return (
            sum(float(event.x_mm) for event in events) / len(events),
            sum(float(event.y_mm) for event in events) / len(events),
        )
