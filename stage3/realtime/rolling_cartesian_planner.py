"""Causal Cartesian planning from timestamped realtime task events.

This module is deliberately *not* another source of canonical trajectory
data.  It consumes immutable :class:`RealtimeTaskEvent` arrivals and creates
new, derived quintic segments only when enough future evidence is visible.

For an interior source segment ``P(i) -> P(i+1)`` we retain four observations:

``P(i-1), P(i), P(i+1), P(i+2)``.

The two endpoint states are estimated with two overlapping local quadratics:

* ``P(i-1), P(i), P(i+1)`` defines ``p/v/a`` at ``P(i)``;
* ``P(i), P(i+1), P(i+2)`` defines ``p/v/a`` at ``P(i+1)``.

Those temporary quadratics are *state estimators*, not the final motion
curve.  The final curve is the existing Cartesian quintic profile that joins
the two estimated endpoint states.  Sharing an estimate at each waypoint lets
neighbouring quintics meet with identical position, velocity, and acceleration.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .cartesian_quintic_profile import (
    CartesianQuinticProfile,
    CartesianQuinticProfileGenerator,
    CartesianTrajectorySample,
    CartesianWaypoint,
)
from .source_kinematics import SourceKinematicEstimate, estimate_source_center
from .stream_contract import MotionMode, RealtimeTaskEvent, SegmentTimeContract


class PlannerBarrierKind(str, Enum):
    """A planning boundary that must split, rather than join, trajectory runs."""

    OBSERVATION_INVALID = "OBSERVATION_INVALID"
    OUTSIDE_WORKSPACE = "OUTSIDE_WORKSPACE"
    PEN_SEMANTIC_TRANSITION = "PEN_SEMANTIC_TRANSITION"


@dataclass(frozen=True)
class PlannerBarrier:
    """Evidence that closed one Cartesian run without connecting the next one."""

    kind: PlannerBarrierKind
    source_index: int
    source_t_ms: float
    replay_t_ms: float
    reason: str


@dataclass(frozen=True)
class PlannedCartesianSegment:
    """One immutable, causally-finalized Cartesian quintic segment.

    ``profile`` is evaluated in the original source-time coordinate solely to
    preserve the demonstrated local shape and source derivatives.  Its
    ``time_contract`` explicitly records that later robot execution is allowed
    to allocate a different duration.
    """

    segment_index: int
    source_start_index: int
    source_end_index: int
    motion_mode: MotionMode
    stroke_id: int | None
    profile: CartesianQuinticProfile
    time_contract: SegmentTimeContract
    start_state_method: str
    end_state_method: str
    finalization_reason: str

    def __post_init__(self) -> None:
        if self.segment_index < 0:
            raise ValueError("segment_index must be non-negative.")
        if self.source_start_index < 0 or self.source_end_index <= self.source_start_index:
            raise ValueError("source segment indices must be increasing non-negative integers.")
        if not isinstance(self.motion_mode, MotionMode):
            raise TypeError("motion_mode must be a MotionMode.")
        if not isinstance(self.profile, CartesianQuinticProfile):
            raise TypeError("profile must be a CartesianQuinticProfile.")
        if not isinstance(self.time_contract, SegmentTimeContract):
            raise TypeError("time_contract must be a SegmentTimeContract.")
        if self.profile.start.t_ms != self.time_contract.source_start_t_ms or (
            self.profile.end.t_ms != self.time_contract.source_end_t_ms
        ):
            raise ValueError("Profile source times must agree with time_contract.")
        if self.finalization_reason not in {"lookahead", "terminal_stop"}:
            raise ValueError("finalization_reason must be lookahead or terminal_stop.")

    def sample_uniform(self, sample_rate_hz: float = 100.0) -> tuple[CartesianTrajectorySample, ...]:
        """Sample the *derived* Cartesian profile; never alters source events."""
        return self.profile.sample_uniform(sample_rate_hz)


@dataclass(frozen=True)
class PlanningResult:
    """All newly created outputs caused by exactly one incoming event or EOF."""

    segments: tuple[PlannedCartesianSegment, ...] = ()
    barriers: tuple[PlannerBarrier, ...] = ()


class RollingCartesianPlanner:
    """Finalize short quintics only after causal one-observation lookahead.

    The first source segment starts from rest because it has no historical
    predecessor.  At a normal interior waypoint the same centered estimate is
    reused by the segment on its left and right.  At EOF or a barrier, the
    pending final segment is retained and closed with an explicit zero-velocity
    / zero-acceleration terminal condition instead of being silently dropped.
    """

    def __init__(self) -> None:
        self._generator = CartesianQuinticProfileGenerator()
        self._run: list[RealtimeTaskEvent] = []
        self._next_segment_index = 0
        self._last_source_t_ms: float | None = None

    @property
    def pending_events(self) -> tuple[RealtimeTaskEvent, ...]:
        """Unfinalized eligible observations retained for lookahead or tail closure."""
        return tuple(self._run)

    def push(self, event: RealtimeTaskEvent) -> PlanningResult:
        """Consume one source arrival and emit only segments now safe to commit.

        A planner may only use an event after ReplaySource released it.  Thus an
        interior segment ending at ``P(i+1)`` becomes available at the replay
        instant where ``P(i+2)`` arrives; this method records that instant in
        ``SegmentTimeContract.available_replay_t_ms``.
        """
        if not isinstance(event, RealtimeTaskEvent):
            raise TypeError("event must be a RealtimeTaskEvent.")
        if self._last_source_t_ms is not None and event.source_t_ms <= self._last_source_t_ms:
            raise ValueError("RealtimeTaskEvent source_t_ms must strictly increase globally.")
        self._last_source_t_ms = event.source_t_ms

        if not event.planning_eligible:
            tail = self._flush_terminal(event.replay_t_ms)
            return PlanningResult(
                segments=tail,
                barriers=(self._observation_barrier(event),),
            )

        if self._run and not self._same_semantics(self._run[-1], event):
            # The new pen state is visible now.  It terminates the old run, but
            # it is also retained as the first observation of the new semantic
            # run; neither side is interpolated through the state change.
            tail = self._flush_terminal(event.replay_t_ms)
            transition = PlannerBarrier(
                PlannerBarrierKind.PEN_SEMANTIC_TRANSITION,
                event.source_index,
                event.source_t_ms,
                event.replay_t_ms,
                "pen_state_or_stroke_id_changed",
            )
            next_result = self._push_eligible(event)
            return PlanningResult(
                segments=tail + next_result.segments,
                barriers=(transition,),
            )

        return self._push_eligible(event)

    def finish(self, replay_t_ms: float) -> PlanningResult:
        """Explicitly close a known finite replay without inventing a new point.

        Live operation should not call this merely because it has waited a
        while.  It is for deterministic ReplaySource EOF, where the caller can
        prove that no fourth point will arrive.
        """
        if replay_t_ms < 0.0:
            raise ValueError("replay_t_ms must be non-negative.")
        return PlanningResult(segments=self._flush_terminal(float(replay_t_ms)))

    def stop_for_stationary_dwell(self, replay_t_ms: float) -> PlanningResult:
        """Close the current valid run at rest without inventing a barrier.

        A sustained stationary hand is neither perception loss nor pen-state
        change.  It must nevertheless end the local derivative window: using
        a same-position point separated by several seconds as quadratic
        support can impose contradictory endpoint derivatives and make a
        quintic overshoot.  The next moved event therefore starts a fresh run
        from explicit zero velocity/acceleration.
        """
        if replay_t_ms < 0.0:
            raise ValueError("replay_t_ms must be non-negative.")
        return PlanningResult(segments=self._flush_terminal(float(replay_t_ms)))

    def reset(self) -> None:
        """Clear derived planner state; canonical source samples remain untouched."""
        self._run.clear()
        self._next_segment_index = 0
        self._last_source_t_ms = None

    def _push_eligible(self, event: RealtimeTaskEvent) -> PlanningResult:
        self._run.append(event)
        if len(self._run) < 3:
            return PlanningResult()

        if len(self._run) == 3:
            first, second, following = self._run
            # No P(-1) exists at the beginning of a run.  Declaring rest is an
            # explicit boundary policy, not a derivative estimate disguised as
            # a measurement.
            start = self._zero_waypoint(first)
            end_estimate = estimate_source_center(first, second, following)
            segment = self._build_segment(
                first,
                second,
                start,
                self._waypoint_from_estimate(end_estimate),
                available_replay_t_ms=following.replay_t_ms,
                start_method="run_start_zero_state",
                end_method=end_estimate.method,
                finalization_reason="lookahead",
            )
            return PlanningResult(segments=(segment,))

        previous, start_event, end_event, following = self._run[-4:]
        start_estimate = estimate_source_center(previous, start_event, end_event)
        end_estimate = estimate_source_center(start_event, end_event, following)
        segment = self._build_segment(
            start_event,
            end_event,
            self._waypoint_from_estimate(start_estimate),
            self._waypoint_from_estimate(end_estimate),
            available_replay_t_ms=following.replay_t_ms,
            start_method=start_estimate.method,
            end_method=end_estimate.method,
            finalization_reason="lookahead",
        )
        # Three observations are enough to estimate the next shared start
        # state.  Discarding older points bounds memory without losing future
        # causal information.
        self._run = self._run[-3:]
        return PlanningResult(segments=(segment,))

    def _flush_terminal(self, available_replay_t_ms: float) -> tuple[PlannedCartesianSegment, ...]:
        if len(self._run) < 2:
            self._run.clear()
            return ()

        if len(self._run) == 2:
            start_event, end_event = self._run
            start = self._zero_waypoint(start_event)
            start_method = "run_start_zero_state"
        else:
            previous, start_event, end_event = self._run[-3:]
            start_estimate = estimate_source_center(previous, start_event, end_event)
            start = self._waypoint_from_estimate(start_estimate)
            start_method = start_estimate.method

        # EOF and a barrier mean no valid successor exists.  The only honest
        # terminal policy is to reach the final observed point and stop there;
        # it neither drops the tail nor predicts a point across the boundary.
        end = self._zero_waypoint(end_event)
        segment = self._build_segment(
            start_event,
            end_event,
            start,
            end,
            available_replay_t_ms=available_replay_t_ms,
            start_method=start_method,
            end_method="terminal_zero_state",
            finalization_reason="terminal_stop",
        )
        self._run.clear()
        return (segment,)

    def _build_segment(
        self,
        start_event: RealtimeTaskEvent,
        end_event: RealtimeTaskEvent,
        start: CartesianWaypoint,
        end: CartesianWaypoint,
        *,
        available_replay_t_ms: float,
        start_method: str,
        end_method: str,
        finalization_reason: str,
    ) -> PlannedCartesianSegment:
        if start_event.motion_mode is not end_event.motion_mode or start_event.stroke_id != end_event.stroke_id:
            raise ValueError("A Cartesian segment cannot cross a pen semantic boundary.")
        profile = self._generator.build(start, end)
        segment = PlannedCartesianSegment(
            segment_index=self._next_segment_index,
            source_start_index=start_event.source_index,
            source_end_index=end_event.source_index,
            motion_mode=start_event.motion_mode,
            stroke_id=start_event.stroke_id,
            profile=profile,
            time_contract=SegmentTimeContract(
                start_event.source_t_ms,
                end_event.source_t_ms,
                available_replay_t_ms,
            ),
            start_state_method=start_method,
            end_state_method=end_method,
            finalization_reason=finalization_reason,
        )
        self._next_segment_index += 1
        return segment

    @staticmethod
    def _waypoint_from_estimate(estimate: SourceKinematicEstimate) -> CartesianWaypoint:
        return CartesianWaypoint(
            t_ms=estimate.source_t_ms,
            x_mm=estimate.position_mm[0],
            y_mm=estimate.position_mm[1],
            velocity_mm_s=estimate.velocity_mm_s,
            acceleration_mm_s2=estimate.acceleration_mm_s2,
        )

    @staticmethod
    def _zero_waypoint(event: RealtimeTaskEvent) -> CartesianWaypoint:
        assert event.x_mm is not None and event.y_mm is not None
        return CartesianWaypoint(
            t_ms=event.source_t_ms,
            x_mm=event.x_mm,
            y_mm=event.y_mm,
            velocity_mm_s=(0.0, 0.0),
            acceleration_mm_s2=(0.0, 0.0),
        )

    @staticmethod
    def _same_semantics(left: RealtimeTaskEvent, right: RealtimeTaskEvent) -> bool:
        return left.motion_mode is right.motion_mode and left.stroke_id == right.stroke_id

    @staticmethod
    def _observation_barrier(event: RealtimeTaskEvent) -> PlannerBarrier:
        if not event.valid:
            kind = PlannerBarrierKind.OBSERVATION_INVALID
            reason = event.invalid_reason or "invalid_observation"
        else:
            kind = PlannerBarrierKind.OUTSIDE_WORKSPACE
            reason = "outside_workspace"
        return PlannerBarrier(kind, event.source_index, event.source_t_ms, event.replay_t_ms, reason)
