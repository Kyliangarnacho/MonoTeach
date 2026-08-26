"""Deterministic ReplaySource-to-Cartesian-planner trace artifact.

The JSON emitted here is intentionally a *causality trace*, not a live IPC
protocol.  Python proves that ReplaySource releases points progressively and
that the rolling planner only commits segments after lookahead.  MATLAB then
uses the recorded arrivals as a deterministic virtual camera schedule while it
performs its own IK, q(t) planning, and 100 Hz execution simulation.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import json
from pathlib import Path

from stage2.workspace_trajectory import WorkspaceTrajectory2D

from .planned_segment_queue import PlannedSegmentQueue
from .replay_source import ReplaySource
from .rolling_cartesian_planner import (
    PlannedCartesianSegment,
    PlannerBarrier,
    RollingCartesianPlanner,
)
from .stream_contract import RealtimeTaskEvent
from .startup_sync import StartupReplaySchedule


TRACE_SCHEMA_VERSION = "stage3.4_replay_execution_trace_v1"


@dataclass(frozen=True)
class ReplayExecutionTrace:
    """Derived, JSON-serializable evidence for one finite replay run."""

    source_trajectory_id: str
    playback_speed: float
    source_events: tuple[RealtimeTaskEvent, ...]
    planned_segments: tuple[PlannedCartesianSegment, ...]
    barriers: tuple[PlannerBarrier, ...]
    queue_depth_after_enqueue: tuple[int, ...]
    startup_schedule: StartupReplaySchedule | None = None

    @property
    def source_finished_replay_t_ms(self) -> float:
        return 0.0 if not self.source_events else self.source_events[-1].replay_t_ms

    def to_dict(self) -> dict[str, object]:
        """Produce plain JSON data; no canonical WorkspaceTrajectory is mutated."""
        payload: dict[str, object] = {
            "schema_version": TRACE_SCHEMA_VERSION,
            "source_trajectory_id": self.source_trajectory_id,
            "playback_speed": self.playback_speed,
            "source_finished_replay_t_ms": self.source_finished_replay_t_ms,
            "source_events": [_task_event_dict(event) for event in self.source_events],
            "planned_segments": [_planned_segment_dict(segment) for segment in self.planned_segments],
            "barriers": [_barrier_dict(barrier) for barrier in self.barriers],
            "queue_depth_after_enqueue": list(self.queue_depth_after_enqueue),
        }
        if self.startup_schedule is not None:
            payload["startup"] = {
                "enabled": True,
                "prepare_duration_ms": self.startup_schedule.prepare_duration_ms,
                "prepare_sample_period_ms": self.startup_schedule.prepare_sample_period_ms,
                "tracking_start_replay_t_ms": self.startup_schedule.prepare_duration_ms,
                "start_source_index": 0,
                "prepare_observations": _prepare_observation_dicts(
                    self.source_events[0], self.startup_schedule
                ) if self.source_events else [],
            }
        return payload


def build_replay_execution_trace(
    trajectory: WorkspaceTrajectory2D,
    *,
    playback_speed: float = 1.0,
    startup_schedule: StartupReplaySchedule | None = None,
) -> ReplayExecutionTrace:
    """Run the actual offline ReplaySource event by event through Task 3/4.

    Advancing to each source event's replay due time deliberately makes the
    pointwise release observable in tests.  It does not bulk-inject the input
    trajectory into the planner.
    """
    if not isinstance(trajectory, WorkspaceTrajectory2D):
        raise TypeError("trajectory must be a WorkspaceTrajectory2D.")
    if startup_schedule is not None and not isinstance(startup_schedule, StartupReplaySchedule):
        raise TypeError("startup_schedule must be a StartupReplaySchedule or None.")
    source = ReplaySource(trajectory, playback_speed=playback_speed)
    planner = RollingCartesianPlanner()
    queue = PlannedSegmentQueue()
    events: list[RealtimeTaskEvent] = []
    segments: list[PlannedCartesianSegment] = []
    barriers: list[PlannerBarrier] = []
    queue_depths: list[int] = []

    for sample in trajectory.samples:
        due_replay_t_ms = sample.t_ms / playback_speed
        for arrival in source.advance_to(due_replay_t_ms):
            event = RealtimeTaskEvent.from_realtime_workspace_sample(arrival)
            if startup_schedule is not None:
                # ReplaySource still releases progressively.  The derived
                # pre-roll reserves time for Home-to-Start before formal P0.
                event = replace(
                    event,
                    replay_t_ms=event.replay_t_ms + startup_schedule.prepare_duration_ms,
                )
            events.append(event)
            result = planner.push(event)
            for segment in result.segments:
                queue.enqueue(segment, event.replay_t_ms)
                segments.append(segment)
                queue_depths.append(queue.depth)
            barriers.extend(result.barriers)

    # Only ReplaySource EOF authorizes terminal closure.  An ordinary live
    # timeout must become STALE_INPUT later, never an implicit terminal stop.
    final_replay_t_ms = source.replay_elapsed_ms
    if startup_schedule is not None:
        final_replay_t_ms += startup_schedule.prepare_duration_ms
    final_result = planner.finish(final_replay_t_ms)
    for segment in final_result.segments:
        queue.enqueue(segment, final_replay_t_ms)
        segments.append(segment)
        queue_depths.append(queue.depth)
    barriers.extend(final_result.barriers)

    return ReplayExecutionTrace(
        source_trajectory_id=trajectory.metadata.source_trajectory_id,
        playback_speed=playback_speed,
        source_events=tuple(events),
        planned_segments=tuple(segments),
        barriers=tuple(barriers),
        queue_depth_after_enqueue=tuple(queue_depths),
        startup_schedule=startup_schedule,
    )


def write_replay_execution_trace_json(trace: ReplayExecutionTrace, path: str | Path) -> Path:
    """Persist a derived trace at a caller-selected path, leaving source JSON intact."""
    if not isinstance(trace, ReplayExecutionTrace):
        raise TypeError("trace must be a ReplayExecutionTrace.")
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(trace.to_dict(), indent=2) + "\n", encoding="utf-8")
    return output


def _task_event_dict(event: RealtimeTaskEvent) -> dict[str, object]:
    return {
        "source_index": event.source_index,
        "source_t_ms": event.source_t_ms,
        "replay_t_ms": event.replay_t_ms,
        "valid": event.valid,
        "inside_workspace": event.inside_workspace,
        "x_mm": event.x_mm,
        "y_mm": event.y_mm,
        "pen_state": event.pen_state,
        "stroke_id": event.stroke_id,
        "invalid_reason": event.invalid_reason,
    }


def _planned_segment_dict(segment: PlannedCartesianSegment) -> dict[str, object]:
    profile = segment.profile
    return {
        "segment_index": segment.segment_index,
        "source_start_index": segment.source_start_index,
        "source_end_index": segment.source_end_index,
        "motion_mode": segment.motion_mode.value,
        "stroke_id": segment.stroke_id,
        "start_state_method": segment.start_state_method,
        "end_state_method": segment.end_state_method,
        "finalization_reason": segment.finalization_reason,
        "time_contract": asdict(segment.time_contract),
        "profile": {
            "start": asdict(profile.start),
            "end": asdict(profile.end),
            "coefficients_x": list(profile.coefficients_x),
            "coefficients_y": list(profile.coefficients_y),
        },
    }


def _barrier_dict(barrier: PlannerBarrier) -> dict[str, object]:
    return {
        "kind": barrier.kind.value,
        "source_index": barrier.source_index,
        "source_t_ms": barrier.source_t_ms,
        "replay_t_ms": barrier.replay_t_ms,
        "reason": barrier.reason,
    }


def _prepare_observation_dicts(
    first_tracking_event: RealtimeTaskEvent,
    schedule: StartupReplaySchedule,
) -> list[dict[str, object]]:
    """Describe derived stationary observations without touching canonical P0."""
    if not first_tracking_event.planning_eligible:
        return []
    return [
        {
            "phase": "PREPARE",
            "derived_from_source_index": first_tracking_event.source_index,
            "capture_t_ms": replay_t_ms,
            "replay_t_ms": replay_t_ms,
            "x_mm": first_tracking_event.x_mm,
            "y_mm": first_tracking_event.y_mm,
            "pen_state": first_tracking_event.pen_state,
        }
        for replay_t_ms in schedule.prepare_replay_times_ms()
    ]
