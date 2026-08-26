"""Thin live adapter from gated camera events to committed Cartesian segments."""
from __future__ import annotations

from dataclasses import dataclass

from .live_protocol import LiveProtocolClient
from .rolling_cartesian_planner import PlannedCartesianSegment, PlannerBarrier, RollingCartesianPlanner
from .stream_contract import RealtimeTaskEvent


@dataclass(frozen=True)
class LivePlanningUpdate:
    event: RealtimeTaskEvent
    segments: tuple[PlannedCartesianSegment, ...]
    barriers: tuple[PlannerBarrier, ...]


class LivePlanningPipeline:
    """Own planning state; network delivery is an optional narrow side effect."""
    def __init__(self, session_id: str, client: LiveProtocolClient | None = None) -> None:
        self.session_id, self._client, self._planner = session_id, client, RollingCartesianPlanner()

    def push(self, event: RealtimeTaskEvent) -> LivePlanningUpdate:
        if self._client is not None:
            self._client.send(self.session_id, "SOURCE_EVENT", _source_payload(event))
        result = self._planner.push(event)
        self._send(result.segments, result.barriers)
        return LivePlanningUpdate(event, result.segments, result.barriers)

    def finish(self, available_t_ms: float) -> tuple[PlannedCartesianSegment, ...]:
        result = self._planner.finish(available_t_ms)
        self._send(result.segments, result.barriers)
        if self._client is not None: self._client.send(self.session_id, "END_OF_STREAM", {"available_t_ms": available_t_ms})
        return result.segments

    def stop_for_stationary_dwell(self, available_t_ms: float) -> tuple[PlannedCartesianSegment, ...]:
        """Flush one valid run to rest before the first post-dwell motion.

        No TCP ``BARRIER`` is sent: the hand remained valid and its pen state
        did not change.  MATLAB receives only the final ordinary segment, then
        later receives the next run's source events and segments.
        """
        result = self._planner.stop_for_stationary_dwell(available_t_ms)
        self._send(result.segments, result.barriers)
        return result.segments

    def heartbeat(self, *, capture_t_ms: float, available_t_ms: float, valid: bool) -> None:
        """Send source-health evidence without creating a Cartesian waypoint."""
        if self._client is not None:
            self._client.send(self.session_id, "SOURCE_HEARTBEAT", {
                "capture_t_ms": float(capture_t_ms),
                "available_t_ms": float(available_t_ms),
                "valid": bool(valid),
            })

    def _send(self, segments: tuple[PlannedCartesianSegment, ...], barriers: tuple[PlannerBarrier, ...]) -> None:
        if self._client is None: return
        # A pen transition must arrive after the terminal segment of the old
        # run but before the first segment of the new run.  TCP preserves this
        # order, allowing MATLAB to insert lift/transfer/lower chunks without
        # clearing already committed motion.
        ordered: list[tuple[str, object]] = [("PLANNED_SEGMENT", segment) for segment in segments]
        for barrier in barriers:
            insertion = next(
                (i for i, (_kind, item) in enumerate(ordered)
                 if isinstance(item, PlannedCartesianSegment) and item.source_end_index >= barrier.source_index),
                len(ordered),
            )
            ordered.insert(insertion, ("BARRIER", barrier))
        for kind, item in ordered:
            if kind == "PLANNED_SEGMENT":
                self._client.send(self.session_id, kind, _segment_payload(item))  # type: ignore[arg-type]
            else:
                self._client.send(self.session_id, kind, _barrier_payload(item))  # type: ignore[arg-type]


def _source_payload(event: RealtimeTaskEvent) -> dict[str, object]:
    """The TCP payload is derived; the immutable event remains the local fact."""
    return {
        "source_index": event.source_index, "source_t_ms": event.source_t_ms, "replay_t_ms": event.replay_t_ms,
        "valid": event.valid, "inside_workspace": event.inside_workspace, "x_mm": event.x_mm, "y_mm": event.y_mm, "z_task_mm": event.z_task_mm,
        "pen_state": event.pen_state, "stroke_id": event.stroke_id, "invalid_reason": event.invalid_reason,
    }


def _segment_payload(segment: PlannedCartesianSegment) -> dict[str, object]:
    from dataclasses import asdict

    profile = segment.profile
    return {
        "segment_index": segment.segment_index, "source_start_index": segment.source_start_index,
        "source_end_index": segment.source_end_index, "motion_mode": segment.motion_mode.value,
        "stroke_id": segment.stroke_id, "start_state_method": segment.start_state_method,
        "end_state_method": segment.end_state_method, "finalization_reason": segment.finalization_reason,
        "time_contract": asdict(segment.time_contract),
        "profile": {"start": asdict(profile.start), "end": asdict(profile.end),
                    "coefficients_x": list(profile.coefficients_x), "coefficients_y": list(profile.coefficients_y)},
    }


def _barrier_payload(barrier: PlannerBarrier) -> dict[str, object]:
    return {"kind": barrier.kind.value, "source_index": barrier.source_index,
            "source_t_ms": barrier.source_t_ms, "replay_t_ms": barrier.replay_t_ms,
            "reason": barrier.reason}
