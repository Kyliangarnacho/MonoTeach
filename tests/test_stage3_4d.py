"""Stage 3.4D deterministic contracts: causal planning and replay trace."""

from __future__ import annotations

import json

import pytest

from stage2.workspace_trajectory import (
    WorkspaceTrajectory2D,
    WorkspaceTrajectoryMetadata,
    WorkspaceTrajectorySample,
)
from stage3.realtime.cartesian_quintic_profile import CartesianQuinticProfileGenerator, CartesianWaypoint
from stage3.realtime.planned_segment_queue import PlannedSegmentQueue
from stage3.realtime.replay_source import ReplaySource
from stage3.realtime.replay_execution_trace import (
    TRACE_SCHEMA_VERSION,
    build_replay_execution_trace,
    write_replay_execution_trace_json,
)
from stage3.realtime.rolling_cartesian_planner import (
    PlannerBarrierKind,
    RollingCartesianPlanner,
)
from stage3.realtime.stream_contract import RealtimeTaskEvent


def event(index: int, t_ms: float, x_mm: float, y_mm: float, *, pen_state: str = "DOWN", stroke_id: int | None = 1) -> RealtimeTaskEvent:
    return RealtimeTaskEvent(
        source_index=index,
        source_t_ms=t_ms,
        replay_t_ms=t_ms,
        valid=True,
        inside_workspace=True,
        x_mm=x_mm,
        y_mm=y_mm,
        pen_state=pen_state,
        stroke_id=stroke_id,
    )


def invalid(index: int, t_ms: float) -> RealtimeTaskEvent:
    return RealtimeTaskEvent(index, t_ms, t_ms, False, False, None, None, invalid_reason="no_hand")


def test_four_observation_window_uses_two_overlapping_quadratic_states_and_one_lookahead():
    # x = 100 * t^2 (with t in seconds) makes the expected derivatives exact:
    # at 0.1 s, v=20 mm/s and a=200 mm/s^2; at 0.2 s, v=40 mm/s.
    planner = RollingCartesianPlanner()
    assert planner.push(event(0, 0.0, 0.0, 0.0)).segments == ()
    assert planner.push(event(1, 100.0, 1.0, 1.0)).segments == ()
    first = planner.push(event(2, 200.0, 4.0, 2.0)).segments[0]
    second = planner.push(event(3, 300.0, 9.0, 3.0)).segments[0]

    first_end = first.profile.sample_at(100.0)
    second_start = second.profile.sample_at(100.0)
    second_end = second.profile.sample_at(200.0)
    assert first.time_contract.available_replay_t_ms == pytest.approx(200.0)
    assert second.time_contract.available_replay_t_ms == pytest.approx(300.0)
    assert first_end.velocity_mm_s == pytest.approx((20.0, 10.0))
    assert first_end.acceleration_mm_s2 == pytest.approx((200.0, 0.0))
    # The exact same P1 state is used on both sides of the join.
    assert second_start.velocity_mm_s == pytest.approx(first_end.velocity_mm_s)
    assert second_start.acceleration_mm_s2 == pytest.approx(first_end.acceleration_mm_s2)
    assert second_end.velocity_mm_s == pytest.approx((40.0, 10.0))
    assert second_end.acceleration_mm_s2 == pytest.approx((200.0, 0.0))


def test_barrier_keeps_pending_tail_then_stops_and_never_connects_new_run():
    planner = RollingCartesianPlanner()
    planner.push(event(0, 0.0, 0.0, 0.0))
    planner.push(event(1, 100.0, 10.0, 0.0))
    committed = planner.push(event(2, 200.0, 20.0, 0.0)).segments
    result = planner.push(invalid(3, 300.0))

    assert committed[0].source_start_index == 0
    assert committed[0].source_end_index == 1
    assert len(result.segments) == 1
    tail = result.segments[0]
    assert (tail.source_start_index, tail.source_end_index) == (1, 2)
    assert tail.finalization_reason == "terminal_stop"
    terminal = tail.profile.sample_at(200.0)
    assert terminal.velocity_mm_s == pytest.approx((0.0, 0.0))
    assert terminal.acceleration_mm_s2 == pytest.approx((0.0, 0.0), abs=1e-9)
    assert result.barriers[0].kind is PlannerBarrierKind.OBSERVATION_INVALID
    assert planner.pending_events == ()

    planner.push(event(4, 400.0, 100.0, 0.0))
    planner.push(event(5, 500.0, 110.0, 0.0))
    restarted = planner.push(event(6, 600.0, 120.0, 0.0)).segments[0]
    assert (restarted.source_start_index, restarted.source_end_index) == (4, 5)


def test_pen_transition_is_a_semantic_barrier_even_when_coordinates_are_valid():
    planner = RollingCartesianPlanner()
    planner.push(event(0, 0.0, 0.0, 0.0))
    planner.push(event(1, 100.0, 10.0, 0.0))
    transition = planner.push(event(2, 200.0, 20.0, 0.0, pen_state="UP", stroke_id=None))

    assert len(transition.segments) == 1
    assert transition.segments[0].finalization_reason == "terminal_stop"
    assert transition.barriers[0].kind is PlannerBarrierKind.PEN_SEMANTIC_TRANSITION
    assert planner.pending_events[0].source_index == 2


def test_explicit_eof_flushes_last_segment_without_creating_future_data():
    planner = RollingCartesianPlanner()
    planner.push(event(0, 0.0, 0.0, 0.0))
    planner.push(event(1, 100.0, 10.0, 0.0))
    result = planner.finish(100.0)
    assert len(result.segments) == 1
    segment = result.segments[0]
    assert segment.time_contract.available_replay_t_ms == pytest.approx(100.0)
    assert segment.finalization_reason == "terminal_stop"


def test_fifo_rejects_reordered_committed_segments():
    planner = RollingCartesianPlanner()
    planner.push(event(0, 0.0, 0.0, 0.0))
    planner.push(event(1, 100.0, 10.0, 0.0))
    first = planner.push(event(2, 200.0, 20.0, 0.0)).segments[0]
    second = planner.push(event(3, 300.0, 30.0, 0.0)).segments[0]
    queue = PlannedSegmentQueue()
    queue.enqueue(first, 200.0)
    queue.enqueue(second, 300.0)
    assert [item.segment.segment_index for item in queue.entries] == [0, 1]
    assert queue.pop_next().segment is first
    with pytest.raises(ValueError, match="increase strictly"):
        queue.enqueue(first, 300.0)


def _trajectory() -> WorkspaceTrajectory2D:
    return WorkspaceTrajectory2D(
        WorkspaceTrajectoryMetadata("trace-test", "calibration", "workspace_2d", 190.0, 290.0),
        (
            WorkspaceTrajectorySample(0.0, True, 0.0, 0.0, True),
            WorkspaceTrajectorySample(100.0, True, 10.0, 0.0, True),
            WorkspaceTrajectorySample(200.0, True, 20.0, 0.0, True),
            WorkspaceTrajectorySample(300.0, True, 30.0, 0.0, True),
        ),
    )


def test_replay_trace_proves_progressive_release_and_writes_matlab_readable_json(tmp_path):
    trace = build_replay_execution_trace(_trajectory(), playback_speed=2.0)
    assert [event.replay_t_ms for event in trace.source_events] == pytest.approx([0.0, 50.0, 100.0, 150.0])
    assert [(segment.source_start_index, segment.source_end_index) for segment in trace.planned_segments] == [
        (0, 1),
        (1, 2),
        (2, 3),
    ]
    assert [segment.time_contract.available_replay_t_ms for segment in trace.planned_segments] == pytest.approx(
        [100.0, 150.0, 150.0]
    )
    output = write_replay_execution_trace_json(trace, tmp_path / "trace.json")
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["schema_version"] == TRACE_SCHEMA_VERSION
    assert payload["planned_segments"][1]["profile"]["start"]["velocity_mm_s"] == pytest.approx([100.0, 0.0])

def test_replay_source_remains_incremental_and_preserves_canonical_source_time():
    trajectory = _trajectory()
    source = ReplaySource(trajectory, playback_speed=2.0)

    first = source.advance_to(0.0)
    assert tuple(sample.t_ms for sample in first) == pytest.approx((0.0,))
    assert source.advance_to(49.9) == ()
    second = source.advance_to(50.0)
    assert tuple(sample.t_ms for sample in second) == pytest.approx((100.0,))
    assert second[0].workspace_sample is trajectory.samples[1]
    assert second[0].replay_time_ms == pytest.approx(50.0)


def test_active_cartesian_quintic_keeps_full_boundary_state_contract():
    start = CartesianWaypoint(
        t_ms=100.0, x_mm=10.0, y_mm=-20.0,
        velocity_mm_s=(4.0, -5.0), acceleration_mm_s2=(2.0, -3.0),
    )
    end = CartesianWaypoint(
        t_ms=1_100.0, x_mm=110.0, y_mm=30.0,
        velocity_mm_s=(-6.0, 8.0), acceleration_mm_s2=(-1.0, 3.0),
    )
    profile = CartesianQuinticProfileGenerator().build(start, end)

    assert profile.sample_at(start.t_ms).velocity_mm_s == pytest.approx(start.velocity_mm_s)
    final = profile.sample_at(end.t_ms)
    assert (final.x_mm, final.y_mm) == pytest.approx((end.x_mm, end.y_mm))
    assert final.velocity_mm_s == pytest.approx(end.velocity_mm_s)
    assert final.acceleration_mm_s2 == pytest.approx(end.acceleration_mm_s2)
