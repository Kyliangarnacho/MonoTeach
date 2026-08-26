"""Task 1/2 contracts for Stage 3.4 stream semantics and source derivatives."""

import pytest

from stage2.workspace_trajectory import WorkspaceTrajectorySample
from stage3.realtime.realtime_sample import RealtimeWorkspaceSample
from stage3.realtime.source_kinematics import (
    estimate_source_center,
    estimate_source_end,
    estimate_source_start,
)
from stage3.realtime.stream_contract import (
    BarrierType,
    MotionMode,
    RealtimeTaskEvent,
    SegmentTimeContract,
    StreamBarrier,
)


def event(
    t_ms: float,
    x_mm: float,
    y_mm: float,
    index: int,
    *,
    pen_state: str = "DOWN",
    stroke_id: int | None = 1,
) -> RealtimeTaskEvent:
    return RealtimeTaskEvent(
        source_index=index,
        source_t_ms=t_ms,
        replay_t_ms=t_ms / 2.0,
        valid=True,
        inside_workspace=True,
        x_mm=x_mm,
        y_mm=y_mm,
        pen_state=pen_state,
        stroke_id=stroke_id,
    )


def test_task_event_preserves_source_and_replay_clocks_from_realtime_sample():
    workspace = WorkspaceTrajectorySample(100.0, True, 10.0, 20.0, True)
    arrival = RealtimeWorkspaceSample(3, 50.0, workspace)
    task = RealtimeTaskEvent.from_realtime_workspace_sample(arrival)
    assert task.source_index == 3
    assert task.source_t_ms == pytest.approx(100.0)
    assert task.replay_t_ms == pytest.approx(50.0)
    assert task.pen_state == "DOWN"
    assert task.motion_mode is MotionMode.DRAW
    assert task.planning_eligible
    assert workspace.t_ms == pytest.approx(100.0)


def test_valid_pen_up_is_lifted_follow_not_an_observation_barrier():
    task = event(20.0, 1.0, 2.0, 0, pen_state="UP", stroke_id=None)
    assert task.planning_eligible
    assert task.motion_mode is MotionMode.LIFTED_FOLLOW
    assert task.observation_barrier_type is None


def test_invalid_and_outside_events_create_distinct_observation_barriers():
    invalid = RealtimeTaskEvent(0, 10.0, 10.0, False, False, None, None, invalid_reason="no_hand")
    outside = RealtimeTaskEvent(1, 20.0, 20.0, True, False, 250.0, 20.0)
    assert StreamBarrier.from_task_event(invalid).barrier_type is BarrierType.OBSERVATION_INVALID
    assert StreamBarrier.from_task_event(invalid).reason == "no_hand"
    assert StreamBarrier.from_task_event(outside).barrier_type is BarrierType.OUTSIDE_WORKSPACE
    assert outside.motion_mode is MotionMode.HOLD
    assert not outside.planning_eligible


def test_segment_time_contract_never_equates_source_and_execution_clocks():
    timing = SegmentTimeContract(100.0, 140.0, 220.0, 1.5, 0.12)
    assert timing.source_end_t_ms - timing.source_start_t_ms == pytest.approx(40.0)
    assert timing.available_replay_t_ms == pytest.approx(220.0)
    assert timing.execution_end_s == pytest.approx(1.62)


def test_nonuniform_source_timestamp_quadratic_recovers_center_velocity_and_acceleration():
    estimate = estimate_source_center(
        event(0.0, 0.0, 0.0, 0),
        event(100.0, 1.0, 1.0, 1),
        event(300.0, 9.0, 3.0, 2),
    )
    assert estimate.position_mm == pytest.approx((1.0, 1.0))
    assert estimate.velocity_mm_s == pytest.approx((20.0, 10.0))
    assert estimate.acceleration_mm_s2 == pytest.approx((200.0, 0.0))


def test_three_source_points_also_define_start_and_end_states_without_p4():
    first = event(0.0, 0.0, 0.0, 0)
    center = event(100.0, 1.0, 1.0, 1)
    third = event(300.0, 9.0, 3.0, 2)
    start = estimate_source_start(first, center, third)
    end = estimate_source_end(first, center, third)
    assert start.velocity_mm_s == pytest.approx((0.0, 10.0))
    assert start.acceleration_mm_s2 == pytest.approx((200.0, 0.0))
    assert end.velocity_mm_s == pytest.approx((60.0, 10.0))
    assert end.acceleration_mm_s2 == pytest.approx((200.0, 0.0))


def test_derivative_estimation_rejects_observation_and_pen_semantic_barriers():
    first = event(0.0, 0.0, 0.0, 0)
    invalid = RealtimeTaskEvent(1, 100.0, 100.0, False, False, None, None, invalid_reason="no_hand")
    third = event(200.0, 2.0, 0.0, 2)
    with pytest.raises(ValueError, match="observation barrier"):
        estimate_source_center(first, invalid, third)
    lifted = event(100.0, 1.0, 0.0, 1, pen_state="UP", stroke_id=None)
    with pytest.raises(ValueError, match="pen semantic transition"):
        estimate_source_center(first, lifted, third)

