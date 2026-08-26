"""Stage 3.4E: ready handshake and derived virtual-camera PREPARE evidence."""

from __future__ import annotations

import pytest

from stage2.workspace_trajectory import (
    WorkspaceTrajectory2D,
    WorkspaceTrajectoryMetadata,
    WorkspaceTrajectorySample,
)
from stage3.realtime.replay_execution_trace import build_replay_execution_trace
from stage3.realtime.startup_sync import (
    StartSyncConfig,
    StartSyncPhase,
    StartSynchronizationController,
    StartupReplaySchedule,
)
from stage3.realtime.stream_contract import RealtimeTaskEvent


def _event(index: int, t_ms: float, x_mm: float, y_mm: float) -> RealtimeTaskEvent:
    return RealtimeTaskEvent(index, t_ms, t_ms, True, True, x_mm, y_mm, "DOWN", 1)


def _trajectory() -> WorkspaceTrajectory2D:
    return WorkspaceTrajectory2D(
        WorkspaceTrajectoryMetadata("startup-test", "calibration", "workspace_2d", 190.0, 290.0),
        (
            WorkspaceTrajectorySample(0.0, True, 10.0, 20.0, True),
            WorkspaceTrajectorySample(100.0, True, 20.0, 20.0, True),
            WorkspaceTrajectorySample(200.0, True, 30.0, 20.0, True),
        ),
    )


def test_stationary_start_is_gated_then_tracking_is_explicitly_enabled():
    sync = StartSynchronizationController(StartSyncConfig(dwell_time_ms=100.0, stationary_radius_mm=1.0))

    assert sync.observe(_event(0, 0.0, 10.0, 20.0)).planner_accepts_observation is False
    decision = sync.observe(_event(1, 100.0, 10.3, 19.9))
    assert decision.phase is StartSyncPhase.APPROACHING_START
    assert decision.start_xy_mm == pytest.approx((10.15, 19.95))

    ready = sync.notify_robot_at_start()
    assert ready.phase is StartSyncPhase.READY
    assert sync.begin_tracking().phase is StartSyncPhase.TRACKING
    assert sync.observe(_event(2, 200.0, 12.0, 20.0)).planner_accepts_observation is True


def test_hand_motion_during_approach_invalidates_candidate_instead_of_buffering_motion():
    sync = StartSynchronizationController(StartSyncConfig(dwell_time_ms=100.0, stationary_radius_mm=1.0))
    sync.observe(_event(0, 0.0, 10.0, 20.0))
    assert sync.observe(_event(1, 100.0, 10.0, 20.0)).phase is StartSyncPhase.APPROACHING_START

    moved = sync.observe(_event(2, 200.0, 15.0, 20.0))
    assert moved.phase is StartSyncPhase.WAIT_FOR_STABLE_START
    assert moved.reason == "hand_moved_before_robot_ready"
    assert moved.planner_accepts_observation is False
    assert sync.start_xy_mm is None


def test_startup_replay_is_derived_and_does_not_modify_canonical_source_time():
    trajectory = _trajectory()
    schedule = StartupReplaySchedule(prepare_duration_ms=1_000.0, prepare_sample_period_ms=250.0)
    trace = build_replay_execution_trace(trajectory, startup_schedule=schedule)
    payload = trace.to_dict()

    assert [sample.t_ms for sample in trajectory.samples] == pytest.approx([0.0, 100.0, 200.0])
    assert [event.source_t_ms for event in trace.source_events] == pytest.approx([0.0, 100.0, 200.0])
    assert [event.replay_t_ms for event in trace.source_events] == pytest.approx([1_000.0, 1_100.0, 1_200.0])
    assert payload["startup"]["tracking_start_replay_t_ms"] == pytest.approx(1_000.0)
    prepare = payload["startup"]["prepare_observations"]
    assert [entry["replay_t_ms"] for entry in prepare] == pytest.approx([0.0, 250.0, 500.0, 750.0, 1_000.0])
    assert {(entry["x_mm"], entry["y_mm"]) for entry in prepare} == {(10.0, 20.0)}
    assert all(entry["phase"] == "PREPARE" for entry in prepare)
    assert all(entry["derived_from_source_index"] == 0 for entry in prepare)
