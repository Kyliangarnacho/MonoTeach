"""Deterministic contracts for the final explicit live-follow boundary."""
from __future__ import annotations

import socket

import numpy as np

from stage2.workspace_geometry import WorkspaceCalibration, WorkspaceDefinition
import pytest

from stage3.realtime.live_follow_session import LiveFollowConfig, LiveFollowPhase, LiveFollowSession, StartNotReadyError
from stage3.realtime.live_observation import LiveWorkspaceObservation
from stage3.realtime.live_position_filter import FilteredWorkspaceEstimate
from stage3.realtime.live_protocol import LIVE_PROTOCOL_VERSION, LiveProtocolClient
from stage3.realtime.live_planning_pipeline import LivePlanningPipeline
from stage3.realtime.stream_contract import RealtimeTaskEvent


def _calibration() -> WorkspaceCalibration:
    return WorkspaceCalibration('1.0','live_follow_h',100,100,WorkspaceDefinition(200,200),np.array([[0,0],[100,0],[100,100],[0,100]]),np.array([[0,0],[200,0],[200,200],[0,200]]),np.array([[2,0,0],[0,2,0],[0,0,1]]),'test')


def _estimate(i: int, x: float | None, y: float | None, *, pen: str = 'DOWN', gesture_active: bool = False, gesture_toggled: bool = False, gesture_candidate: bool = False) -> FilteredWorkspaceEstimate:
    valid=x is not None
    raw=LiveWorkspaceObservation(i,i*100.0,i*100.0+5,valid,x,y,valid,None if valid else 'no_hand',(10,10) if valid else None,(10,10) if valid else None,pen,1 if pen=='DOWN' else None, 'L_COMMAND' if (gesture_active or gesture_candidate) else 'POINT', gesture_active, gesture_toggled, gesture_candidate)
    return FilteredWorkspaceEstimate(raw,x,y,valid,'new_continuous_run' if valid else 'invalid_observation')


def test_two_confirmations_make_relative_anchor_and_gate_jitter() -> None:
    session=LiveFollowSession(_calibration(),LiveFollowConfig(anchor_window_ms=500,operating_margin_mm=10,minimum_motion_mm=2,heartbeat_ms=200))
    for i,x in enumerate((100.0,100.5,99.8,100.2,100.0)):
        e=_estimate(i,x,100);assert session.process(e.raw_observation,e) is None
    target=session.confirm_start();assert target.x_mm==100.0
    session.notify_robot_ready();assert session.phase is LiveFollowPhase.WAIT_FOR_TEACH_CONFIRM
    e=_estimate(6,102,101);session.process(e.raw_observation,e);p0=session.confirm_teaching();assert (p0.x_mm,p0.y_mm)==(100,100)
    e=_estimate(7,103,101);assert session.process(e.raw_observation,e) is None
    e=_estimate(8,105,101);event=session.process(e.raw_observation,e);assert event is not None and (event.x_mm,event.y_mm)==(103,100)


def test_live_follow_emits_barrier_for_outside_operating_margin() -> None:
    session=LiveFollowSession(_calibration(),LiveFollowConfig(operating_margin_mm=10))
    for i in range(3):
        e=_estimate(i,100,100);session.process(e.raw_observation,e)
    session.confirm_start();session.notify_robot_ready();session.confirm_teaching()
    e=_estimate(4,3,100);event=session.process(e.raw_observation,e)
    assert event is not None and not event.valid and event.invalid_reason=='outside_operating_region'


def test_start_readiness_explains_rejection_without_changing_phase() -> None:
    session=LiveFollowSession(_calibration())
    first=_estimate(0,100,100);session.process(first.raw_observation,first)
    readiness=session.start_readiness
    assert readiness.reason=='collecting_recent_samples' and readiness.candidate_count==1
    with pytest.raises(StartNotReadyError) as error:
        session.confirm_start()
    assert error.value.readiness==readiness
    assert session.phase is LiveFollowPhase.WAIT_FOR_START_CONFIRM


def test_start_readiness_rejects_a_spread_out_p0_cloud_but_exposes_its_median() -> None:
    session = LiveFollowSession(_calibration(), LiveFollowConfig(maximum_start_spread_mm=2.0))
    for i, x in enumerate((100.0, 104.0, 100.0)):
        estimate = _estimate(i, x, 100.0)
        session.process(estimate.raw_observation, estimate)
    readiness = session.start_readiness
    assert readiness.reason == 'start_points_too_spread'
    assert readiness.median_x_mm == pytest.approx(100.0)
    assert readiness.max_spread_mm == pytest.approx(4.0)
    with pytest.raises(StartNotReadyError):
        session.confirm_start()


def test_no_hand_clears_old_start_candidates_but_edge_frame_keeps_them() -> None:
    session=LiveFollowSession(_calibration())
    for i in range(2):
        estimate=_estimate(i,100,100);session.process(estimate.raw_observation,estimate)
    assert session.start_readiness.ready
    edge=_estimate(2,5,100);session.process(edge.raw_observation,edge)
    assert session.start_readiness.reason=='inside_but_near_edge'
    assert session.start_readiness.candidate_count==2
    missing=_estimate(3,None,None);session.process(missing.raw_observation,missing)
    assert session.start_readiness.reason=='no_hand'
    assert session.start_readiness.candidate_count==0


def test_protocol_client_preserves_line_framed_message() -> None:
    listener=socket.socket();listener.bind(('127.0.0.1',0));listener.listen(1);port=listener.getsockname()[1];received=[]
    client=LiveProtocolClient(port=port);client.connect('unit')
    conn,_=listener.accept();received.append(conn.recv(4096).decode());conn.close();listener.close();client.close()
    assert received and LIVE_PROTOCOL_VERSION in received[0] and 'HELLO' in received[0]


def test_stationary_heartbeats_close_the_old_run_without_becoming_derivative_support() -> None:
    session = LiveFollowSession(_calibration(), LiveFollowConfig(minimum_motion_mm=2.0, heartbeat_ms=200.0))
    for i in range(3):
        estimate = _estimate(i, 100.0, 100.0); session.process_many(estimate.raw_observation, estimate)
    session.confirm_start(); session.notify_robot_ready(); initial = session.confirm_teaching()

    # Quiet frames advance human time but do not become source waypoints.  On
    # resumed motion they create a separate planner stop/restart boundary, so
    # a same-position point five seconds later cannot corrupt p/v/a fitting.
    for i in (4, 5, 6):
        estimate = _estimate(i, 100.3, 100.0)
        assert session.process_many(estimate.raw_observation, estimate) == ()
    moved = _estimate(7, 104.0, 100.0)
    events = session.process_many(moved.raw_observation, moved)
    assert len(events) == 1
    (motion,) = events
    assert (motion.x_mm, motion.y_mm) == (104.0, 100.0)
    boundary = session.take_motion_restart_boundary()
    assert boundary is not None
    assert boundary.source_t_ms > initial.source_t_ms
    assert session.take_motion_restart_boundary() is None


def test_live_pen_height_is_explicit_task_plane_evidence() -> None:
    session = LiveFollowSession(_calibration(), LiveFollowConfig(contact_z_task_mm=1.0, lifted_z_task_mm=13.0))
    for i in range(3):
        estimate = _estimate(i, 100.0, 100.0, pen='DOWN'); session.process_many(estimate.raw_observation, estimate)
    session.confirm_start(); session.notify_robot_ready(); down = session.confirm_teaching()
    assert down.z_task_mm == 1.0
    up = _estimate(4, 100.0, 100.0, pen='UP')
    events = session.process_many(up.raw_observation, up)
    assert events[-1].pen_state == 'UP'
    assert events[-1].z_task_mm == 13.0


def test_confirmed_l_command_is_one_pen_transition_not_a_safety_barrier() -> None:
    session = LiveFollowSession(_calibration())
    for i in range(3):
        estimate = _estimate(i, 100.0, 100.0); session.process_many(estimate.raw_observation, estimate)
    session.confirm_start(); session.notify_robot_ready(); initial = session.confirm_teaching()
    command = _estimate(4, 130.0, 140.0, pen='UP', gesture_active=True, gesture_toggled=True)
    events = session.process_many(command.raw_observation, command)
    assert len(events) == 1
    transition = events[0]
    assert transition.valid and transition.pen_state == 'UP'
    assert (transition.x_mm, transition.y_mm) == (initial.x_mm, initial.y_mm)
    held = _estimate(5, 130.0, 140.0, pen='UP', gesture_active=True)
    assert session.process_many(held.raw_observation, held) == ()


def test_l_candidate_protects_short_no_hand_from_becoming_a_spatial_barrier() -> None:
    session = LiveFollowSession(_calibration())
    for i in range(3):
        estimate = _estimate(i, 100.0, 100.0); session.process_many(estimate.raw_observation, estimate)
    session.confirm_start(); session.notify_robot_ready(); session.confirm_teaching()
    protected_missing = _estimate(4, None, None, gesture_candidate=True)
    assert session.process_many(protected_missing.raw_observation, protected_missing) == ()
    command = _estimate(5, 150.0, 150.0, pen='UP', gesture_active=True, gesture_toggled=True, gesture_candidate=True)
    events = session.process_many(command.raw_observation, command)
    assert len(events) == 1 and events[0].valid and events[0].pen_state == 'UP'


def test_confirmed_pen_command_pauses_new_spatial_events_until_matlab_ack() -> None:
    session = LiveFollowSession(_calibration())
    for i in range(3):
        estimate = _estimate(i, 100.0, 100.0)
        session.process_many(estimate.raw_observation, estimate)
    session.confirm_start(); session.notify_robot_ready(); session.confirm_teaching()

    command = _estimate(4, 130.0, 140.0, pen='UP', gesture_active=True, gesture_toggled=True)
    assert len(session.process_many(command.raw_observation, command)) == 1
    assert session.phase is LiveFollowPhase.PEN_TRANSITION
    # Physical finger motion while making/releasing the L cannot add a new
    # tracking point before MATLAB says its queued lift is actually complete.
    released = _estimate(5, 150.0, 150.0, pen='UP')
    assert session.process_many(released.raw_observation, released) == ()
    session.notify_pen_transition_ready()
    assert session.phase is LiveFollowPhase.TRACKING
    moved = _estimate(6, 154.0, 150.0, pen='UP')
    resumed = session.process_many(moved.raw_observation, moved)
    assert resumed and resumed[-1].valid and resumed[-1].pen_state == 'UP'


def test_pen_rebase_after_motion_uses_last_target_not_the_original_p0() -> None:
    """Regression: L after moving must not snap the next target back to P0."""
    session = LiveFollowSession(_calibration(), LiveFollowConfig(minimum_motion_mm=2.0))
    for i in range(3):
        estimate = _estimate(i, 100.0, 100.0)
        session.process_many(estimate.raw_observation, estimate)
    session.confirm_start(); session.notify_robot_ready(); session.confirm_teaching()

    moved = _estimate(4, 130.0, 100.0)
    ordinary = session.process_many(moved.raw_observation, moved)
    assert ordinary[-1].x_mm == pytest.approx(130.0)

    command = _estimate(5, 150.0, 135.0, pen='UP', gesture_active=True, gesture_toggled=True)
    transition = session.process_many(command.raw_observation, command)[-1]
    assert (transition.x_mm, transition.y_mm) == pytest.approx((130.0, 100.0))
    session.notify_pen_transition_ready()

    release = _estimate(6, 154.0, 135.0, pen='UP')
    # The first released POINT is the fresh mapping reference; it must not
    # itself become an unintended XY move.
    assert session.process_many(release.raw_observation, release) == ()
    released_motion = _estimate(7, 158.0, 135.0, pen='UP')
    resumed = session.process_many(released_motion.raw_observation, released_motion)[-1]
    # The hand then moves +4 mm from its released pose, hence the target moves
    # +4 mm from the pre-command target.  It must not return toward P0=100.
    assert (resumed.x_mm, resumed.y_mm) == pytest.approx((134.0, 100.0))
    assert session.mapping_offset_xy_mm == pytest.approx((-24.0, -35.0))


def test_reacquire_candidate_window_does_not_survive_an_outside_frame() -> None:
    session = LiveFollowSession(_calibration(), LiveFollowConfig(minimum_reacquire_candidates=2))
    for i in range(3):
        estimate = _estimate(i, 100.0, 100.0)
        session.process_many(estimate.raw_observation, estimate)
    session.confirm_start(); session.notify_robot_ready(); session.confirm_teaching()
    missing = _estimate(4, None, None)
    session.process_many(missing.raw_observation, missing)
    first = _estimate(5, 110.0, 100.0)
    session.process_many(first.raw_observation, first)
    outside = _estimate(6, 5.0, 100.0)
    session.process_many(outside.raw_observation, outside)
    second = _estimate(7, 120.0, 100.0)
    session.process_many(second.raw_observation, second)
    assert session.take_reacquire_target() is None


def test_visual_gap_waits_for_fresh_target_and_ack_before_seeding_new_run() -> None:
    session = LiveFollowSession(_calibration(), LiveFollowConfig(minimum_reacquire_candidates=2))
    for i in range(3):
        estimate = _estimate(i, 100.0, 100.0)
        session.process_many(estimate.raw_observation, estimate)
    session.confirm_start(); session.notify_robot_ready(); session.confirm_teaching()
    moved = _estimate(4, 105.0, 100.0); session.process_many(moved.raw_observation, moved)

    missing = _estimate(5, None, None)
    boundary = session.process_many(missing.raw_observation, missing)
    assert len(boundary) == 1 and not boundary[0].valid
    assert session.phase is LiveFollowPhase.VISION_GAP
    # Fresh frames are observed but not silently planned across the boundary.
    for i, x in ((6, 110.0), (7, 110.5)):
        estimate = _estimate(i, x, 100.0)
        assert session.process_many(estimate.raw_observation, estimate) == ()
    target = session.take_reacquire_target()
    assert target is not None and target.x_mm == pytest.approx(110.25)
    assert session.phase is LiveFollowPhase.WAIT_FOR_REACQUIRE_READY
    still_waiting = _estimate(8, 114.0, 100.0)
    assert session.process_many(still_waiting.raw_observation, still_waiting) == ()
    seed = session.notify_reacquire_ready()
    assert seed.valid and (seed.x_mm, seed.y_mm) == pytest.approx((target.x_mm, target.y_mm))
    assert session.phase is LiveFollowPhase.TRACKING


class _RecordingClient:
    def __init__(self) -> None:
        self.messages: list[tuple[str, dict[str, object]]] = []

    def send(self, _session: str, kind: str, payload: dict[str, object]) -> int:
        self.messages.append((kind, payload))
        return len(self.messages) - 1


def test_pen_transition_is_delivered_between_old_tail_and_new_run() -> None:
    client = _RecordingClient()
    pipeline = LivePlanningPipeline('unit', client)  # type: ignore[arg-type]
    down0 = RealtimeTaskEvent(0, 0, 0, True, True, 10, 10, 'DOWN', 1)
    down1 = RealtimeTaskEvent(1, 100, 100, True, True, 20, 10, 'DOWN', 1)
    up2 = RealtimeTaskEvent(2, 200, 200, True, True, 20, 10, 'UP', None, None, 12)
    pipeline.push(down0); pipeline.push(down1); pipeline.push(up2)
    kinds = [kind for kind, _payload in client.messages]
    assert kinds == ['SOURCE_EVENT', 'SOURCE_EVENT', 'SOURCE_EVENT', 'PLANNED_SEGMENT', 'BARRIER']
    assert client.messages[-1][1]['kind'] == 'PEN_SEMANTIC_TRANSITION'


def test_heartbeat_is_transport_liveness_not_a_planner_waypoint() -> None:
    client = _RecordingClient()
    pipeline = LivePlanningPipeline('unit', client)  # type: ignore[arg-type]
    pipeline.heartbeat(capture_t_ms=10, available_t_ms=15, valid=True)
    assert client.messages == [('SOURCE_HEARTBEAT', {'capture_t_ms': 10.0, 'available_t_ms': 15.0, 'valid': True})]


def test_stationary_dwell_flushes_to_rest_without_a_spatial_barrier() -> None:
    client = _RecordingClient()
    pipeline = LivePlanningPipeline('unit', client)  # type: ignore[arg-type]
    for index, x in enumerate((100.0, 110.0, 120.0)):
        pipeline.push(RealtimeTaskEvent(index, index * 100.0, index * 100.0, True, True, x, 100.0))
    stopped = pipeline.stop_for_stationary_dwell(5_000.0)
    assert len(stopped) == 1
    segment = stopped[0]
    assert segment.finalization_reason == 'terminal_stop'
    assert segment.time_contract.source_start_t_ms == 100.0
    assert segment.time_contract.source_end_t_ms == 200.0
    samples = segment.sample_uniform(1_000.0)
    assert min(sample.x_mm for sample in samples) >= 110.0
    assert max(sample.x_mm for sample in samples) <= 120.0
    assert 'BARRIER' not in [kind for kind, _payload in client.messages]
