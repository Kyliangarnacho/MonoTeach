"""Final manual demo: C920 live source -> Python quintics -> MATLAB q(t)."""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import time
from typing import Sequence
import cv2

from stage2.camera_calibration import load_camera_calibration, validate_calibration_resolution
from stage2.camera_stream import CameraConfig, CameraStream
from stage2.hand_tracker import HandTracker
from stage2.workspace_calibration_io import load_workspace_calibration_json
from .live_follow_session import LiveFollowSession, LiveFollowPhase, StartNotReadyError
from .live_planning_pipeline import LivePlanningPipeline
from .live_position_filter import LivePositionFilter, OneEuroPositionFilterConfig
from .live_protocol import LiveProtocolClient
from .live_workspace_source import LiveWorkspaceSource

WINDOW = "MonoTeach Live Follow (camera is raw / not mirrored)"


def _reply_summary(reply: dict[str, object]) -> str:
    """Compact MATLAB state for both the terminal and camera overlay."""
    kind = str(reply.get("kind", "UNKNOWN"))
    payload = reply.get("payload", {})
    if not isinstance(payload, dict):
        return kind
    reason = payload.get("reason")
    source = payload.get("source_index")
    suffix = "" if reason in (None, "") else f": {reason}"
    if source is not None:
        suffix += f" [source={source}]"
    return kind + suffix


def _wait_for_matlab_finish(client: LiveProtocolClient, timeout_s: float = 30.0) -> None:
    """Keep TCP alive until MATLAB has consumed the final q(t) FIFO tail."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        reply = client.receive()
        if reply is not None and reply.get('kind') == 'FINISHED':
            return
        if reply is not None and reply.get('kind') in {'HOLD', 'PLANNING_BLOCKED', 'EXECUTOR_TIMEOUT', 'PROTOCOL_ERROR'}:
            raise RuntimeError(f"MATLAB {_reply_summary(reply)}")
        time.sleep(0.01)
    raise TimeoutError('MATLAB did not confirm FINISHED within 30 seconds.')


def _xy_text(xy: tuple[float, float] | None) -> str:
    """Render one coordinate layer without pretending it is another layer."""
    return '--' if xy is None else f'({xy[0]:.1f}, {xy[1]:.1f}) mm'


def _overlay_lines(
    follow: LiveFollowSession,
    obs: object,
    matlab_status: str,
) -> list[str]:
    """Keep operator UI concise; phase-specific facts replace stale diagnostics."""
    # ``obs`` is kept duck-typed here so this display helper does not become a
    # second source/data model.  The source owns the immutable observation.
    phase = follow.phase
    lines = [
        'SPACE: lock P0 / start teaching | P: pen | Q: end and drain',
        f'PHASE: {phase.value} | PEN: {obs.pen_state} | MATLAB: {matlab_status}',
        f'MEASURED XY: {_xy_text(follow.latest_measured_xy_mm)} | valid={obs.valid} inside={obs.inside_workspace}',
        f'FORMAL TARGET: {_xy_text(follow.latest_formal_target_xy_mm)} | offset={_xy_text(follow.mapping_offset_xy_mm)}',
    ]
    if phase is LiveFollowPhase.WAIT_FOR_START_CONFIRM:
        lines.append(f'P0 readiness: {follow.start_readiness.summary}')
    elif phase is LiveFollowPhase.PEN_TRANSITION:
        lines.append(
            'Pen transition: waiting for queued lift/transfer/lower; '
            'ordinary XY tracking is paused.'
        )
    elif phase in {LiveFollowPhase.VISION_GAP, LiveFollowPhase.WAIT_FOR_REACQUIRE_READY}:
        lines.append('Vision recovery: no new Cartesian segment is admitted until safe re-acquire completes.')
    elif obs.control_gesture_candidate or obs.control_gesture_active:
        lines.append(
            f'Gesture: candidate={obs.control_gesture_candidate} active={obs.control_gesture_active} '
            f'dwell={obs.control_gesture_candidate_elapsed_ms:.0f} ms'
        )
    return lines

def main(argv: Sequence[str] | None = None) -> int:
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--camera-calibration',required=True);p.add_argument('--workspace-calibration',required=True);p.add_argument('--camera-index',type=int,default=0);p.add_argument('--port',type=int,default=51011)
    a=p.parse_args(argv); stream=None; source=None; client=None
    try:
        camera=load_camera_calibration(a.camera_calibration); workspace=load_workspace_calibration_json(a.workspace_calibration);validate_calibration_resolution(camera,workspace.frame_width,workspace.frame_height)
        stream=CameraStream(CameraConfig(index=a.camera_index,width=camera.frame_width,height=camera.frame_height));source=LiveWorkspaceSource(stream,HandTracker(),camera,workspace);profile=source.open()
        session_id='live_follow_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ');client=LiveProtocolClient(port=a.port);client.connect(session_id)
        client.send(session_id, 'WORKSPACE_GEOMETRY', {
            'width_mm': workspace.workspace_definition.width_mm,
            'height_mm': workspace.workspace_definition.height_mm,
            'calibration_id': workspace.calibration_id,
        })
        follow=LiveFollowSession(workspace); filter_=LivePositionFilter(OneEuroPositionFilterConfig()); pipeline=LivePlanningPipeline(session_id,client)
        print(f'Connected to MATLAB on port {a.port}; camera {profile.width}x{profile.height}. SPACE: lock start / begin teaching. P: pen UP/DOWN fallback. Q: finish.')
        cv2.namedWindow(WINDOW,cv2.WINDOW_AUTOSIZE)
        last_available=0.0; matlab_status='CONNECTED: waiting for P0'
        while True:
            frame,obs=source.read();last_available=obs.available_t_ms;estimate=filter_.push(obs)
            # Camera health is reported every frame, while formal planning
            # events are deliberately sparse.  A quiet hand is compressed into
            # a later dwell endpoint rather than generating redundant IK work.
            if follow.phase is LiveFollowPhase.TRACKING:
                pipeline.heartbeat(capture_t_ms=obs.capture_t_ms, available_t_ms=obs.available_t_ms, valid=obs.valid)
            events = follow.process_many(obs,estimate)
            # A sustained hold closes the old derivative window before the
            # first moved event reaches it.  This is a temporal restart, not
            # an invalid/outside barrier and not a robot-side emergency stop.
            restart = follow.take_motion_restart_boundary()
            if restart is not None:
                pipeline.stop_for_stationary_dwell(restart.replay_t_ms)
                matlab_status = 'STATIONARY_DWELL: old motion run closed at rest'
            for event in events:
                last_available=event.replay_t_ms
                pipeline.push(event)
            # A real vision boundary never permits post-gap tracking chunks to
            # accumulate behind a robot-side bridge.  The session exposes one
            # stable target; MATLAB replies before ordinary planning resumes.
            reacquire=follow.take_reacquire_target()
            if reacquire is not None:
                client.send(session_id,'REACQUIRE_TARGET',{
                    'x_mm':reacquire.x_mm,'y_mm':reacquire.y_mm,
                    'z_task_mm':reacquire.z_task_mm,'pen_state':reacquire.pen_state,
                })
                matlab_status='REACQUIRE_TARGET: waiting for safe robot bridge'
                print(f'Reacquire target queued at ({reacquire.x_mm:.1f},{reacquire.y_mm:.1f}); source planning paused.')
            while (reply:=client.receive()) is not None:
                if reply.get('kind')=='START_READY' and follow.phase is LiveFollowPhase.WAIT_FOR_ROBOT_READY:
                    follow.notify_robot_ready();matlab_status='START_READY: waiting for teaching SPACE';print('MATLAB reached P0. Hold hand at desired start, then press SPACE to begin teaching.')
                elif reply.get('kind')=='PEN_TRANSITION_READY' and follow.phase is LiveFollowPhase.PEN_TRANSITION:
                    follow.notify_pen_transition_ready()
                    if follow.phase is LiveFollowPhase.TRACKING:
                        matlab_status='PEN_TRANSITION_READY: tracking resumed';print('MATLAB finished pen transition; tracking re-anchored.')
                    else:
                        matlab_status='PEN_TRANSITION_READY: release L to re-anchor';print('MATLAB finished pen transition; release L once to establish the fresh hand anchor.')
                elif reply.get('kind')=='REACQUIRE_READY' and follow.phase is LiveFollowPhase.WAIT_FOR_REACQUIRE_READY:
                    pipeline.push(follow.notify_reacquire_ready());matlab_status='REACQUIRE_READY: tracking resumed';print('MATLAB reached recovered target; tracking re-anchored.')
                elif reply.get('kind') != 'ACK':
                    matlab_status=_reply_summary(reply);print('MATLAB:',matlab_status)
            display=frame.copy()
            if obs.raw_pixel_xy: cv2.circle(display,tuple(map(int,obs.raw_pixel_xy)),7,(0,0,255),-1)
            lines = _overlay_lines(follow, obs, matlab_status)
            for i,line in enumerate(lines):cv2.putText(display,line,(12,28+26*i),cv2.FONT_HERSHEY_SIMPLEX,.58,(255,255,255),2,cv2.LINE_AA)
            cv2.imshow(WINDOW,display);key=cv2.waitKey(1)&0xff
            if key==ord(' '):
                if follow.phase is LiveFollowPhase.WAIT_FOR_START_CONFIRM:
                    try:
                        target=follow.confirm_start()
                    except StartNotReadyError as error:
                        print(f'Start not ready; preview remains open: {error.readiness.summary}')
                    else:
                        client.send(session_id,'START_TARGET',{'x_mm':target.x_mm,'y_mm':target.y_mm});print(f'P0 locked at ({target.x_mm:.1f},{target.y_mm:.1f}); MATLAB moving HOME_TO_START.')
                elif follow.phase is LiveFollowPhase.WAIT_FOR_TEACH_CONFIRM:
                    try:
                        pipeline.push(follow.confirm_teaching())
                    except RuntimeError as error:
                        print(f'Teaching not ready; preview remains open: {error}')
                    else:
                        print('Tracking started: relative hand motion now drives planned segments.')
                else: print('SPACE ignored in phase',follow.phase.value)
            if key in (ord('p'),ord('P')):
                source.toggle_pen_state();print('Keyboard pen toggle; applies to following source events.')
            if key in (ord('q'),ord('Q')):
                if follow.phase is LiveFollowPhase.TRACKING:
                    for event in follow.finish_tracking(): pipeline.push(event)
                    pipeline.finish(last_available)
                    print('Input closed; MATLAB is draining the final q(t) FIFO...')
                    _wait_for_matlab_finish(client)
                follow.close();break
        return 0
    except (RuntimeError,ValueError,TypeError,TimeoutError,cv2.error,OSError) as e:
        print('Live MATLAB follow failed:',e);return 1
    finally:
        if client: client.close()
        if source: source.close()
        elif stream: stream.release()
        cv2.destroyAllWindows()
if __name__=='__main__':raise SystemExit(main())
