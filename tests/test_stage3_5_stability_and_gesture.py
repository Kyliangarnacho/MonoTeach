"""Deterministic tests for the live measurement filter and L pen command."""

from __future__ import annotations

import numpy as np
import pytest

from stage2.hand_observation import HandObservation
from stage2.workspace_geometry import WorkspaceCalibration, WorkspaceDefinition
from stage3.realtime.l_gesture import HandControlGesture, LGestureConfig, LGestureToggleDetector, classify_l_gesture
from stage3.realtime.live_observation import LiveWorkspaceObservation
from stage3.realtime.live_position_filter import FilteredWorkspaceEstimate, LivePositionFilter, OneEuroPositionFilterConfig


def _calibration() -> WorkspaceCalibration:
    return WorkspaceCalibration(
        schema_version="1.0",
        calibration_id="stability_test_h",
        frame_width=100,
        frame_height=100,
        workspace_definition=WorkspaceDefinition(200.0, 200.0),
        image_points=np.array([[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]]),
        workspace_points_mm=np.array([[0.0, 0.0], [200.0, 0.0], [200.0, 200.0], [0.0, 200.0]]),
        H_image_to_workspace=np.array([[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 1.0]]),
        camera_calibration_reference="test.npz",
    )


def _live(
    frame_index: int,
    x_mm: float | None,
    y_mm: float | None,
    *,
    active: bool = False,
) -> LiveWorkspaceObservation:
    valid = x_mm is not None and y_mm is not None
    return LiveWorkspaceObservation(
        frame_index=frame_index,
        capture_t_ms=frame_index * 33.0,
        available_t_ms=frame_index * 33.0 + 2.0,
        valid=valid,
        x_mm=x_mm,
        y_mm=y_mm,
        inside_workspace=valid,
        invalid_reason=None if valid else "no_hand",
        raw_pixel_xy=(10.0, 20.0) if valid else None,
        undistorted_pixel_xy=(10.0, 20.0) if valid else None,
        pen_state="DOWN",
        stroke_id=1,
        control_gesture="L_COMMAND" if active else "POINT",
        control_gesture_active=active,
    )


def test_one_euro_reduces_static_range_and_resets_on_invalid_barrier() -> None:
    raw = [_live(index, 50.0 + (-1.0 if index % 2 else 1.0), 60.0) for index in range(12)]
    position_filter = LivePositionFilter(OneEuroPositionFilterConfig(min_cutoff_hz=1.0, beta=0.0, derivative_cutoff_hz=1.0))
    filtered = [position_filter.push(item).filtered_x_mm for item in raw]
    assert max(filtered) - min(filtered) < max(item.x_mm for item in raw) - min(item.x_mm for item in raw)

    invalid = _live(12, None, None)
    assert position_filter.push(invalid).reset_reason == "invalid_observation"
    reseeded = position_filter.push(_live(13, 80.0, 60.0))
    assert reseeded.filtered_x_mm == 80.0
    assert reseeded.reset_reason == "new_continuous_run"


def _landmarks(kind: str) -> tuple[tuple[float, float, float], ...]:
    """Simple 2D synthetic hand shapes with index extended and others folded."""
    points = [(0.0, 0.0, 0.0) for _ in range(21)]
    points[0] = (0.0, 0.0, 0.0)
    points[5], points[6], points[7], points[8] = (0.0, 0.25, 0.0), (0.0, 0.60, 0.0), (0.0, 1.0, 0.0), (0.0, 1.4, 0.0)
    for mcp, pip, dip, tip in ((9, 10, 11, 12), (13, 14, 15, 16), (17, 18, 19, 20)):
        x = (mcp - 5) * 0.07
        points[mcp], points[pip], points[dip], points[tip] = (x, 0.25, 0.0), (x, 0.55, 0.0), (x, 0.45, 0.0), (x, 0.30, 0.0)
    if kind == "L":
        points[1], points[2], points[3], points[4] = (-0.15, 0.0, 0.0), (-0.4, 0.0, 0.0), (-0.7, 0.0, 0.0), (-1.0, 0.0, 0.0)
    else:
        points[1], points[2], points[3], points[4] = (-0.15, 0.0, 0.0), (-0.4, 0.0, 0.0), (-0.3, 0.15, 0.0), (-0.2, 0.28, 0.0)
    return tuple(points)


def _hand(kind: str, timestamp_ms: float = 0.0) -> HandObservation:
    return HandObservation(timestamp_ms, True, landmarks_norm=_landmarks(kind), index_tip_px=(10, 10))


def test_l_gesture_toggle_needs_point_rearm_and_does_not_repeat_while_held() -> None:
    detector = LGestureToggleDetector(LGestureConfig(command_dwell_ms=100.0, point_rearm_ms=100.0))
    assert classify_l_gesture(_hand("POINT")) is HandControlGesture.POINT
    assert classify_l_gesture(_hand("L")) is HandControlGesture.L_COMMAND
    assert [detector.update(_hand("POINT", t)) for t in (0, 110)] == [False, False]
    assert [detector.update(_hand("L", t)) for t in (120, 230, 340)] == [False, True, False]
    assert not detector.armed
    assert [detector.update(_hand("POINT", t)) for t in (350, 460)] == [False, False]
    assert [detector.update(_hand("L", t)) for t in (470, 580)] == [False, True]


def test_confirmed_l_uses_lower_exit_threshold_before_releasing() -> None:
    config = LGestureConfig(command_dwell_ms=10.0, point_rearm_ms=10.0)
    detector = LGestureToggleDetector(config)
    detector.update(_hand("POINT", 0.0)); detector.update(_hand("POINT", 20.0))
    detector.update(_hand("L", 30.0)); assert detector.update(_hand("L", 50.0))

    # This thumb is between the 1.15 entry and 0.85 exit thresholds: a
    # confirmed command must remain held rather than flickering off.
    points = list(_landmarks("L")); points[4] = (-0.40, 0.0, 0.0)
    middle = HandObservation(60.0, True, landmarks_norm=tuple(points), index_tip_px=(10, 10))
    assert not detector.update(middle)
    assert detector.command_active
    assert detector.last_gesture is HandControlGesture.L_COMMAND


def test_l_candidate_pauses_for_one_missing_frame_but_does_not_count_it_as_dwell() -> None:
    detector = LGestureToggleDetector(LGestureConfig(command_dwell_ms=100.0, point_rearm_ms=10.0, dropout_grace_ms=80.0))
    detector.update(_hand("POINT", 0.0)); detector.update(_hand("POINT", 20.0))
    assert not detector.update(_hand("L", 30.0))
    missing = HandObservation(60.0, False)
    assert not detector.update(missing)
    assert detector.candidate_active and not detector.command_active
    # The 30 ms no-hand interval is paused: 90 ms has only accumulated 30 ms
    # of visible L evidence, so it cannot yet toggle the pen.
    assert not detector.update(_hand("L", 90.0))
    assert detector.update(_hand("L", 160.0))


def test_l_active_expires_after_dropout_grace_instead_of_holding_forever() -> None:
    detector = LGestureToggleDetector(LGestureConfig(command_dwell_ms=10.0, point_rearm_ms=10.0, dropout_grace_ms=50.0))
    detector.update(_hand("POINT", 0.0)); detector.update(_hand("POINT", 20.0))
    detector.update(_hand("L", 30.0)); assert detector.update(_hand("L", 50.0))
    assert not detector.update(HandObservation(60.0, False))
    assert detector.command_active
    assert not detector.update(HandObservation(120.0, False))
    assert not detector.command_active and not detector.candidate_active
