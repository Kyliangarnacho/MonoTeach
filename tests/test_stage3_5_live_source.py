"""Deterministic contracts for the Stage 3.5 real-camera source boundary.

These tests never open C920 or MediaPipe.  Their purpose is to prove that the
new source keeps capture/availability clocks, calibration geometry, start-gate
evidence and formal tracking events separate before a user performs Task 6A.
"""

from __future__ import annotations

import numpy as np
import pytest

from stage2.camera_calibration import CameraCalibration
from stage2.camera_stream import CameraConfig, CameraStream
from stage2.hand_observation import HandObservation
from stage2.pen_state import PenStateController
from stage2.workspace_geometry import WorkspaceCalibration, WorkspaceDefinition
from stage3.realtime.live_observation import LiveWorkspaceObservation
from stage3.realtime.live_workspace_source import LiveWorkspaceSource


def _camera_calibration() -> CameraCalibration:
    return CameraCalibration(
        K=np.array([[50.0, 0.0, 50.0], [0.0, 50.0, 50.0], [0.0, 0.0, 1.0]]),
        D=np.zeros(5),
        frame_width=100,
        frame_height=100,
        source_path="test_camera_params.npz",
        source_format="npz",
    )


def _workspace_calibration() -> WorkspaceCalibration:
    return WorkspaceCalibration(
        schema_version="1.0",
        calibration_id="fresh_test_h",
        frame_width=100,
        frame_height=100,
        workspace_definition=WorkspaceDefinition(200.0, 200.0),
        image_points=np.array([[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]]),
        workspace_points_mm=np.array([[0.0, 0.0], [200.0, 0.0], [200.0, 200.0], [0.0, 200.0]]),
        H_image_to_workspace=np.array([[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 1.0]]),
        camera_calibration_reference="test_camera_params.npz",
    )


class _Tracker:
    """Tiny fake that makes source tests independent of the MediaPipe model."""

    def __init__(self, observations: list[HandObservation]) -> None:
        self._observations = iter(observations)
        self.timestamps: list[float] = []

    def process(self, _frame: np.ndarray, timestamp_ms: float) -> HandObservation:
        self.timestamps.append(timestamp_ms)
        return next(self._observations)



def test_live_workspace_source_uses_capture_for_source_and_availability_for_latency() -> None:
    tracker = _Tracker([HandObservation(1_000.0, True, index_tip_px=(10, 20))])
    pen = PenStateController()
    pen.toggle()  # Source receives a full, valid DOWN sample for the legacy-compatible subset.
    source = LiveWorkspaceSource(
        CameraStream(CameraConfig(index=99, width=100, height=100)),
        tracker,
        _camera_calibration(),
        _workspace_calibration(),
        pen_controller=pen,
        monotonic_ms=lambda: 1_004.0,
    )

    result = source.process_frame(np.zeros((100, 100, 3), dtype=np.uint8), 1_000.0)

    assert result.capture_t_ms == 0.0
    assert result.available_t_ms == 4.0
    assert result.processing_latency_ms == 4.0
    assert (result.x_mm, result.y_mm) == pytest.approx((20.0, 40.0))
    assert result.raw_pixel_xy == pytest.approx((10.0, 20.0))
    assert result.pen_state == "DOWN"
    assert result.to_workspace_sample(0.0).stroke_id == 1
    assert tracker.timestamps == [1_000.0]


def test_live_workspace_source_retains_no_hand_as_an_explicit_invalid_observation() -> None:
    source = LiveWorkspaceSource(
        CameraStream(CameraConfig(index=99, width=100, height=100)),
        _Tracker([HandObservation(2_000.0, False)]),
        _camera_calibration(),
        _workspace_calibration(),
        monotonic_ms=lambda: 2_001.0,
    )

    result = source.process_frame(np.zeros((100, 100, 3), dtype=np.uint8), 2_000.0)

    assert not result.valid
    assert result.invalid_reason == "no_hand"
    assert result.x_mm is None
    assert result.pen_state == "UP"


def test_valid_up_cannot_be_silently_downgraded_into_frozen_stage2_trajectory() -> None:
    valid_up = LiveWorkspaceObservation(
        frame_index=0,
        capture_t_ms=0.0,
        available_t_ms=1.0,
        valid=True,
        x_mm=20.0,
        y_mm=40.0,
        inside_workspace=True,
        invalid_reason=None,
        raw_pixel_xy=(10.0, 20.0),
        undistorted_pixel_xy=(10.0, 20.0),
        pen_state="UP",
        stroke_id=None,
    )

    with pytest.raises(ValueError, match="cannot encode a valid UP"):
        valid_up.to_workspace_sample(0.0)
