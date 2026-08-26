"""Single-threaded C920 -> workspace-mm source for the first live stage.

The source owns no planner and no execution queue.  Its one responsibility is
to turn one camera frame into one immutable :class:`LiveWorkspaceObservation`.
Keeping this boundary narrow makes it testable with a fake camera/tracker and
prevents a stale calibration from being silently selected by default.
"""

from __future__ import annotations

import time
from typing import Callable, Protocol

import numpy as np

from stage2.camera_calibration import (
    CameraCalibration,
    undistort_image_points,
    validate_calibration_resolution,
)
from stage2.camera_stream import CameraProfile, CameraStream
from stage2.hand_observation import HandObservation
from stage2.hand_tracker import HandTracker
from stage2.pen_state import PenStateController, PinchToggleDetector
from stage2.workspace_geometry import (
    WorkspaceCalibration,
    image_points_to_workspace,
    inside_workspace,
)

from .live_observation import LiveWorkspaceObservation
from .l_gesture import HandControlGesture, LGestureToggleDetector


class _Tracker(Protocol):
    def process(self, bgr_frame: np.ndarray, timestamp_ms: float) -> HandObservation: ...


class _PenCommandDetector(Protocol):
    """Small shared boundary for legacy pinch or new L-command detectors."""

    def update(self, observation: HandObservation) -> bool: ...


class LiveWorkspaceSource:
    """Map current camera frames using caller-supplied, explicitly chosen calibration."""

    def __init__(
        self,
        stream: CameraStream,
        tracker: _Tracker,
        camera_calibration: CameraCalibration,
        workspace_calibration: WorkspaceCalibration,
        *,
        pinch_detector: PinchToggleDetector | None = None,
        gesture_detector: LGestureToggleDetector | None = None,
        pen_controller: PenStateController | None = None,
        monotonic_ms: Callable[[], float] | None = None,
    ) -> None:
        if not isinstance(stream, CameraStream):
            raise TypeError("stream must be a CameraStream.")
        if not isinstance(camera_calibration, CameraCalibration):
            raise TypeError("camera_calibration must be a CameraCalibration.")
        if not isinstance(workspace_calibration, WorkspaceCalibration):
            raise TypeError("workspace_calibration must be a WorkspaceCalibration.")
        validate_calibration_resolution(
            camera_calibration,
            workspace_calibration.frame_width,
            workspace_calibration.frame_height,
        )
        self._stream = stream
        self._tracker = tracker
        self._camera_calibration = camera_calibration
        self._workspace_calibration = workspace_calibration
        if pinch_detector is not None and gesture_detector is not None:
            raise ValueError("Choose either legacy pinch_detector or gesture_detector, not both.")
        # The L detector is now the live default.  Pinch remains injectable
        # only so old deterministic tests and historical recordings retain a
        # precise meaning; it is no longer the operator-facing baseline.
        self._pen_command_detector: _PenCommandDetector = (
            PinchToggleDetector()
            if pinch_detector is not None
            else (LGestureToggleDetector() if gesture_detector is None else gesture_detector)
        )
        if pinch_detector is not None:
            self._pen_command_detector = pinch_detector
        self._gesture_detector = gesture_detector if gesture_detector is not None else (
            self._pen_command_detector if isinstance(self._pen_command_detector, LGestureToggleDetector) else None
        )
        self._pen_controller = PenStateController() if pen_controller is None else pen_controller
        self._monotonic_ms = (lambda: time.monotonic() * 1_000.0) if monotonic_ms is None else monotonic_ms
        self._frame_index = 0
        self._time_origin_ms: float | None = None
        self._last_capture_t_ms: float | None = None

    @property
    def workspace_calibration(self) -> WorkspaceCalibration:
        return self._workspace_calibration

    @property
    def camera_profile(self) -> CameraProfile:
        return self._stream.profile

    def toggle_pen_state(self) -> None:
        """Explicit keyboard fallback when camera gesture evidence is uncertain."""
        self._pen_controller.toggle()

    def open(self) -> CameraProfile:
        """Open C920 and reject a resolution that would invalidate K/D/H."""
        profile = self._stream.open()
        validate_calibration_resolution(self._camera_calibration, profile.width, profile.height)
        if (profile.width, profile.height) != (
            self._workspace_calibration.frame_width,
            self._workspace_calibration.frame_height,
        ):
            raise ValueError("Camera profile does not match workspace calibration resolution.")
        return profile

    def read(self) -> tuple[np.ndarray, LiveWorkspaceObservation]:
        """Read exactly one current frame and process it synchronously."""
        frame, capture_monotonic_ms = self._stream.read()
        return frame, self.process_frame(frame, capture_monotonic_ms)

    def process_frame(self, bgr_frame: np.ndarray, capture_monotonic_ms: float) -> LiveWorkspaceObservation:
        """Pure one-frame path used by both the C920 loop and deterministic tests."""
        capture_t_ms = self._session_time(capture_monotonic_ms)
        if self._last_capture_t_ms is not None and capture_t_ms <= self._last_capture_t_ms:
            raise ValueError("Live camera capture timestamps must strictly increase.")
        self._last_capture_t_ms = capture_t_ms
        frame_height, frame_width = bgr_frame.shape[:2]
        validate_calibration_resolution(self._camera_calibration, frame_width, frame_height)
        if (frame_width, frame_height) != (
            self._workspace_calibration.frame_width,
            self._workspace_calibration.frame_height,
        ):
            raise ValueError("Frame resolution does not match workspace calibration.")

        observation = self._tracker.process(bgr_frame, capture_monotonic_ms)
        pen_toggled = self._pen_command_detector.update(observation)
        if pen_toggled:
            self._pen_controller.toggle()

        control_gesture = (
            self._gesture_detector.last_gesture.value
            if self._gesture_detector is not None
            else HandControlGesture.UNAVAILABLE.value
        )
        # A raw L-shaped hand is only a candidate.  It becomes a spatial
        # barrier after the detector's timestamp-based dwell confirms an
        # intentional command; ordinary thumb motion must not erase points.
        control_gesture_active = bool(
            self._gesture_detector is not None and self._gesture_detector.command_active
        )
        control_gesture_candidate = bool(
            self._gesture_detector is not None and self._gesture_detector.candidate_active
        )
        control_gesture_armed = bool(
            self._gesture_detector is not None and self._gesture_detector.armed
        )
        candidate_elapsed_ms = (
            self._gesture_detector.candidate_elapsed_ms if self._gesture_detector is not None else 0.0
        )
        dropout_grace_remaining_ms = (
            self._gesture_detector.dropout_grace_remaining_ms if self._gesture_detector is not None else 0.0
        )

        available_t_ms = self._session_time(self._monotonic_ms())
        if not observation.detected or observation.index_tip_px is None:
            result = LiveWorkspaceObservation(
                frame_index=self._frame_index,
                capture_t_ms=capture_t_ms,
                available_t_ms=max(available_t_ms, capture_t_ms),
                valid=False,
                x_mm=None,
                y_mm=None,
                inside_workspace=False,
                invalid_reason="no_hand",
                raw_pixel_xy=None,
                undistorted_pixel_xy=None,
                pen_state=self._pen_controller.state.value,
                stroke_id=self._pen_controller.active_stroke_id,
                control_gesture=control_gesture,
                control_gesture_active=control_gesture_active,
                control_gesture_candidate=control_gesture_candidate,
                control_gesture_armed=control_gesture_armed,
                control_gesture_candidate_elapsed_ms=candidate_elapsed_ms,
                control_gesture_dropout_grace_remaining_ms=dropout_grace_remaining_ms,
                control_gesture_toggled=pen_toggled,
            )
        else:
            raw_xy = (float(observation.index_tip_px[0]), float(observation.index_tip_px[1]))
            undistorted = undistort_image_points([raw_xy], self._camera_calibration)[0]
            x_mm, y_mm = image_points_to_workspace(
                [undistorted], self._workspace_calibration.H_image_to_workspace
            )[0]
            result = LiveWorkspaceObservation(
                frame_index=self._frame_index,
                capture_t_ms=capture_t_ms,
                available_t_ms=max(available_t_ms, capture_t_ms),
                valid=True,
                x_mm=float(x_mm),
                y_mm=float(y_mm),
                inside_workspace=inside_workspace(
                    float(x_mm), float(y_mm), self._workspace_calibration.workspace_definition
                ),
                invalid_reason=None,
                raw_pixel_xy=raw_xy,
                undistorted_pixel_xy=(float(undistorted[0]), float(undistorted[1])),
                pen_state=self._pen_controller.state.value,
                stroke_id=self._pen_controller.active_stroke_id,
                control_gesture=control_gesture,
                control_gesture_active=control_gesture_active,
                control_gesture_candidate=control_gesture_candidate,
                control_gesture_armed=control_gesture_armed,
                control_gesture_candidate_elapsed_ms=candidate_elapsed_ms,
                control_gesture_dropout_grace_remaining_ms=dropout_grace_remaining_ms,
                control_gesture_toggled=pen_toggled,
            )
        self._frame_index += 1
        return result

    def close(self) -> None:
        """Release camera and optional tracker resources; repeated calls are safe."""
        close = getattr(self._tracker, "close", None)
        if callable(close):
            close()
        self._stream.release()

    def _session_time(self, monotonic_ms: float) -> float:
        numeric = float(monotonic_ms)
        if self._time_origin_ms is None:
            self._time_origin_ms = numeric
        return max(numeric - self._time_origin_ms, 0.0)
