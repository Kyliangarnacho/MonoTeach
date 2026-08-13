"""Hardware-free Stage 2.1 unit tests."""

import pytest
import numpy as np

from stage2.camera_stream import CameraConfig, CameraStream
from stage2.demo_camera import _argument_parser as camera_demo_argument_parser
from stage2.demo_fingertip_live import _argument_parser as fingertip_demo_argument_parser
from stage2.demo_fingertip_live import status_lines
from stage2.fingertip import normalized_to_pixel
from stage2.hand_observation import HandObservation
from stage2.hand_tracker import HandTracker


def test_normalized_to_pixel_at_image_boundaries():
    assert normalized_to_pixel(0.0, 0.0, 1280, 720) == (0, 0)
    assert normalized_to_pixel(1.0, 1.0, 1280, 720) == (1279, 719)


def test_normalized_to_pixel_clips_out_of_range_values():
    assert normalized_to_pixel(-0.2, 1.5, 320, 240) == (0, 239)


def test_normalized_to_pixel_uses_the_supplied_image_size():
    assert normalized_to_pixel(0.5, 0.5, 640, 480) == (320, 240)
    assert normalized_to_pixel(0.5, 0.5, 100, 200) == (50, 100)


def test_hand_observation_no_hand_state():
    observation = HandObservation(timestamp_ms=10.0, detected=False)

    assert observation.detected is False
    assert observation.landmarks_norm == ()
    assert observation.index_tip_norm is None
    assert observation.index_tip_px is None


def test_hand_observation_detected_state():
    observation = HandObservation(
        timestamp_ms=20.0,
        detected=True,
        handedness="Right",
        handedness_score=0.95,
        landmarks_norm=((0.1, 0.2, -0.01),),
        index_tip_norm=(0.1, 0.2),
        index_tip_px=(64, 96),
    )

    assert observation.detected is True
    assert observation.handedness == "Right"
    assert observation.index_tip_px == (64, 96)


def test_no_hand_observation_rejects_landmarks():
    with pytest.raises(ValueError, match="no-hand"):
        HandObservation(
            timestamp_ms=10.0,
            detected=False,
            landmarks_norm=((0.1, 0.2, 0.0),),
        )


def test_camera_stream_read_returns_monotonic_millisecond_timestamps(monkeypatch):
    class FakeCapture:
        def isOpened(self):
            return True

        def read(self):
            return True, np.zeros((2, 2, 3), dtype=np.uint8)

    timestamps = iter((10.001, 10.002))
    monkeypatch.setattr(
        "stage2.camera_stream.time.monotonic",
        lambda: next(timestamps),
    )
    stream = CameraStream(CameraConfig())
    stream._capture = FakeCapture()

    _frame_a, timestamp_a = stream.read()
    _frame_b, timestamp_b = stream.read()

    assert timestamp_a == pytest.approx(10_001.0)
    assert timestamp_b == pytest.approx(10_002.0)
    assert timestamp_b > timestamp_a


def test_hand_tracker_initializes_and_returns_no_hand_for_blank_frame():
    blank_frame = np.zeros((120, 160, 3), dtype=np.uint8)

    with HandTracker() as tracker:
        observation = tracker.process(blank_frame, timestamp_ms=1_000.0)

    assert observation.timestamp_ms == 1_000.0
    assert observation.detected is False
    assert observation.landmarks_norm == ()


def test_live_demo_status_lines_for_no_hand():
    observation = HandObservation(timestamp_ms=1.0, detected=False)

    assert status_lines(observation, fps=30.0) == ("FPS: 30.0", "No hand")


def test_live_demo_status_lines_for_detected_hand():
    observation = HandObservation(
        timestamp_ms=1.0,
        detected=True,
        handedness="Right",
        handedness_score=0.95,
        index_tip_norm=(0.125, 0.875),
        index_tip_px=(80, 420),
    )

    assert status_lines(observation, fps=24.5) == (
        "FPS: 24.5",
        "Hand: Right (0.95)",
        "Index tip norm: x=0.125, y=0.875",
        "Index tip px: u=80, v=420",
    )


def test_camera_facing_stage_2_1_demos_accept_explicit_camera_index():
    assert camera_demo_argument_parser().parse_args(["--camera-index", "0"]).camera_index == 0
    assert fingertip_demo_argument_parser().parse_args(["--camera-index", "0"]).camera_index == 0
