"""Synthetic tests for landmark-only pen-state recording semantics."""

from __future__ import annotations

from stage2.hand_observation import HandObservation
from stage2.pen_state import PenState, PenStateController, PinchToggleConfig, PinchToggleDetector
from stage2.trajectory_recorder import TrajectoryRecorder


def pinch_observation(timestamp_ms: float, thumb_middle_distance: float) -> HandObservation:
    landmarks = [(0.0, 0.0, 0.0) for _ in range(21)]
    landmarks[9] = (0.0, 1.0, 0.0)
    landmarks[12] = (0.0, 2.0, 0.0)
    landmarks[4] = (thumb_middle_distance, 2.0, 0.0)
    return HandObservation(
        timestamp_ms=timestamp_ms,
        detected=True,
        landmarks_norm=tuple(landmarks),
        index_tip_norm=(0.25, 0.50),
        index_tip_px=(25, 50),
    )


def start(recorder: TrajectoryRecorder) -> None:
    recorder.start(
        recording_start_timestamp_ms=0.0,
        frame_width=100,
        frame_height=100,
        camera_index=0,
    )


def test_pinch_toggle_requires_stable_press_and_release_before_retriggering():
    detector = PinchToggleDetector(PinchToggleConfig(stable_frame_count=2))

    assert detector.update(pinch_observation(10.0, 0.20)) is False
    assert detector.update(pinch_observation(20.0, 0.20)) is True
    assert detector.update(pinch_observation(30.0, 0.20)) is False
    assert detector.update(pinch_observation(40.0, 0.20)) is False
    assert detector.update(pinch_observation(50.0, 0.80)) is False
    assert detector.update(pinch_observation(60.0, 0.80)) is False
    assert detector.update(pinch_observation(70.0, 0.20)) is False
    assert detector.update(pinch_observation(80.0, 0.20)) is True


def test_pen_controller_starts_and_ends_incrementing_semantic_strokes():
    controller = PenStateController()

    first_down = controller.toggle()
    assert first_down.begins_stroke is True
    assert controller.state is PenState.DOWN
    assert first_down.stroke_id == 1
    assert controller.active_stroke_id == 1

    first_up = controller.toggle()
    assert first_up.ends_stroke is True
    assert controller.state is PenState.UP
    assert controller.active_stroke_id is None
    assert controller.display_stroke_id == 1

    second_down = controller.toggle()
    assert second_down.begins_stroke is True
    assert second_down.stroke_id == 2
    assert controller.active_stroke_id == 2


def test_pen_up_motion_is_not_connected_into_writing_strokes():
    recorder = TrajectoryRecorder()
    controller = PenStateController()
    start(recorder)

    controller.toggle()
    recorder.record(
        pinch_observation(10.0, 0.80),
        pen_state=PenState.DOWN.value,
        stroke_id=controller.active_stroke_id,
    )
    recorder.record(
        pinch_observation(20.0, 0.80),
        pen_state=PenState.DOWN.value,
        stroke_id=controller.active_stroke_id,
    )

    controller.toggle()
    recorder.record_pen_up_barrier(30.0)
    # 40/50 ms UP observations intentionally do not call recorder.record.

    controller.toggle()
    recorder.record(
        pinch_observation(60.0, 0.80),
        pen_state=PenState.DOWN.value,
        stroke_id=controller.active_stroke_id,
    )

    samples = recorder.samples
    assert [(sample.pen_state, sample.stroke_id) for sample in samples] == [
        ("DOWN", 1),
        ("DOWN", 1),
        ("UP", None),
        ("DOWN", 2),
    ]
    assert [sample.invalid_reason for sample in samples] == [None, None, "pen_up", None]
    assert [sample.valid for sample in samples] == [True, True, False, True]
    assert samples[2].invalid_reason == "pen_up"
    assert samples[3].stroke_id == 2
