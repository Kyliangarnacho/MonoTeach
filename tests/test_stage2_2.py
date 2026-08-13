"""Hardware-free tests for the Stage 2.2 trajectory data contract."""

import json

import pytest

from stage2.hand_observation import HandObservation
from stage2.demo_trajectory_record import (
    EMA_ALPHA_PRESETS,
    _argument_parser as trajectory_record_argument_parser,
    derive_display_trajectories,
    valid_pixel_segments,
)
from stage2.display_geometry import display_pixel
from stage2.trajectory import (
    COORDINATE_SPACE,
    TRAJECTORY_SOURCE,
    Trajectory2D,
    TrajectoryMetadata,
    TrajectorySample,
    observation_to_trajectory_sample,
)
from stage2.trajectory_recorder import RecorderState, TrajectoryRecorder
from stage2.trajectory_quality import QualityGateConfig, apply_quality_gate
from stage2.trajectory_filter import EMAConfig, apply_ema_filter
from stage2.trajectory_io import load_trajectory_json, save_trajectory_json
from stage2.trajectory_playback import build_playback_timeline


def detected_observation(timestamp_ms: float = 1_250.0) -> HandObservation:
    return HandObservation(
        timestamp_ms=timestamp_ms,
        detected=True,
        handedness="Right",
        handedness_score=0.95,
        index_tip_norm=(0.25, 0.75),
        index_tip_px=(320, 540),
    )


def test_trajectory_record_demo_accepts_explicit_camera_index():
    arguments = trajectory_record_argument_parser().parse_args(["--camera-index", "0"])

    assert arguments.camera_index == 0


def test_valid_observation_converts_to_valid_sample():
    sample = observation_to_trajectory_sample(
        detected_observation(),
        recording_start_timestamp_ms=1_000.0,
    )

    assert sample == TrajectorySample(
        t_ms=250.0,
        valid=True,
        x_norm=0.25,
        y_norm=0.75,
        u=320,
        v=540,
        invalid_reason=None,
    )


def test_no_hand_converts_to_invalid_sample():
    observation = HandObservation(timestamp_ms=1_100.0, detected=False)

    sample = observation_to_trajectory_sample(
        observation,
        recording_start_timestamp_ms=1_000.0,
    )

    assert sample.valid is False
    assert (sample.x_norm, sample.y_norm, sample.u, sample.v) == (
        None,
        None,
        None,
        None,
    )
    assert sample.invalid_reason == "no_hand"


def test_sample_timestamp_is_relative_to_recording_start():
    sample = observation_to_trajectory_sample(
        detected_observation(timestamp_ms=5_050.5),
        recording_start_timestamp_ms=5_000.0,
    )

    assert sample.t_ms == pytest.approx(50.5)


def test_observation_before_recording_start_is_rejected():
    with pytest.raises(ValueError, match="cannot precede"):
        observation_to_trajectory_sample(
            detected_observation(timestamp_ms=999.0),
            recording_start_timestamp_ms=1_000.0,
        )


def test_trajectory_metadata_records_capture_context():
    metadata = TrajectoryMetadata(
        schema_version="1.0",
        trajectory_id="trajectory-001",
        frame_width=640,
        frame_height=480,
        camera_index=1,
        recording_start_timestamp_ms=10_000.0,
    )

    assert metadata.source == TRAJECTORY_SOURCE
    assert metadata.coordinate_space == COORDINATE_SPACE
    assert metadata.frame_width == 640
    assert metadata.frame_height == 480
    assert metadata.camera_index == 1


def test_conversion_does_not_modify_trajectory_raw_samples():
    original_sample = TrajectorySample(
        t_ms=0.0,
        valid=False,
        x_norm=None,
        y_norm=None,
        u=None,
        v=None,
        invalid_reason="no_hand",
    )
    raw_samples = (original_sample,)
    trajectory = Trajectory2D(
        metadata=TrajectoryMetadata(
            schema_version="1.0",
            trajectory_id="trajectory-001",
            frame_width=1280,
            frame_height=720,
            camera_index=1,
            recording_start_timestamp_ms=1_000.0,
        ),
        raw_samples=raw_samples,
    )

    converted = observation_to_trajectory_sample(
        detected_observation(),
        recording_start_timestamp_ms=1_000.0,
    )

    assert converted not in trajectory.raw_samples
    assert trajectory.raw_samples is raw_samples
    assert trajectory.raw_samples == (original_sample,)


def start_recorder(recorder: TrajectoryRecorder) -> None:
    recorder.start(
        recording_start_timestamp_ms=1_000.0,
        frame_width=1280,
        frame_height=720,
        camera_index=1,
    )


def test_recorder_initial_state_is_idle():
    recorder = TrajectoryRecorder()

    assert recorder.state is RecorderState.IDLE
    assert recorder.sample_count == 0
    assert recorder.finished_trajectory is None


def test_record_while_idle_is_a_no_op():
    recorder = TrajectoryRecorder()

    result = recorder.record(detected_observation())

    assert result is None
    assert recorder.sample_count == 0
    assert recorder.state is RecorderState.IDLE


def test_start_enters_recording_and_creates_metadata():
    recorder = TrajectoryRecorder()

    metadata = recorder.start(
        recording_start_timestamp_ms=1_000.0,
        frame_width=640,
        frame_height=480,
        camera_index=1,
    )

    assert recorder.state is RecorderState.RECORDING
    assert metadata.trajectory_id
    assert metadata.frame_width == 640
    assert metadata.frame_height == 480


def test_recording_preserves_valid_and_no_hand_samples():
    recorder = TrajectoryRecorder()
    start_recorder(recorder)

    first = recorder.record(detected_observation(timestamp_ms=1_100.0))
    second = recorder.record(HandObservation(timestamp_ms=1_200.0, detected=False))
    third = recorder.record(detected_observation(timestamp_ms=1_300.0))

    assert first is not None and first.valid is True
    assert second is not None and second.invalid_reason == "no_hand"
    assert third is not None and third.valid is True
    assert recorder.sample_count == 3


@pytest.mark.parametrize("timestamp_ms", [1_100.0, 1_099.0])
def test_record_rejects_non_increasing_timestamps(timestamp_ms):
    recorder = TrajectoryRecorder()
    start_recorder(recorder)
    recorder.record(detected_observation(timestamp_ms=1_100.0))

    with pytest.raises(ValueError, match="strictly increasing"):
        recorder.record(detected_observation(timestamp_ms=timestamp_ms))

    assert recorder.sample_count == 1


def test_stop_enters_ready_and_seals_raw_samples_as_tuple():
    recorder = TrajectoryRecorder()
    start_recorder(recorder)
    recorder.record(detected_observation(timestamp_ms=1_100.0))

    trajectory = recorder.stop()

    assert recorder.state is RecorderState.READY
    assert isinstance(trajectory, Trajectory2D)
    assert isinstance(trajectory.raw_samples, tuple)
    assert len(trajectory.raw_samples) == 1
    assert recorder.finished_trajectory is trajectory
    assert recorder.current_trajectory is trajectory


def test_ready_recorder_requires_reset_before_new_start():
    recorder = TrajectoryRecorder()
    start_recorder(recorder)
    recorder.stop()

    with pytest.raises(RuntimeError, match=r"READY.*reset"):
        start_recorder(recorder)


def test_reset_returns_to_idle_and_clears_old_trajectory():
    recorder = TrajectoryRecorder()
    start_recorder(recorder)
    recorder.record(detected_observation(timestamp_ms=1_100.0))
    recorder.stop()

    recorder.reset()

    assert recorder.state is RecorderState.IDLE
    assert recorder.sample_count == 0
    assert recorder.finished_trajectory is None


def test_invalid_state_operations_report_clear_errors():
    recorder = TrajectoryRecorder()

    with pytest.raises(RuntimeError, match="Cannot stop.*IDLE"):
        recorder.stop()

    start_recorder(recorder)
    with pytest.raises(RuntimeError, match="Cannot start.*RECORDING"):
        start_recorder(recorder)

    recorder.stop()
    with pytest.raises(RuntimeError, match="Cannot record.*READY"):
        recorder.record(detected_observation(timestamp_ms=1_100.0))


def test_recorder_samples_exposes_read_only_snapshot():
    recorder = TrajectoryRecorder()
    start_recorder(recorder)
    recorder.record(detected_observation(timestamp_ms=1_100.0))

    snapshot = recorder.samples

    assert isinstance(snapshot, tuple)
    assert len(snapshot) == 1
    assert snapshot[0].valid is True


def test_valid_pixel_segments_preserve_invalid_gaps():
    samples = (
        TrajectorySample(0.0, True, 0.1, 0.1, 10, 10),
        TrajectorySample(10.0, True, 0.2, 0.2, 20, 20),
        TrajectorySample(20.0, False, None, None, None, None, "no_hand"),
        TrajectorySample(30.0, True, 0.7, 0.7, 70, 70),
        TrajectorySample(40.0, False, None, None, None, None, "no_hand"),
    )

    assert valid_pixel_segments(samples) == (
        ((10, 10), (20, 20)),
        ((70, 70),),
    )


def test_valid_pixel_segments_handles_only_invalid_samples():
    samples = (
        TrajectorySample(0.0, False, None, None, None, None, "no_hand"),
    )

    assert valid_pixel_segments(samples) == ()


def quality_sample(
    t_ms: float,
    x_norm: float,
    y_norm: float,
    u: int,
    v: int,
) -> TrajectorySample:
    return TrajectorySample(t_ms, True, x_norm, y_norm, u, v)


def test_quality_gate_accepts_normal_slow_movement():
    raw_samples = (
        quality_sample(0.0, 0.10, 0.10, 100, 100),
        quality_sample(100.0, 0.12, 0.13, 120, 130),
    )

    result = apply_quality_gate(raw_samples)

    assert result.processed_samples == raw_samples
    assert result.accepted_count == 2
    assert result.rejected_speed_count == 0


def test_quality_gate_rejects_large_jump_and_preserves_coordinates():
    jumping_sample = quality_sample(10.0, 0.90, 0.80, 900, 800)
    raw_samples = (
        quality_sample(0.0, 0.10, 0.10, 100, 100),
        jumping_sample,
    )

    result = apply_quality_gate(
        raw_samples,
        QualityGateConfig(max_normalized_speed=5.0),
    )
    rejected = result.processed_samples[1]

    assert rejected.valid is False
    assert rejected.invalid_reason == "speed_gate"
    assert (rejected.x_norm, rejected.y_norm, rejected.u, rejected.v) == (
        jumping_sample.x_norm,
        jumping_sample.y_norm,
        jumping_sample.u,
        jumping_sample.v,
    )
    assert result.rejected_speed_count == 1


def test_quality_gate_preserves_no_hand_and_resets_reference_after_gap():
    no_hand = TrajectorySample(
        100.0,
        False,
        None,
        None,
        None,
        None,
        "no_hand",
    )
    after_gap = quality_sample(110.0, 0.95, 0.95, 950, 950)
    raw_samples = (
        quality_sample(0.0, 0.05, 0.05, 50, 50),
        no_hand,
        after_gap,
    )

    result = apply_quality_gate(raw_samples)

    assert result.processed_samples[1] is no_hand
    assert result.processed_samples[2] is after_gap
    assert result.processed_samples[2].valid is True
    assert result.original_invalid_count == 1
    assert result.accepted_count == 2


def test_quality_gate_does_not_modify_raw_input():
    raw_samples = (
        quality_sample(0.0, 0.10, 0.10, 100, 100),
        quality_sample(10.0, 0.90, 0.90, 900, 900),
    )
    original_snapshot = tuple(raw_samples)

    result = apply_quality_gate(raw_samples)

    assert raw_samples == original_snapshot
    assert raw_samples[1].valid is True
    assert result.processed_samples is not raw_samples
    assert result.processed_samples[1].valid is False


@pytest.mark.parametrize("t_ms", [100.0, 90.0])
def test_quality_gate_rejects_non_positive_dt(t_ms):
    raw_samples = (
        quality_sample(100.0, 0.10, 0.10, 100, 100),
        quality_sample(t_ms, 0.11, 0.11, 110, 110),
    )

    result = apply_quality_gate(raw_samples)

    rejected = result.processed_samples[1]
    assert rejected.valid is False
    assert rejected.invalid_reason == "invalid_time"
    assert result.rejected_time_count == 1


def test_speed_rejected_sample_does_not_replace_last_accepted_reference():
    raw_samples = (
        quality_sample(0.0, 0.10, 0.10, 100, 100),
        quality_sample(10.0, 0.90, 0.90, 900, 900),
        quality_sample(200.0, 0.12, 0.12, 120, 120),
    )

    result = apply_quality_gate(raw_samples)

    assert result.processed_samples[1].invalid_reason == "speed_gate"
    assert result.processed_samples[2].valid is True
    assert result.accepted_count == 2


def test_display_pixel_mirrors_only_horizontal_coordinate():
    assert display_pixel((0, 25), frame_width=100, mirror=True) == (99, 25)
    assert display_pixel((99, 25), frame_width=100, mirror=True) == (0, 25)
    assert display_pixel((30, 25), frame_width=100, mirror=False) == (30, 25)


@pytest.mark.parametrize("alpha", [0.0, -0.1, 1.01])
def test_ema_alpha_rejects_out_of_range_values(alpha):
    with pytest.raises(ValueError, match="0 < alpha <= 1"):
        EMAConfig(alpha=alpha)


def filter_metadata(width: int = 101, height: int = 101) -> TrajectoryMetadata:
    return TrajectoryMetadata(
        schema_version="1.0",
        trajectory_id="filter-test",
        frame_width=width,
        frame_height=height,
        camera_index=1,
        recording_start_timestamp_ms=1_000.0,
    )


def test_ema_alpha_one_matches_valid_input_values():
    samples = (
        quality_sample(0.0, 0.10, 0.20, 10, 20),
        quality_sample(100.0, 0.80, 0.70, 80, 70),
    )

    filtered = apply_ema_filter(samples, filter_metadata(), EMAConfig(alpha=1.0))

    assert filtered == samples
    assert filtered is not samples


def test_ema_smooths_continuous_valid_samples():
    samples = (
        quality_sample(0.0, 0.0, 0.0, 0, 0),
        quality_sample(100.0, 1.0, 1.0, 100, 100),
    )

    filtered = apply_ema_filter(samples, filter_metadata(), EMAConfig(alpha=0.25))

    assert filtered[0].x_norm == pytest.approx(0.0)
    assert filtered[1].x_norm == pytest.approx(0.25)
    assert filtered[1].y_norm == pytest.approx(0.25)
    assert (filtered[1].u, filtered[1].v) == (25, 25)


def test_ema_preserves_invalid_gap_and_reinitializes_after_gap():
    invalid = TrajectorySample(
        100.0,
        False,
        0.90,
        0.90,
        90,
        90,
        "speed_gate",
    )
    after_gap = quality_sample(200.0, 0.80, 0.70, 80, 70)
    samples = (
        quality_sample(0.0, 0.10, 0.20, 10, 20),
        invalid,
        after_gap,
    )

    filtered = apply_ema_filter(samples, filter_metadata())

    assert filtered[1] == invalid
    assert filtered[1] is not invalid
    assert filtered[2].x_norm == pytest.approx(after_gap.x_norm)
    assert filtered[2].y_norm == pytest.approx(after_gap.y_norm)
    assert (filtered[2].u, filtered[2].v) == (80, 70)


def test_ema_preserves_no_hand_as_a_new_invalid_sample():
    no_hand = TrajectorySample(
        100.0,
        False,
        None,
        None,
        None,
        None,
        "no_hand",
    )

    filtered = apply_ema_filter((no_hand,), filter_metadata())

    assert filtered == (no_hand,)
    assert filtered[0] is not no_hand


def test_ema_does_not_modify_input_and_recomputes_consistent_pixels():
    samples = (
        quality_sample(0.0, 0.20, 0.40, 20, 40),
        quality_sample(100.0, 0.60, 0.80, 60, 80),
    )
    snapshot = tuple(samples)

    filtered = apply_ema_filter(
        samples,
        filter_metadata(width=201, height=101),
        EMAConfig(alpha=0.5),
    )

    assert samples == snapshot
    assert filtered[1].x_norm == pytest.approx(0.40)
    assert filtered[1].y_norm == pytest.approx(0.60)
    assert (filtered[1].u, filtered[1].v) == (80, 60)


def test_demo_alpha_presets_are_explicit_and_include_identity_filter():
    assert EMA_ALPHA_PRESETS == {
        ord("1"): 0.15,
        ord("2"): 0.25,
        ord("3"): 0.50,
        ord("4"): 1.00,
    }


def test_display_derivation_preserves_raw_and_filters_quality_output():
    raw_samples = (
        quality_sample(0.0, 0.10, 0.10, 10, 10),
        quality_sample(10.0, 0.90, 0.90, 90, 90),
        quality_sample(200.0, 0.12, 0.12, 12, 12),
    )
    raw_snapshot = tuple(raw_samples)

    quality_result, filtered = derive_display_trajectories(
        raw_samples,
        filter_metadata(),
        alpha=1.0,
    )

    assert raw_samples == raw_snapshot
    assert raw_samples[1].valid is True
    assert quality_result.processed_samples[1].invalid_reason == "speed_gate"
    assert filtered[1].invalid_reason == "speed_gate"
    assert filtered[2] == quality_result.processed_samples[2]
    assert filtered is not raw_samples


def persisted_trajectory() -> Trajectory2D:
    return Trajectory2D(
        metadata=TrajectoryMetadata(
            schema_version="1.0",
            trajectory_id="round-trip-trajectory",
            frame_width=1280,
            frame_height=720,
            camera_index=1,
            recording_start_timestamp_ms=10_000.0,
        ),
        raw_samples=(
            quality_sample(0.0, 0.10, 0.20, 128, 144),
            TrajectorySample(
                50.0,
                False,
                None,
                None,
                None,
                None,
                "no_hand",
            ),
            TrajectorySample(
                100.0,
                False,
                0.90,
                0.80,
                1151,
                575,
                "speed_gate",
            ),
        ),
    )


def test_trajectory_json_round_trip_preserves_raw_contract_and_processing(tmp_path):
    trajectory = persisted_trajectory()
    quality_config = QualityGateConfig(max_normalized_speed=4.25)
    ema_config = EMAConfig(alpha=0.5)

    saved_path = save_trajectory_json(
        trajectory,
        quality_config,
        ema_config,
        output_directory=tmp_path / "nested" / "trajectories",
    )
    loaded = load_trajectory_json(saved_path)

    assert loaded.trajectory.metadata == trajectory.metadata
    assert loaded.trajectory.raw_samples == trajectory.raw_samples
    assert loaded.trajectory.raw_samples[1].invalid_reason == "no_hand"
    assert loaded.trajectory.raw_samples[2].invalid_reason == "speed_gate"
    assert loaded.processing.quality_gate == quality_config
    assert loaded.processing.ema == ema_config


def test_loaded_raw_samples_can_rebuild_quality_and_ema_derivatives(tmp_path):
    trajectory = persisted_trajectory()
    path = save_trajectory_json(trajectory, output_directory=tmp_path)

    loaded = load_trajectory_json(path)
    quality = apply_quality_gate(
        loaded.trajectory.raw_samples,
        loaded.processing.quality_gate,
    )
    filtered = apply_ema_filter(
        quality.processed_samples,
        loaded.trajectory.metadata,
        loaded.processing.ema,
    )

    assert len(filtered) == len(loaded.trajectory.raw_samples)
    assert loaded.trajectory.raw_samples == trajectory.raw_samples


def test_save_creates_directory_and_readable_non_overwriting_names(tmp_path):
    output_directory = tmp_path / "not-created-yet" / "trajectories"

    first = save_trajectory_json(
        persisted_trajectory(), output_directory=output_directory
    )
    second = save_trajectory_json(
        persisted_trajectory(), output_directory=output_directory
    )

    assert output_directory.is_dir()
    assert first.is_file() and second.is_file()
    assert first != second
    assert first.name.startswith("round-trip-trajectory_")


@pytest.mark.parametrize(
    "payload",
    [
        "{not valid json",
        json.dumps({"schema_version": "1.0", "trajectory_id": "missing"}),
        json.dumps(
            {
                "schema_version": "9.9",
                "trajectory_id": "unsupported",
                "metadata": {},
                "raw_samples": [],
                "processing": {},
            }
        ),
    ],
)
def test_load_rejects_malformed_or_incomplete_json(tmp_path, payload):
    path = tmp_path / "malformed.json"
    path.write_text(payload, encoding="utf-8")

    with pytest.raises(ValueError):
        load_trajectory_json(path)


def playback_trajectory() -> Trajectory2D:
    return Trajectory2D(
        metadata=filter_metadata(),
        raw_samples=(
            quality_sample(0.0, 0.10, 0.10, 10, 10),
            quality_sample(85.0, 0.20, 0.20, 20, 20),
            TrajectorySample(190.0, False, None, None, None, None, "no_hand"),
            quality_sample(310.0, 0.40, 0.40, 40, 40),
        ),
    )


def test_playback_timeline_preserves_time_order_and_source_duration():
    timeline = build_playback_timeline(playback_trajectory())

    assert [event.sample.t_ms for event in timeline.events] == [0.0, 85.0, 190.0, 310.0]
    assert [event.due_ms for event in timeline.events] == [0.0, 85.0, 190.0, 310.0]
    assert timeline.source_duration_ms == 310.0
    assert timeline.playback_duration_ms == 310.0


@pytest.mark.parametrize(
    ("speed", "expected_duration"),
    [(0.5, 620.0), (1.0, 310.0), (2.0, 155.0)],
)
def test_playback_speed_scales_due_times_and_duration(speed, expected_duration):
    timeline = build_playback_timeline(
        playback_trajectory(), playback_speed=speed
    )

    assert timeline.playback_duration_ms == pytest.approx(expected_duration)
    assert timeline.events[1].due_ms == pytest.approx(85.0 / speed)


def test_playback_due_prefix_preserves_invalid_gap_and_raw_input():
    trajectory = playback_trajectory()
    raw_snapshot = tuple(trajectory.raw_samples)
    timeline = build_playback_timeline(trajectory)

    due = timeline.samples_due(200.0)

    assert due == trajectory.raw_samples[:3]
    assert due[-1].valid is False
    assert due[-1].invalid_reason == "no_hand"
    assert trajectory.raw_samples == raw_snapshot
    assert timeline.events[2].sample is trajectory.raw_samples[2]


def test_playback_accepts_loaded_trajectory_and_empty_trajectory(tmp_path):
    empty = Trajectory2D(metadata=filter_metadata(), raw_samples=())
    path = save_trajectory_json(empty, output_directory=tmp_path)
    loaded = load_trajectory_json(path)

    timeline = build_playback_timeline(loaded)

    assert timeline.events == ()
    assert timeline.source_duration_ms == 0.0
    assert timeline.playback_duration_ms == 0.0
    assert timeline.samples_due(1_000.0) == ()
