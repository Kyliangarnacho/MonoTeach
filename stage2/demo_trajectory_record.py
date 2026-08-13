"""Live Stage 2.2 fingertip trajectory recording and visualization."""

from __future__ import annotations

import argparse
from collections.abc import Iterable
from pathlib import Path
import time
from typing import Sequence

import cv2
import numpy as np

from .camera_stream import CameraConfig, CameraStream
from .demo_fingertip_live import draw_observation
from .display_geometry import display_pixel
from .hand_tracker import HandTracker
from .trajectory import TrajectoryMetadata, TrajectorySample
from .trajectory_filter import EMAConfig, apply_ema_filter
from .trajectory_io import save_trajectory_json
from .trajectory_quality import (
    QualityGateConfig,
    TrajectoryQualityResult,
    apply_quality_gate,
)
from .trajectory_recorder import RecorderState, TrajectoryRecorder


EMA_ALPHA_PRESETS = {
    ord("1"): 0.15,
    ord("2"): 0.25,
    ord("3"): 0.50,
    ord("4"): 1.00,
}


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--camera-index",
        type=int,
        default=CameraConfig().index,
        help="OpenCV camera index (default: CameraConfig default)",
    )
    return parser


def derive_display_trajectories(
    raw_samples: tuple[TrajectorySample, ...],
    metadata: TrajectoryMetadata,
    alpha: float,
) -> tuple[TrajectoryQualityResult, tuple[TrajectorySample, ...]]:
    """Build quality-gated and EMA-filtered derivatives for visualization."""
    quality_result = apply_quality_gate(raw_samples)
    filtered_samples = apply_ema_filter(
        quality_result.processed_samples,
        metadata,
        EMAConfig(alpha=alpha),
    )
    return quality_result, filtered_samples


def valid_pixel_segments(
    samples: Iterable[TrajectorySample],
) -> tuple[tuple[tuple[int, int], ...], ...]:
    """Split samples into valid pixel segments without bridging invalid gaps."""
    segments: list[tuple[tuple[int, int], ...]] = []
    current: list[tuple[int, int]] = []

    for sample in samples:
        if sample.valid:
            assert sample.u is not None and sample.v is not None
            current.append((sample.u, sample.v))
            continue

        if current:
            segments.append(tuple(current))
            current = []

    if current:
        segments.append(tuple(current))
    return tuple(segments)


def draw_recorded_trajectory(
    frame: np.ndarray,
    samples: tuple[TrajectorySample, ...],
    mirror_preview: bool = False,
    color: tuple[int, int, int] = (255, 80, 255),
    thickness: int = 2,
) -> None:
    """Draw each continuous valid segment independently on the current frame."""
    frame_width = frame.shape[1]
    for raw_segment in valid_pixel_segments(samples):
        segment = tuple(
            display_pixel(pixel, frame_width, mirror_preview)
            for pixel in raw_segment
        )
        if len(segment) == 1:
            cv2.circle(frame, segment[0], max(thickness, 2), color, cv2.FILLED)
            continue
        points = np.asarray(segment, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(frame, [points], False, color, thickness, cv2.LINE_AA)


def recording_status_lines(
    recorder: TrajectoryRecorder,
    quality_result: TrajectoryQualityResult | None,
    ema_alpha: float,
    notice: str | None = None,
    mirror_preview: bool | None = None,
) -> tuple[str, ...]:
    """Build recorder status text without a camera dependency."""
    samples = recorder.samples
    duration_ms = samples[-1].t_ms if samples else 0.0
    accepted_count = quality_result.accepted_count if quality_result else 0
    rejected_speed_count = (
        quality_result.rejected_speed_count if quality_result else 0
    )
    original_invalid_count = (
        quality_result.original_invalid_count if quality_result else 0
    )
    lines = (
        f"Recorder: {recorder.state.name}",
        f"Raw samples: {recorder.sample_count}",
        f"Accepted: {accepted_count}",
        f"Speed rejected: {rejected_speed_count}",
        f"Original invalid: {original_invalid_count}",
        f"EMA alpha: {ema_alpha:.2f} (1/2/3/4)",
        f"Duration: {duration_ms / 1_000.0:.2f} s",
        "Raw: magenta | Filtered: green",
        "Space: start/stop | S: save | P: playback help | R/M/Q",
    )
    if mirror_preview is not None:
        lines += (f"Mirror: {'ON' if mirror_preview else 'OFF'}",)
    return lines + ((notice,) if notice else ())


def draw_recording_status(
    frame: np.ndarray,
    recorder: TrajectoryRecorder,
    quality_result: TrajectoryQualityResult | None,
    ema_alpha: float,
    notice: str | None,
    mirror_preview: bool,
) -> None:
    """Overlay recorder state, counts, duration, controls, and optional notice."""
    for line_number, line in enumerate(
        recording_status_lines(
            recorder,
            quality_result,
            ema_alpha,
            notice,
            mirror_preview,
        )
    ):
        color = (0, 255, 255) if line_number == 0 else (255, 255, 255)
        if notice and line == notice:
            color = (0, 165, 255)
        cv2.putText(
            frame,
            line,
            (12, 185 + line_number * 27),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            color,
            2,
            cv2.LINE_AA,
        )


def main(argv: Sequence[str] | None = None) -> int:
    """Run live fingertip tracking with explicit trajectory recording controls."""
    arguments = _argument_parser().parse_args(argv)
    config = CameraConfig(index=arguments.camera_index)
    stream = CameraStream(config)
    recorder = TrajectoryRecorder()
    recording_metadata: TrajectoryMetadata | None = None
    notice: str | None = None
    last_saved_path: Path | None = None
    mirror_preview = True
    ema_alpha = EMAConfig().alpha

    try:
        profile = stream.open()
        print(
            "Camera opened: "
            f"backend={profile.backend}, "
            f"resolution={profile.width}x{profile.height}, "
            f"fps={profile.fps:.3f}"
        )
        print(
            "Space: start/stop | R: reset | M: mirror | "
            "S: save | P: playback help | 1/2/3/4: EMA alpha | Q: quit"
        )

        with HandTracker() as tracker:
            previous_time = time.perf_counter()
            while True:
                frame, timestamp_ms = stream.read()
                observation = tracker.process(frame, timestamp_ms)

                if recorder.state is RecorderState.RECORDING:
                    recorder.record(observation)

                now = time.perf_counter()
                elapsed_seconds = max(now - previous_time, 1e-9)
                previous_time = now

                raw_samples = recorder.samples
                quality_result: TrajectoryQualityResult | None = None
                filtered_samples: tuple[TrajectorySample, ...] = ()
                if recording_metadata is not None:
                    quality_result, filtered_samples = derive_display_trajectories(
                        raw_samples,
                        recording_metadata,
                        ema_alpha,
                    )

                display_frame = cv2.flip(frame, 1) if mirror_preview else frame.copy()
                draw_recorded_trajectory(
                    display_frame,
                    raw_samples,
                    mirror_preview,
                    color=(255, 80, 255),
                    thickness=1,
                )
                draw_recorded_trajectory(
                    display_frame,
                    filtered_samples,
                    mirror_preview,
                    color=(80, 255, 80),
                    thickness=3,
                )
                draw_observation(
                    display_frame,
                    observation,
                    1.0 / elapsed_seconds,
                    mirror_preview,
                )
                draw_recording_status(
                    display_frame,
                    recorder,
                    quality_result,
                    ema_alpha,
                    notice,
                    mirror_preview,
                )

                cv2.imshow("MonoTeach Stage 2.2 Trajectory Recorder", display_frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q")):
                    return 0
                if key in (ord("r"), ord("R")):
                    recorder.reset()
                    recording_metadata = None
                    last_saved_path = None
                    notice = "Recorder reset; press Space to start."
                    continue
                if key in (ord("m"), ord("M")):
                    mirror_preview = not mirror_preview
                    notice = f"Mirror {'ON' if mirror_preview else 'OFF'}."
                    continue
                if key in EMA_ALPHA_PRESETS:
                    ema_alpha = EMA_ALPHA_PRESETS[key]
                    notice = f"EMA alpha set to {ema_alpha:.2f}."
                    continue
                if key in (ord("s"), ord("S")):
                    trajectory = recorder.finished_trajectory
                    if recorder.state is not RecorderState.READY or trajectory is None:
                        notice = "Save requires a READY trajectory."
                        continue
                    try:
                        saved_path = save_trajectory_json(
                            trajectory,
                            quality_config=QualityGateConfig(),
                            ema_config=EMAConfig(alpha=ema_alpha),
                        )
                    except OSError as error:
                        notice = f"Save failed: {error}"
                        print(notice)
                    else:
                        last_saved_path = saved_path
                        notice = f"Saved: {saved_path}"
                        print(notice)
                    continue
                if key in (ord("p"), ord("P")):
                    if last_saved_path is None:
                        notice = "Save the READY trajectory before playback."
                    else:
                        command = (
                            "python -m stage2.demo_trajectory_playback "
                            f'"{last_saved_path}"'
                        )
                        notice = f"Playback command: {command}"
                        print(notice)
                    continue
                if key != ord(" "):
                    continue

                if recorder.state is RecorderState.IDLE:
                    last_saved_path = None
                    frame_height, frame_width = frame.shape[:2]
                    recording_metadata = recorder.start(
                        recording_start_timestamp_ms=timestamp_ms,
                        frame_width=frame_width,
                        frame_height=frame_height,
                        camera_index=config.index,
                    )
                    notice = "Recording started."
                elif recorder.state is RecorderState.RECORDING:
                    recorder.stop()
                    notice = "Recording stopped; press R before a new recording."
                else:
                    notice = "Trajectory is READY; press R before recording again."
                    print(notice)
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"Trajectory recording demo failed: {error}")
        return 1
    finally:
        stream.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
