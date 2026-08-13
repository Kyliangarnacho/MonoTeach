"""OpenCV millimetre playback for a saved workspace trajectory JSON artifact."""

from __future__ import annotations

import argparse
from pathlib import Path
import time
from typing import Sequence

import cv2
import numpy as np

from .trajectory_playback import build_playback_timeline
from .workspace_trajectory import WorkspaceTrajectory2D, WorkspaceTrajectorySample
from .workspace_trajectory_io import load_workspace_trajectory_json


CANVAS_MAX_WIDTH_PX = 900
CANVAS_MAX_HEIGHT_PX = 720
MARGIN_PX = 55


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("workspace_trajectory_json", type=Path)
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        choices=(0.5, 1.0, 2.0),
        help="Playback speed (default: 1.0)",
    )
    return parser


def _canvas_geometry(trajectory: WorkspaceTrajectory2D) -> tuple[int, int, float]:
    metadata = trajectory.metadata
    available_width = CANVAS_MAX_WIDTH_PX - 2 * MARGIN_PX
    available_height = CANVAS_MAX_HEIGHT_PX - 2 * MARGIN_PX
    pixels_per_mm = min(
        available_width / metadata.width_mm,
        available_height / metadata.height_mm,
    )
    workspace_width_px = int(round(metadata.width_mm * pixels_per_mm))
    workspace_height_px = int(round(metadata.height_mm * pixels_per_mm))
    return workspace_width_px + 2 * MARGIN_PX, workspace_height_px + 2 * MARGIN_PX, pixels_per_mm


def _workspace_pixel(
    x_mm: float,
    y_mm: float,
    pixels_per_mm: float,
) -> tuple[int, int]:
    return (
        int(round(MARGIN_PX + x_mm * pixels_per_mm)),
        int(round(MARGIN_PX + y_mm * pixels_per_mm)),
    )


def _draw_workspace_trajectory(
    canvas: np.ndarray,
    samples: Sequence[WorkspaceTrajectorySample],
    pixels_per_mm: float,
) -> int:
    previous_pixel: tuple[int, int] | None = None
    outside_count = 0
    for sample in samples:
        if not sample.valid:
            previous_pixel = None
            continue
        assert sample.x_mm is not None and sample.y_mm is not None
        pixel = _workspace_pixel(sample.x_mm, sample.y_mm, pixels_per_mm)
        color = (0, 165, 255) if not sample.inside_workspace else (80, 255, 80)
        if previous_pixel is not None:
            cv2.line(canvas, previous_pixel, pixel, color, 2, cv2.LINE_AA)
        cv2.circle(canvas, pixel, 4, color, -1, cv2.LINE_AA)
        previous_pixel = pixel
        if not sample.inside_workspace:
            outside_count += 1
    return outside_count


def main(argv: Sequence[str] | None = None) -> int:
    """Play a millimetre trajectory with pause, restart, and time scaling controls."""
    arguments = _argument_parser().parse_args(argv)
    try:
        trajectory = load_workspace_trajectory_json(arguments.workspace_trajectory_json)
        timeline = build_playback_timeline(trajectory, playback_speed=arguments.speed)
    except (OSError, TypeError, ValueError) as error:
        print(f"Workspace trajectory playback failed: {error}")
        return 1

    paused = False
    elapsed_playback_ms = 0.0
    previous_tick = time.perf_counter()
    window_name = "MonoTeach Stage 2.3 Workspace Trajectory Playback"
    canvas_width, canvas_height, pixels_per_mm = _canvas_geometry(trajectory)
    print("Space: pause/resume | R: restart | Q: quit")

    while True:
        now = time.perf_counter()
        if not paused:
            elapsed_playback_ms = min(
                elapsed_playback_ms + (now - previous_tick) * 1_000.0,
                timeline.playback_duration_ms,
            )
        previous_tick = now

        canvas = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)
        top_left = (MARGIN_PX, MARGIN_PX)
        bottom_right = (
            canvas_width - MARGIN_PX,
            canvas_height - MARGIN_PX,
        )
        cv2.rectangle(canvas, top_left, bottom_right, (255, 255, 255), 2)
        due_samples = timeline.samples_due(elapsed_playback_ms)
        outside_count = _draw_workspace_trajectory(canvas, due_samples, pixels_per_mm)
        source_time_ms = min(
            elapsed_playback_ms * timeline.playback_speed,
            timeline.source_duration_ms,
        )
        status = (
            f"Workspace: {trajectory.metadata.width_mm:g} x {trajectory.metadata.height_mm:g} mm",
            "X (mm) ->    Y (mm) down",
            f"Time: {source_time_ms / 1000.0:.2f} / {timeline.source_duration_ms / 1000.0:.2f} s",
            f"Speed: {timeline.playback_speed:.1f}x | {'PAUSED' if paused else 'PLAYING'}",
            f"Outside valid samples shown: {outside_count}",
            "Space: pause/resume | R: restart | Q: quit",
        )
        for line_number, line in enumerate(status):
            cv2.putText(
                canvas,
                line,
                (12, 22 + line_number * 23),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        cv2.imshow(window_name, canvas)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), ord("Q")):
            cv2.destroyAllWindows()
            return 0
        if key == ord(" "):
            paused = not paused
        elif key in (ord("r"), ord("R")):
            elapsed_playback_ms = 0.0
            paused = False
            previous_tick = time.perf_counter()


if __name__ == "__main__":
    raise SystemExit(main())
