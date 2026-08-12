"""OpenCV playback demo for a saved Stage 2.2 trajectory JSON."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

import cv2
import numpy as np

from .demo_trajectory_record import draw_recorded_trajectory
from .trajectory_filter import apply_ema_filter
from .trajectory_io import load_trajectory_json
from .trajectory_playback import build_playback_timeline
from .trajectory_quality import apply_quality_gate


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Play a saved MonoTeach trajectory.")
    parser.add_argument("trajectory_json", type=Path, help="Saved trajectory JSON path")
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        choices=(0.5, 1.0, 2.0),
        help="Playback speed (default: 1.0)",
    )
    parser.add_argument(
        "--show-raw",
        action="store_true",
        help="Show the raw trajectory in addition to the filtered trajectory",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        loaded = load_trajectory_json(args.trajectory_json)
        trajectory = loaded.trajectory
        quality_result = apply_quality_gate(
            trajectory.raw_samples,
            loaded.processing.quality_gate,
        )
        filtered_samples = apply_ema_filter(
            quality_result.processed_samples,
            trajectory.metadata,
            loaded.processing.ema,
        )
        raw_timeline = build_playback_timeline(
            loaded,
            playback_speed=args.speed,
        )
        filtered_timeline = build_playback_timeline(
            loaded,
            samples=filtered_samples,
            playback_speed=args.speed,
        )
    except (OSError, ValueError) as error:
        print(f"Trajectory playback failed: {error}")
        return 1

    show_raw = args.show_raw
    show_filtered = True
    paused = False
    elapsed_playback_ms = 0.0
    previous_tick = time.perf_counter()
    window_name = "MonoTeach Stage 2.2 Trajectory Playback"
    print("Space: pause/resume | R: restart | W: raw | F: filtered | Q: quit")

    while True:
        now = time.perf_counter()
        if not paused:
            elapsed_playback_ms = min(
                elapsed_playback_ms + (now - previous_tick) * 1_000.0,
                filtered_timeline.playback_duration_ms,
            )
        previous_tick = now

        metadata = trajectory.metadata
        canvas = np.zeros(
            (metadata.frame_height, metadata.frame_width, 3),
            dtype=np.uint8,
        )
        if show_raw:
            draw_recorded_trajectory(
                canvas,
                raw_timeline.samples_due(elapsed_playback_ms),
                color=(255, 80, 255),
                thickness=1,
            )
        if show_filtered:
            draw_recorded_trajectory(
                canvas,
                filtered_timeline.samples_due(elapsed_playback_ms),
                color=(80, 255, 80),
                thickness=3,
            )

        source_time_ms = min(
            elapsed_playback_ms * filtered_timeline.playback_speed,
            filtered_timeline.source_duration_ms,
        )
        status = (
            f"Time: {source_time_ms / 1000.0:.2f} / "
            f"{filtered_timeline.source_duration_ms / 1000.0:.2f} s",
            f"Speed: {filtered_timeline.playback_speed:.1f}x | "
            f"{'PAUSED' if paused else 'PLAYING'}",
            f"Raw: {'ON' if show_raw else 'OFF'} | "
            f"Filtered: {'ON' if show_filtered else 'OFF'}",
            "Space: pause | R: restart | W: raw | F: filtered | Q: quit",
        )
        for line_number, line in enumerate(status):
            cv2.putText(
                canvas,
                line,
                (12, 30 + line_number * 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
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
        elif key in (ord("w"), ord("W")):
            show_raw = not show_raw
        elif key in (ord("f"), ord("F")):
            show_filtered = not show_filtered


if __name__ == "__main__":
    raise SystemExit(main())
