"""Live Stage 2.1 fingertip visualization using the existing camera pipeline."""

from __future__ import annotations

import time

import cv2
import numpy as np

from .camera_stream import CameraConfig, CameraStream
from .fingertip import normalized_to_pixel
from .hand_observation import HandObservation
from .hand_tracker import HandTracker


HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
)


def status_lines(observation: HandObservation, fps: float) -> tuple[str, ...]:
    """Build display text without depending on a camera or OpenCV window."""
    lines = (f"FPS: {fps:.1f}",)
    if not observation.detected:
        return lines + ("No hand",)

    assert observation.index_tip_norm is not None
    assert observation.index_tip_px is not None
    handedness = observation.handedness or "Unknown"
    score = observation.handedness_score or 0.0
    x_norm, y_norm = observation.index_tip_norm
    u_px, v_px = observation.index_tip_px
    return lines + (
        f"Hand: {handedness} ({score:.2f})",
        f"Index tip norm: x={x_norm:.3f}, y={y_norm:.3f}",
        f"Index tip px: u={u_px}, v={v_px}",
    )


def draw_observation(frame: np.ndarray, observation: HandObservation, fps: float) -> None:
    """Draw hand landmarks, their skeleton, fingertip emphasis, and status text."""
    frame_height, frame_width = frame.shape[:2]

    if observation.detected:
        landmark_pixels = tuple(
            normalized_to_pixel(x_norm, y_norm, frame_width, frame_height)
            for x_norm, y_norm, _z_norm in observation.landmarks_norm
        )
        for start, end in HAND_CONNECTIONS:
            if start < len(landmark_pixels) and end < len(landmark_pixels):
                cv2.line(frame, landmark_pixels[start], landmark_pixels[end], (80, 220, 80), 2)
        for pixel in landmark_pixels:
            cv2.circle(frame, pixel, 3, (255, 255, 255), cv2.FILLED)

        if observation.index_tip_px is not None:
            cv2.circle(frame, observation.index_tip_px, 11, (0, 220, 255), 2)
            cv2.circle(frame, observation.index_tip_px, 5, (0, 80, 255), cv2.FILLED)

    for line_number, line in enumerate(status_lines(observation, fps)):
        color = (0, 0, 255) if line == "No hand" else (255, 255, 255)
        cv2.putText(
            frame,
            line,
            (12, 30 + line_number * 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
            cv2.LINE_AA,
        )


def main() -> int:
    """Run the live camera-to-fingertip visualization until q is pressed."""
    stream = CameraStream(CameraConfig())
    try:
        profile = stream.open()
        print(
            "Camera opened: "
            f"backend={profile.backend}, "
            f"resolution={profile.width}x{profile.height}, "
            f"fps={profile.fps:.3f}"
        )
        print("Press q in the preview window to exit.")

        with HandTracker() as tracker:
            previous_time = time.perf_counter()
            while True:
                frame, timestamp_ms = stream.read()
                observation = tracker.process(frame, timestamp_ms)

                now = time.perf_counter()
                elapsed_seconds = max(now - previous_time, 1e-9)
                previous_time = now
                draw_observation(frame, observation, 1.0 / elapsed_seconds)

                cv2.imshow("MonoTeach Stage 2.1 Live Fingertip", frame)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    return 0
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"Live fingertip demo failed: {error}")
        return 1
    finally:
        stream.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
