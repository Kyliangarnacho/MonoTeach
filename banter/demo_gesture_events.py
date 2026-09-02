"""Minimal C920 acceptance demo for Banter Task 1 GestureEvent output."""

from __future__ import annotations

import argparse
import time
from typing import Sequence

import cv2
import numpy as np

from stage2.camera_stream import CameraConfig, CameraStream
from stage2.display_geometry import display_pixel
from stage2.fingertip import normalized_to_pixel

from .contracts import GestureEvidence, GesturePerception, HandKey
from .gesture_events import GestureEventStabilizer
from .gesture_perception import LandmarkGesturePerceiver


HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
)


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-index", type=int, default=CameraConfig().index)
    parser.add_argument("--no-mirror", action="store_true", help="Start with an unmirrored preview.")
    return parser


def _label(evidence: GestureEvidence) -> str:
    """Return the one concise per-hand HUD state."""
    side = evidence.hand_key.value if evidence.hand_key is not HandKey.UNKNOWN else "HAND?"
    return f"{side}: {evidence.gesture.value}"


def status_lines(
    perception: GesturePerception,
    fps: float,
    mirror_preview: bool,
    event_count: int,
) -> tuple[str, ...]:
    """Return only global camera HUD text; hand labels sit by their skeleton."""
    lines = (
        f"FPS: {fps:.1f}",
        f"Events: {event_count}",
        f"Mirror: {'ON' if mirror_preview else 'OFF'}",
    )
    if not perception.evidence:
        return lines + ("No hand",)
    return lines


def _draw_hand(frame: np.ndarray, evidence: GestureEvidence, mirror_preview: bool) -> None:
    height, width = frame.shape[:2]
    color = (80, 220, 80) if evidence.hand_key is HandKey.LEFT else (255, 180, 60)
    pixels = tuple(
        display_pixel(normalized_to_pixel(x, y, width, height), width, mirror_preview)
        for x, y, _z in evidence.hand.landmarks_norm
    )
    for first, second in HAND_CONNECTIONS:
        if first < len(pixels) and second < len(pixels):
            cv2.line(frame, pixels[first], pixels[second], color, 2)
    for pixel in pixels:
        cv2.circle(frame, pixel, 3, (255, 255, 255), cv2.FILLED)
    if evidence.hand.index_tip_px is not None:
        tip = display_pixel(evidence.hand.index_tip_px, width, mirror_preview)
        cv2.circle(frame, tip, 10, color, 2)
        cv2.putText(frame, _label(evidence), (tip[0] + 8, max(20, tip[1] - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)


def draw_perception(
    frame: np.ndarray,
    perception: GesturePerception,
    fps: float,
    mirror_preview: bool,
    event_count: int,
) -> None:
    """Overlay raw landmarks and the Task 1 evidence/event counter."""
    for evidence in perception.evidence:
        _draw_hand(frame, evidence, mirror_preview)
    for line_index, line in enumerate(status_lines(perception, fps, mirror_preview, event_count)):
        color = (0, 0, 255) if line == "No hand" else (255, 255, 255)
        cv2.putText(frame, line, (12, 30 + line_index * 27), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)


def main(argv: Sequence[str] | None = None) -> int:
    """Show evidence continuously and print one immutable event per held pose."""
    arguments = _argument_parser().parse_args(argv)
    stream = CameraStream(CameraConfig(index=arguments.camera_index))
    mirror_preview = not arguments.no_mirror
    event_count = 0
    try:
        profile = stream.open()
        print(f"Camera opened: backend={profile.backend}, resolution={profile.width}x{profile.height}, fps={profile.fps:.3f}")
        print("Hold POINT_ONE, VICTORY_TWO, OPEN_PALM_FIVE, THUMBS_UP, FIST, or MIDDLE_FINGER for 0.4 s. Press M to mirror; Q to exit.")
        with LandmarkGesturePerceiver() as perceiver:
            stabilizer = GestureEventStabilizer()
            previous_time = time.perf_counter()
            while True:
                raw_frame, capture_t_ms = stream.read()
                perception = perceiver.process(raw_frame, capture_t_ms)
                events = stabilizer.update(perception)
                for event in events:
                    event_count += 1
                    print(event.to_dict())

                now = time.perf_counter()
                fps = 1.0 / max(now - previous_time, 1e-9)
                previous_time = now
                display_frame = cv2.flip(raw_frame, 1) if mirror_preview else raw_frame.copy()
                draw_perception(display_frame, perception, fps, mirror_preview, event_count)
                cv2.imshow("MonoTeach Banter Task 1 Gesture Events", display_frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q")):
                    return 0
                if key in (ord("m"), ord("M")):
                    mirror_preview = not mirror_preview
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"Banter gesture demo failed: {error}")
        return 1
    finally:
        stream.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
