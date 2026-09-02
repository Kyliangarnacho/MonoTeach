"""Minimal C920 acceptance demo for Banter Task 2 Token / Chord / Sequence."""

from __future__ import annotations

import argparse
import time
from typing import Sequence

import cv2

from stage2.camera_stream import CameraConfig, CameraStream

from .demo_gesture_events import draw_perception
from .gesture_events import GestureEventStabilizer
from .gesture_grammar import GestureGrammarEngine
from .gesture_perception import LandmarkGesturePerceiver
from .grammar_contracts import GrammarUpdate


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-index", type=int, default=CameraConfig().index)
    parser.add_argument("--no-mirror", action="store_true", help="Start with an unmirrored preview.")
    return parser


def grammar_status_lines(update: GrammarUpdate, phrase_count: int) -> tuple[str, ...]:
    """Build the derived-grammar HUD independently of a camera window."""
    token_text = "None" if not update.tokens else ", ".join(
        f"{event.hand_key.value}:{event.gesture.value}" for event in update.tokens
    )
    phrase_text = "None" if not update.phrases else ", ".join(
        f"{phrase.name}#{phrase.phrase_id}" for phrase in update.phrases
    )
    return (f"New tokens: {token_text}", f"New phrases: {phrase_text}", f"Phrases: {phrase_count}")


def _draw_grammar_status(frame, update: GrammarUpdate, phrase_count: int) -> None:
    height = frame.shape[0]
    for index, line in enumerate(grammar_status_lines(update, phrase_count)):
        cv2.putText(
            frame,
            line,
            (12, height - 68 + index * 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255) if "phrases:" in line.lower() else (255, 255, 255),
            2,
            cv2.LINE_AA,
        )


def main(argv: Sequence[str] | None = None) -> int:
    """Show raw Task 1 evidence plus new Task 2 grammar facts until Q."""
    arguments = _argument_parser().parse_args(argv)
    stream = CameraStream(CameraConfig(index=arguments.camera_index))
    mirror_preview = not arguments.no_mirror
    token_count = 0
    phrase_count = 0
    latest_update = GrammarUpdate(tokens=(), phrases=())
    try:
        profile = stream.open()
        print(f"Camera opened: backend={profile.backend}, resolution={profile.width}x{profile.height}, fps={profile.fps:.3f}")
        print("Chord: LEFT:FIST + RIGHT:VICTORY_TWO within about 0.45 s. Sequences begin with OPEN_PALM_FIVE: OPEN -> FIST -> VICTORY_TWO, OPEN -> POINT_ONE -> THUMBS_UP, or OPEN -> VICTORY_TWO -> THUMBS_UP (each step <= 2.5 s; total <= 6.0 s).")
        print("Press M to mirror the preview; Q to exit.")
        with LandmarkGesturePerceiver() as perceiver:
            stabilizer = GestureEventStabilizer()
            grammar = GestureGrammarEngine()
            previous_time = time.perf_counter()
            while True:
                raw_frame, capture_t_ms = stream.read()
                perception = perceiver.process(raw_frame, capture_t_ms)
                tokens = stabilizer.update(perception)
                latest_update = grammar.update(tokens)
                for token in latest_update.tokens:
                    token_count += 1
                    print({"kind": "TOKEN", **token.to_dict()})
                for phrase in latest_update.phrases:
                    phrase_count += 1
                    print({"kind": "PHRASE", **phrase.to_dict()})

                now = time.perf_counter()
                fps = 1.0 / max(now - previous_time, 1e-9)
                previous_time = now
                display_frame = cv2.flip(raw_frame, 1) if mirror_preview else raw_frame.copy()
                draw_perception(display_frame, perception, fps, mirror_preview, token_count)
                _draw_grammar_status(display_frame, latest_update, phrase_count)
                cv2.imshow("MonoTeach Banter Task 2 Gesture Grammar", display_frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q")):
                    return 0
                if key in (ord("m"), ord("M")):
                    mirror_preview = not mirror_preview
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"Banter grammar demo failed: {error}")
        return 1
    finally:
        stream.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
