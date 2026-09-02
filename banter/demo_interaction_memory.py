"""Minimal C920 acceptance demo for Banter Task 3 Interaction Memory."""

from __future__ import annotations

import argparse
import time
from typing import Sequence

import cv2

from stage2.camera_stream import CameraConfig, CameraStream

from .contracts import HandKey
from .demo_gesture_events import draw_perception
from .gesture_events import GestureEventStabilizer
from .gesture_grammar import GestureGrammarEngine
from .gesture_perception import LandmarkGesturePerceiver
from .grammar_contracts import GrammarUpdate
from .interaction_memory import InteractionMemory
from .memory_contracts import InteractionMemorySnapshot, PhraseStreak, TokenStreak


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-index", type=int, default=CameraConfig().index)
    parser.add_argument("--no-mirror", action="store_true", help="Start with an unmirrored preview.")
    return parser


def _token_streak_text(streak: TokenStreak | None) -> str:
    return "None" if streak is None else f"{streak.key.gesture.value} x{streak.count}"


def _phrase_streak_text(streak: PhraseStreak | None) -> str:
    return "None" if streak is None else f"{streak.key.name} x{streak.count}"


def memory_status_lines(snapshot: InteractionMemorySnapshot) -> tuple[str, ...]:
    """Build compact, camera-independent Task 3 HUD lines."""
    last_token = "None" if not snapshot.recent_tokens else (
        f"{snapshot.recent_tokens[-1].hand_key.value}:{snapshot.recent_tokens[-1].gesture.value}"
    )
    last_phrase = "None" if not snapshot.recent_phrases else snapshot.recent_phrases[-1].name
    recent_tokens = ", ".join(
        f"{item.hand_key.value[0]}:{item.gesture.value}" for item in snapshot.recent_tokens[-3:]
    ) or "None"
    recent_phrases = ", ".join(item.name for item in snapshot.recent_phrases[-3:]) or "None"
    return (
        f"Session: {snapshot.session_id} r{snapshot.revision}",
        f"Totals: T={snapshot.total_token_count} P={snapshot.total_phrase_count}",
        f"Left: {_token_streak_text(snapshot.current_token_streak(HandKey.LEFT))}",
        f"Right: {_token_streak_text(snapshot.current_token_streak(HandKey.RIGHT))}",
        f"Phrase: {_phrase_streak_text(snapshot.current_phrase_streak())}",
        f"Last: {last_token} | {last_phrase}",
        f"Recent T: {recent_tokens}",
        f"Recent P: {recent_phrases}",
    )


def _draw_memory_status(frame, snapshot: InteractionMemorySnapshot) -> None:
    height = frame.shape[0]
    lines = memory_status_lines(snapshot)
    start_y = max(20, height - 20 - 18 * len(lines))
    for index, line in enumerate(lines):
        cv2.putText(
            frame,
            line,
            (12, start_y + index * 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (0, 255, 255) if line.startswith("Totals") else (255, 255, 255),
            1,
            cv2.LINE_AA,
        )


def main(argv: Sequence[str] | None = None) -> int:
    """Show Task 1 evidence and the Task 3 canonical interaction context."""
    arguments = _argument_parser().parse_args(argv)
    stream = CameraStream(CameraConfig(index=arguments.camera_index))
    mirror_preview = not arguments.no_mirror
    session_number = 1
    event_count = 0
    latest_update = GrammarUpdate((), ())
    try:
        profile = stream.open()
        print(f"Camera opened: backend={profile.backend}, resolution={profile.width}x{profile.height}, fps={profile.fps:.3f}")
        print("M mirror | R reset complete Banter interaction session | Q exit")
        with LandmarkGesturePerceiver() as perceiver:
            stabilizer = GestureEventStabilizer()
            grammar = GestureGrammarEngine()
            memory = InteractionMemory(f"banter-live-{session_number}")
            previous_time = time.perf_counter()
            while True:
                raw_frame, capture_t_ms = stream.read()
                perception = perceiver.process(raw_frame, capture_t_ms)
                tokens = stabilizer.update(perception)
                latest_update = grammar.update(tokens)
                before = memory.snapshot
                snapshot = memory.update(latest_update)
                if snapshot is not before:
                    event_count += len(latest_update.tokens)
                    print({
                        "kind": "MEMORY",
                        "session_id": snapshot.session_id,
                        "revision": snapshot.revision,
                        "total_token_count": snapshot.total_token_count,
                        "total_phrase_count": snapshot.total_phrase_count,
                    })

                now = time.perf_counter()
                fps = 1.0 / max(now - previous_time, 1e-9)
                previous_time = now
                display_frame = cv2.flip(raw_frame, 1) if mirror_preview else raw_frame.copy()
                draw_perception(display_frame, perception, fps, mirror_preview, event_count)
                _draw_memory_status(display_frame, snapshot)
                cv2.imshow("MonoTeach Banter Task 3 Interaction Memory", display_frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q")):
                    return 0
                if key in (ord("m"), ord("M")):
                    mirror_preview = not mirror_preview
                if key in (ord("r"), ord("R")):
                    session_number += 1
                    stabilizer = GestureEventStabilizer()
                    grammar = GestureGrammarEngine()
                    memory = InteractionMemory(f"banter-live-{session_number}")
                    latest_update = GrammarUpdate((), ())
                    event_count = 0
                    print({"kind": "MEMORY_RESET", "session_id": memory.snapshot.session_id})
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"Banter interaction-memory demo failed: {error}")
        return 1
    finally:
        stream.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
