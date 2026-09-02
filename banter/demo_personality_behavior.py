"""Minimal C920 acceptance demo for Banter Task 4 persona and behavior intent."""

from __future__ import annotations

import argparse
import time
from typing import Sequence

import cv2

from stage2.camera_stream import CameraConfig, CameraStream

from .demo_gesture_events import draw_perception
from .gesture_events import GestureEventStabilizer
from .gesture_grammar import GestureGrammarConfig, GestureGrammarEngine
from .gesture_perception import LandmarkGesturePerceiver
from .grammar_contracts import GrammarUpdate
from .interaction_memory import InteractionMemory
from .personality_behavior import PersonaBehaviorEngine
from .personality_contracts import Task4Update


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-index", type=int, default=CameraConfig().index)
    parser.add_argument("--no-mirror", action="store_true", help="Start with an unmirrored preview.")
    return parser


def personality_status_lines(update: Task4Update) -> tuple[str, ...]:
    """Build a compact camera-independent Task 4 HUD summary."""
    persona = update.persona
    behavior = "None" if update.behavior is None else (
        f"#{update.behavior.behavior_id} {update.behavior.behavior_name}:{update.behavior.variant}"
    )
    return (
        f"Persona F:{persona.friendliness:.2f} P:{persona.playfulness:.2f} E:{persona.energy:.2f} A:{persona.annoyance:.2f}",
        f"Behavior: {behavior}",
        f"Memory revision: {persona.source_memory_revision} | {update.admission.value}",
    )


def _draw_personality_status(frame, update: Task4Update) -> None:
    for index, line in enumerate(personality_status_lines(update)):
        cv2.putText(
            frame,
            line,
            (12, 145 + index * 21),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 255, 255) if line.startswith("Behavior") else (255, 255, 255),
            1,
            cv2.LINE_AA,
        )


def _cleanup_step(name: str, action) -> None:
    """Print bounded cleanup diagnostics while allowing later steps to run."""
    started = time.perf_counter()
    print(f"[CLEANUP] {name} begin", flush=True)
    try:
        action()
    except Exception as error:  # pragma: no cover - hardware/backend dependent
        print(f"[CLEANUP] {name} error: {error}", flush=True)
    else:
        elapsed_ms = (time.perf_counter() - started) * 1_000.0
        print(f"[CLEANUP] {name} done {elapsed_ms:.1f} ms", flush=True)


def cleanup_demo_resources(perceiver: LandmarkGesturePerceiver | None, stream: CameraStream, window_name: str) -> None:
    """Release camera/UI before MediaPipe close so a later close stall is visible."""
    def destroy_windows() -> None:
        try:
            cv2.destroyWindow(window_name)
        finally:
            cv2.destroyAllWindows()
            cv2.waitKey(1)

    _cleanup_step("camera_release", stream.release)
    _cleanup_step("windows_destroy", destroy_windows)
    if perceiver is not None:
        _cleanup_step("perceiver_close", perceiver.close)
    print("[CLEANUP] complete", flush=True)


def main(argv: Sequence[str] | None = None) -> int:
    """Show Task 1 evidence plus Task 4 semantic, non-motion output."""
    arguments = _argument_parser().parse_args(argv)
    stream = CameraStream(CameraConfig(index=arguments.camera_index))
    mirror_preview = not arguments.no_mirror
    session_number = 1
    event_count = 0
    latest_grammar = GrammarUpdate((), ())
    window_name = "MonoTeach Banter Task 4 Persona + Behavior"
    perceiver: LandmarkGesturePerceiver | None = None
    try:
        profile = stream.open()
        print(f"Camera opened: backend={profile.backend}, resolution={profile.width}x{profile.height}, fps={profile.fps:.3f}")
        print("M mirror | R reset complete Banter persona session | Q exit")
        perceiver = LandmarkGesturePerceiver()
        grammar_config = GestureGrammarConfig()
        stabilizer = GestureEventStabilizer()
        grammar = GestureGrammarEngine(grammar_config)
        memory = InteractionMemory(f"banter-live-{session_number}")
        engine = PersonaBehaviorEngine(memory.snapshot.session_id, grammar_config=grammar_config)
        latest_update = engine.last_update
        previous_time = time.perf_counter()
        while True:
            raw_frame, capture_t_ms = stream.read()
            perception = perceiver.process(raw_frame, capture_t_ms)
            tokens = stabilizer.update(perception)
            latest_grammar = grammar.update(tokens)
            before = memory.snapshot
            snapshot = memory.update(latest_grammar)
            latest_update = engine.update(snapshot)
            latest_update = engine.advance_time(capture_t_ms)
            if snapshot is not before:
                event_count += len(latest_grammar.tokens)
                if latest_update.behavior is not None:
                    print({
                        "kind": "PERSONA_BEHAVIOR",
                        "persona": latest_update.persona.to_dict(),
                        "behavior": latest_update.behavior.to_dict(),
                    })

            now = time.perf_counter()
            fps = 1.0 / max(now - previous_time, 1e-9)
            previous_time = now
            display_frame = cv2.flip(raw_frame, 1) if mirror_preview else raw_frame.copy()
            draw_perception(display_frame, perception, fps, mirror_preview, event_count)
            _draw_personality_status(display_frame, latest_update)
            cv2.imshow(window_name, display_frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                return 0
            if key in (ord("m"), ord("M")):
                mirror_preview = not mirror_preview
            if key in (ord("r"), ord("R")):
                session_number += 1
                stabilizer = GestureEventStabilizer()
                grammar = GestureGrammarEngine(grammar_config)
                memory = InteractionMemory(f"banter-live-{session_number}")
                engine = PersonaBehaviorEngine(memory.snapshot.session_id, grammar_config=grammar_config)
                latest_grammar = GrammarUpdate((), ())
                latest_update = engine.last_update
                event_count = 0
                print({"kind": "PERSONA_RESET", "session_id": memory.snapshot.session_id})
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"Banter personality-behavior demo failed: {error}")
        return 1
    finally:
        cleanup_demo_resources(perceiver, stream, window_name)


if __name__ == "__main__":
    raise SystemExit(main())
