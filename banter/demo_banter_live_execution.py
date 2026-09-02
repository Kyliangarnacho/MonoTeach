"""Task 7 C920 -> gated Banter -> MATLAB Legacy5 simulated execution demo."""

from __future__ import annotations

import argparse
import time
from typing import Sequence

import cv2

from stage2.camera_stream import CameraConfig, CameraStream

from .demo_gesture_events import draw_perception
from .demo_personality_behavior import cleanup_demo_resources
from .execution_contracts import ExecutionState
from .execution_protocol import BanterExecutionClient
from .execution_runtime import BanterExecutionRuntime
from .gesture_events import GestureEventStabilizer
from .gesture_grammar import GestureGrammarConfig, GestureGrammarEngine
from .gesture_perception import LandmarkGesturePerceiver
from .interaction_arm_gate import InteractionArmGate, InteractionArmState
from .interaction_memory import InteractionMemory
from .motion_library import MotionPlanner
from .motion_styling import MotionStyler
from .personality_behavior import PersonaBehaviorEngine


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-index", type=int, default=CameraConfig().index)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=51012)
    parser.add_argument("--no-mirror", action="store_true")
    return parser


def _draw_status(frame, arm: InteractionArmState, runtime: BanterExecutionRuntime, latest_behavior: str) -> None:
    lines = (
        f"Interaction: {arm.value}",
        f"Executor: {runtime.state.value}",
        f"Behavior: {latest_behavior}",
    )
    for index, line in enumerate(lines):
        cv2.putText(frame, line, (12, 145 + index * 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 255) if index == 0 else (255, 255, 255), 1, cv2.LINE_AA)


def _wait_for_executor_shutdown(runtime: BanterExecutionRuntime, timeout_s: float = 12.0) -> None:
    """Let an already accepted motion return neutral without keeping the UI alive."""
    runtime.request_shutdown()
    deadline = time.perf_counter() + timeout_s
    while runtime.state is ExecutionState.SHUTTING_DOWN and time.perf_counter() < deadline:
        runtime.poll()
        time.sleep(0.01)
    if runtime.state is ExecutionState.SHUTTING_DOWN:
        print("[Task7] executor shutdown wait timed out; TCP connection will close.", flush=True)


def main(argv: Sequence[str] | None = None) -> int:
    arguments = _argument_parser().parse_args(argv)
    stream = CameraStream(CameraConfig(index=arguments.camera_index))
    perceiver: LandmarkGesturePerceiver | None = None
    window_name = "MonoTeach Banter Task 7 Live Execution"
    mirror = not arguments.no_mirror
    runtime: BanterExecutionRuntime | None = None
    try:
        profile = stream.open()
        print(f"Camera opened: backend={profile.backend}, resolution={profile.width}x{profile.height}, fps={profile.fps:.3f}")
        print("Task 7 starts DISARMED. Hold LEFT:FIVE + RIGHT:FIVE to ARM/DISARM. M mirror | Q exit")
        perceiver = LandmarkGesturePerceiver()
        grammar_config = GestureGrammarConfig()
        stabilizer = GestureEventStabilizer()
        gate = InteractionArmGate()
        session_id = "banter-task7-live"
        grammar = GestureGrammarEngine(grammar_config)
        memory = InteractionMemory(session_id)
        engine = PersonaBehaviorEngine(session_id, grammar_config=grammar_config)
        runtime = BanterExecutionRuntime(
            session_id, engine, MotionPlanner(), MotionStyler(), BanterExecutionClient(arguments.host, arguments.port)
        )
        runtime.connect()
        latest_behavior = "None"
        event_count = 0
        previous_time = time.perf_counter()
        while True:
            for _ in range(4):
                if runtime.poll() is None:
                    break
            raw_frame, capture_t_ms = stream.read()
            perception = perceiver.process(raw_frame, capture_t_ms)
            task1_events = stabilizer.update(perception)
            gate_update = gate.update(task1_events, stabilizer.active_gestures, capture_t_ms)
            if gate_update.transition.value != "NONE":
                grammar.clear_transient_history()
                engine.clear_transient_history()
                print(f"[Task7] interaction {gate_update.transition.value}; control events={gate_update.control_event_ids}", flush=True)

            if gate.state is InteractionArmState.ARMED:
                for batch in gate_update.forwarded_batches:
                    grammar_update = grammar.update(batch)
                    before = memory.snapshot
                    snapshot = memory.update(grammar_update)
                    if snapshot is not before:
                        update = engine.update(snapshot)
                        if update.behavior is not None:
                            latest_behavior = f"#{update.behavior.behavior_id} {update.behavior.behavior_name}:{update.behavior.variant}"
                        runtime.dispatch(update)
                        event_count += len(batch)
                update = engine.advance_time(capture_t_ms)
                runtime.dispatch(update)  # duplicate updates are rejected by behavior-id provenance.

            now = time.perf_counter(); fps = 1.0 / max(now - previous_time, 1e-9); previous_time = now
            display = cv2.flip(raw_frame, 1) if mirror else raw_frame.copy()
            draw_perception(display, perception, fps, mirror, event_count)
            _draw_status(display, gate.state, runtime, latest_behavior)
            cv2.imshow(window_name, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                return 0
            if key in (ord("m"), ord("M")):
                mirror = not mirror
    except (FileNotFoundError, RuntimeError, ValueError, ConnectionError) as error:
        print(f"Task 7 live demo failed: {error}", flush=True)
        return 1
    finally:
        # Release the camera/UI first.  A MATLAB drain never freezes the
        # preview window or leaves C920 locked while it reaches neutral.
        cleanup_demo_resources(perceiver, stream, window_name)
        if runtime is not None:
            _wait_for_executor_shutdown(runtime)
            runtime.close()


if __name__ == "__main__":
    raise SystemExit(main())
