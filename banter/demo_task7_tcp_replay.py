"""Deterministic no-camera Task 7 TCP rehearsal against MATLAB executor."""

from __future__ import annotations

import argparse
import time
from typing import Sequence

from .contracts import GestureEvent, GestureKind, HandKey
from .execution_contracts import ExecutionState
from .execution_protocol import BanterExecutionClient
from .execution_runtime import BanterExecutionRuntime
from .gesture_grammar import GestureGrammarConfig
from .grammar_contracts import GrammarUpdate
from .interaction_memory import InteractionMemory
from .motion_library import MotionPlanner
from .motion_styling import MotionStyler
from .personality_behavior import PersonaBehaviorEngine


def _arguments() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=51012)
    parser.add_argument("--timeout-s", type=float, default=15.0)
    return parser


def _wait(runtime: BanterExecutionRuntime, wanted: ExecutionState, deadline: float) -> None:
    while time.monotonic() < deadline:
        runtime.poll()
        if runtime.state is wanted:
            return
        if runtime.state is ExecutionState.FAULTED:
            raise RuntimeError(runtime.fault_reason)
        time.sleep(0.01)
    raise TimeoutError(f"Timed out waiting for {wanted.value}; state={runtime.state.value}")


def main(argv: Sequence[str] | None = None) -> int:
    args = _arguments().parse_args(argv)
    session = "banter-task7-replay"
    grammar_config = GestureGrammarConfig(chords=(), sequences=())
    memory = InteractionMemory(session)
    engine = PersonaBehaviorEngine(session, grammar_config=grammar_config)
    runtime = BanterExecutionRuntime(
        session, engine, MotionPlanner(), MotionStyler(), BanterExecutionClient(args.host, args.port)
    )
    deadline = time.monotonic() + args.timeout_s
    try:
        runtime.connect()
        _wait(runtime, ExecutionState.READY, deadline)
        token = GestureEvent(1, GestureKind.THUMBS_UP, HandKey.LEFT, 1, 100.0, 500.0, 501.0, 400.0, 0.9, 1)
        update = engine.update(memory.update(GrammarUpdate((token,), ())))
        command = runtime.dispatch(update)
        if command is None:
            raise RuntimeError(f"Could not dispatch replay: {runtime.fault_reason}")
        _wait(runtime, ExecutionState.READY, deadline)
        print(f"PASS: Task 7 TCP replay completed {command.behavior.behavior_name}:{command.behavior.variant}.")
        runtime.request_shutdown()
        _wait(runtime, ExecutionState.CLOSED, deadline)
        return 0
    finally:
        runtime.close()


if __name__ == "__main__":
    raise SystemExit(main())
