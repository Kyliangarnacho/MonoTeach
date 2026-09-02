"""Task 7 local busy-before-send and completion-authority tests."""

from __future__ import annotations

from banter.contracts import GestureEvent, GestureKind, HandKey
from banter.execution_contracts import ExecutionState
from banter.execution_runtime import BanterExecutionRuntime
from banter.gesture_grammar import GestureGrammarConfig
from banter.grammar_contracts import GrammarUpdate
from banter.interaction_memory import InteractionMemory
from banter.motion_library import MotionPlanner
from banter.motion_styling import MotionStyler
from banter.personality_behavior import PersonaBehaviorEngine
from banter.personality_contracts import BehaviorAdmission


class _FakeClient:
    def __init__(self) -> None:
        self.sent: list[tuple[str, str, dict]] = []
        self.replies: list[dict] = []

    def connect(self, session_id: str) -> None:
        self.sent.append((session_id, "HELLO", {"role": "python_banter_runtime"}))

    def send(self, session_id: str, kind: str, payload: dict) -> int:
        self.sent.append((session_id, kind, payload))
        return len(self.sent) - 1

    def receive(self):
        return self.replies.pop(0) if self.replies else None

    def close(self) -> None:
        pass


def _token(event_id: int, gesture: GestureKind, t_ms: float) -> GestureEvent:
    return GestureEvent(event_id, gesture, HandKey.LEFT, event_id, t_ms - 50.0, t_ms, t_ms + 1.0, 50.0, 0.9, 1)


def _reply(session: str, kind: str, payload: dict) -> dict:
    return {"schema_version": "banter_execution_v1", "session_id": session, "sequence_id": 1, "kind": kind, "payload": payload}


def test_busy_is_acquired_before_send_and_only_matching_completion_releases() -> None:
    grammar_config = GestureGrammarConfig(chords=(), sequences=())
    memory = InteractionMemory("task7", clock_ms=lambda: 10_000.0)
    engine = PersonaBehaviorEngine("task7", grammar_config=grammar_config, clock_ms=lambda: 20_000.0)
    client = _FakeClient()
    runtime = BanterExecutionRuntime(
        "task7", engine, MotionPlanner(clock_ms=lambda: 30_000.0), MotionStyler(clock_ms=lambda: 40_000.0), client  # type: ignore[arg-type]
    )
    runtime.connect()
    client.replies.append(_reply("task7", "CONNECTED", {}))
    runtime.poll()
    assert runtime.state is ExecutionState.READY

    first = engine.update(memory.update(GrammarUpdate((_token(1, GestureKind.POINT_ONE, 100.0),), ())))
    command = runtime.dispatch(first)
    assert command is not None
    assert runtime.state is ExecutionState.DISPATCHED
    assert engine.external_active_behavior_id == first.behavior.behavior_id  # type: ignore[union-attr]
    assert client.sent[-1][1] == "EXECUTE_STYLED_MOTION"

    second = engine.update(memory.update(GrammarUpdate((_token(2, GestureKind.MIDDLE_FINGER, 200.0),), ())))
    assert second.admission is BehaviorAdmission.IGNORED_BUSY
    assert runtime.dispatch(second) is None

    payload = command.to_payload()
    client.replies.extend((_reply("task7", "MOTION_ACCEPTED", payload), _reply("task7", "MOTION_STARTED", payload)))
    runtime.poll(); runtime.poll()
    assert runtime.state is ExecutionState.EXECUTING

    wrong = dict(payload); wrong["behavior_id"] = 99
    client.replies.append(_reply("task7", "MOTION_COMPLETED", wrong))
    runtime.poll()
    assert runtime.state is ExecutionState.FAULTED
    assert engine.external_active_behavior_id == command.behavior.behavior_id


def test_matching_completion_is_the_only_normal_unlock_path() -> None:
    grammar_config = GestureGrammarConfig(chords=(), sequences=())
    memory = InteractionMemory("complete", clock_ms=lambda: 10_000.0)
    engine = PersonaBehaviorEngine("complete", grammar_config=grammar_config, clock_ms=lambda: 20_000.0)
    client = _FakeClient()
    runtime = BanterExecutionRuntime("complete", engine, MotionPlanner(clock_ms=lambda: 30_000.0), MotionStyler(clock_ms=lambda: 40_000.0), client)  # type: ignore[arg-type]
    runtime.connect(); client.replies.append(_reply("complete", "CONNECTED", {})); runtime.poll()
    update = engine.update(memory.update(GrammarUpdate((_token(1, GestureKind.THUMBS_UP, 100.0),), ())))
    command = runtime.dispatch(update); assert command is not None
    payload = command.to_payload()
    client.replies.extend((_reply("complete", "MOTION_ACCEPTED", payload), _reply("complete", "MOTION_STARTED", payload), _reply("complete", "MOTION_COMPLETED", payload)))
    runtime.poll(); runtime.poll(); runtime.poll()
    assert runtime.state is ExecutionState.READY
    assert engine.external_active_behavior_id is None


def test_shutdown_keeps_completion_authoritative_while_camera_side_is_closed() -> None:
    grammar_config = GestureGrammarConfig(chords=(), sequences=())
    memory = InteractionMemory("shutdown", clock_ms=lambda: 10_000.0)
    engine = PersonaBehaviorEngine("shutdown", grammar_config=grammar_config, clock_ms=lambda: 20_000.0)
    client = _FakeClient()
    runtime = BanterExecutionRuntime("shutdown", engine, MotionPlanner(clock_ms=lambda: 30_000.0), MotionStyler(clock_ms=lambda: 40_000.0), client)  # type: ignore[arg-type]
    runtime.connect(); client.replies.append(_reply("shutdown", "CONNECTED", {})); runtime.poll()
    update = engine.update(memory.update(GrammarUpdate((_token(1, GestureKind.FIST, 100.0),), ())))
    command = runtime.dispatch(update); assert command is not None
    payload = command.to_payload()
    client.replies.extend((_reply("shutdown", "MOTION_ACCEPTED", payload), _reply("shutdown", "MOTION_STARTED", payload)))
    runtime.poll(); runtime.poll()
    runtime.request_shutdown()
    assert runtime.state is ExecutionState.SHUTTING_DOWN
    client.replies.extend((_reply("shutdown", "MOTION_COMPLETED", payload), _reply("shutdown", "SESSION_FINISHED", {})))
    runtime.poll(); runtime.poll()
    assert runtime.state is ExecutionState.CLOSED
    assert engine.external_active_behavior_id is None
