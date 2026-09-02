"""Task 7 single-flight bridge from BehaviorEvent to executor completion."""

from __future__ import annotations

from typing import Any

from .execution_contracts import ExecutionCommand, ExecutionState
from .execution_protocol import BANTER_EXECUTION_PROTOCOL_VERSION, BanterExecutionClient
from .motion_library import MotionPlanner
from .motion_styling import MotionStyler
from .personality_behavior import PersonaBehaviorEngine
from .personality_contracts import Task4Update


class BanterExecutionRuntime:
    """Own only dispatch/ack/completion state; it never queues or preempts."""

    def __init__(
        self,
        session_id: str,
        engine: PersonaBehaviorEngine,
        planner: MotionPlanner,
        styler: MotionStyler,
        client: BanterExecutionClient,
    ) -> None:
        if not session_id:
            raise ValueError("session_id is required.")
        self.session_id = session_id
        self.engine, self.planner, self.styler, self.client = engine, planner, styler, client
        self.state = ExecutionState.DISCONNECTED
        self.fault_reason: str | None = None
        self._next_execution_id = 1
        self._active: ExecutionCommand | None = None
        self._seen_behavior_ids: set[int] = set()
        self._shutdown_requested = False

    @property
    def active_command(self) -> ExecutionCommand | None:
        return self._active

    def connect(self) -> None:
        if self.state is not ExecutionState.DISCONNECTED:
            raise RuntimeError("Execution runtime may only connect from DISCONNECTED.")
        self.client.connect(self.session_id)

    def dispatch(self, update: Task4Update) -> ExecutionCommand | None:
        """Acquire busy locally before plan construction, serialization, or send."""
        behavior = update.behavior
        if behavior is None or behavior.behavior_id in self._seen_behavior_ids:
            return None
        self._seen_behavior_ids.add(behavior.behavior_id)
        if self.state is not ExecutionState.READY:
            self._fault("BEHAVIOR_WHILE_EXECUTOR_NOT_READY")
            return None
        try:
            # No I/O occurs between Task 4 admission and this state change.
            self.engine.acquire_execution_lock(behavior.behavior_id)
            self.state = ExecutionState.DISPATCHED
            plan = self.planner.plan(behavior)
            styled = self.styler.style(plan)
            command = ExecutionCommand(self._next_execution_id, behavior, styled)
            self._next_execution_id += 1
            self._active = command
            self.client.send(self.session_id, "EXECUTE_STYLED_MOTION", command.to_payload())
            return command
        except Exception as error:
            self._fault(f"DISPATCH_FAILED:{type(error).__name__}:{error}")
            return None

    def poll(self) -> dict[str, Any] | None:
        """Process at most one reply without blocking the camera loop."""
        try:
            reply = self.client.receive()
        except Exception as error:
            self._fault(f"TCP_RECEIVE_FAILED:{type(error).__name__}:{error}")
            return None
        if reply is None:
            return None
        try:
            self._validate_envelope(reply)
            kind = reply["kind"]
            if kind == "CONNECTED":
                if self.state is not ExecutionState.DISCONNECTED:
                    raise ValueError("CONNECTED is only valid from DISCONNECTED.")
                self.state = ExecutionState.READY
            elif kind == "MOTION_ACCEPTED":
                self._require_active(reply, ExecutionState.DISPATCHED, ExecutionState.SHUTTING_DOWN)
            elif kind == "MOTION_STARTED":
                self._require_active(reply, ExecutionState.DISPATCHED, ExecutionState.SHUTTING_DOWN)
                if not self._shutdown_requested:
                    self.state = ExecutionState.EXECUTING
            elif kind == "MOTION_COMPLETED":
                command = self._require_active(reply, ExecutionState.EXECUTING, ExecutionState.SHUTTING_DOWN)
                self.engine.release_execution_lock(command.behavior.behavior_id)
                self._active = None
                self.state = ExecutionState.SHUTTING_DOWN if self._shutdown_requested else ExecutionState.READY
            elif kind in {"MOTION_REJECTED", "MOTION_BUSY", "PROTOCOL_ERROR"}:
                self._require_active(reply, ExecutionState.DISPATCHED, ExecutionState.EXECUTING)
                self._fault(kind)
            elif kind == "SESSION_FINISHED":
                if self.state is not ExecutionState.SHUTTING_DOWN:
                    raise ValueError("SESSION_FINISHED without shutdown request.")
                self.state = ExecutionState.CLOSED
            elif kind == "RENDERING_DISABLED":
                # Drawing is explicitly non-authoritative; the MATLAB q(t)
                # clock and completion lifecycle remain valid without it.
                pass
            else:
                raise ValueError(f"Unsupported executor reply kind {kind!r}.")
        except Exception as error:
            self._fault(f"REPLY_INVALID:{type(error).__name__}:{error}")
        return reply

    def request_shutdown(self) -> None:
        if self.state in {ExecutionState.CLOSED, ExecutionState.DISCONNECTED}:
            return
        try:
            self.client.send(self.session_id, "END_SESSION", {})
            self._shutdown_requested = True
            self.state = ExecutionState.SHUTTING_DOWN
        except Exception as error:
            self._fault(f"SHUTDOWN_SEND_FAILED:{type(error).__name__}:{error}")

    def close(self) -> None:
        self.client.close()

    def _validate_envelope(self, reply: dict[str, Any]) -> None:
        if reply.get("schema_version") != BANTER_EXECUTION_PROTOCOL_VERSION:
            raise ValueError("Unexpected executor schema_version.")
        if reply.get("session_id") != self.session_id:
            raise ValueError("Executor reply has another session_id.")
        if not isinstance(reply.get("payload"), dict) or not isinstance(reply.get("kind"), str):
            raise ValueError("Malformed executor reply.")

    def _require_active(self, reply: dict[str, Any], *allowed: ExecutionState) -> ExecutionCommand:
        if self.state not in allowed or self._active is None:
            raise ValueError("Executor reply is invalid in the current runtime state.")
        payload = reply["payload"]
        expected = self._active.to_payload()
        for key in ("execution_id", "behavior_id", "plan_id", "styled_plan_id", "behavior_name", "variant", "macro_name"):
            if payload.get(key) != expected[key]:
                raise ValueError(f"Executor reply provenance mismatch for {key}.")
        return self._active

    def _fault(self, reason: str) -> None:
        self.state = ExecutionState.FAULTED
        self.fault_reason = reason
