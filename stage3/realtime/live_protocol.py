"""Small ordered localhost JSON protocol between Python planning and MATLAB.

One newline is one UTF-8 JSON object.  TCP gives local ordered delivery; the
sequence number makes accidental duplication or reordering an explicit error.
This is intentionally a narrow bridge, not ROS or a generic message bus.
"""
from __future__ import annotations

from dataclasses import asdict
import json
import socket
from typing import Any

from .rolling_cartesian_planner import PlannedCartesianSegment, PlannerBarrier
from .stream_contract import RealtimeTaskEvent

LIVE_PROTOCOL_VERSION = "monoteach_live_v1"


def planned_segment_message(session_id: str, sequence_id: int, segment: PlannedCartesianSegment) -> dict[str, Any]:
    p = segment.profile
    return _message(session_id, sequence_id, "PLANNED_SEGMENT", {
        "segment_index": segment.segment_index,
        "source_start_index": segment.source_start_index,
        "source_end_index": segment.source_end_index,
        "motion_mode": segment.motion_mode.value,
        "stroke_id": segment.stroke_id,
        "start_state_method": segment.start_state_method,
        "end_state_method": segment.end_state_method,
        "finalization_reason": segment.finalization_reason,
        "time_contract": asdict(segment.time_contract),
        "profile": {"start": asdict(p.start), "end": asdict(p.end), "coefficients_x": list(p.coefficients_x), "coefficients_y": list(p.coefficients_y)},
    })


def barrier_message(session_id: str, sequence_id: int, barrier: PlannerBarrier) -> dict[str, Any]:
    return _message(session_id, sequence_id, "BARRIER", {"kind": barrier.kind.value, "source_index": barrier.source_index, "source_t_ms": barrier.source_t_ms, "replay_t_ms": barrier.replay_t_ms, "reason": barrier.reason})


def source_event_message(session_id: str, sequence_id: int, event: RealtimeTaskEvent) -> dict[str, Any]:
    return _message(session_id, sequence_id, "SOURCE_EVENT", {
        "source_index": event.source_index, "source_t_ms": event.source_t_ms, "replay_t_ms": event.replay_t_ms,
        "valid": event.valid, "inside_workspace": event.inside_workspace, "x_mm": event.x_mm, "y_mm": event.y_mm, "z_task_mm": event.z_task_mm,
        "pen_state": event.pen_state, "stroke_id": event.stroke_id, "invalid_reason": event.invalid_reason,
    })


def _message(session_id: str, sequence_id: int, kind: str, payload: dict[str, Any]) -> dict[str, Any]:
    if not session_id or sequence_id < 0: raise ValueError("session_id and non-negative sequence_id are required.")
    return {"schema_version": LIVE_PROTOCOL_VERSION, "session_id": session_id, "sequence_id": sequence_id, "kind": kind, "payload": payload}


class LiveProtocolClient:
    """Synchronous client with a bounded responsibility: send one planning fact."""
    def __init__(self, host: str = "127.0.0.1", port: int = 51011, timeout_s: float = 2.0) -> None:
        self.host, self.port, self.timeout_s = host, int(port), float(timeout_s)
        self._socket: socket.socket | None = None
        self._sequence = 0
        self._receive_buffer = bytearray()
        self._pending_replies: list[dict[str, Any]] = []

    def connect(self, session_id: str) -> None:
        if self._socket is not None: return
        self._socket = socket.create_connection((self.host, self.port), self.timeout_s)
        self.send(session_id, "HELLO", {"role": "python_live_planner"})

    def send(self, session_id: str, kind: str, payload: dict[str, Any]) -> int:
        if self._socket is None: raise RuntimeError("Protocol client is not connected.")
        sequence = self._sequence; self._sequence += 1
        wire = json.dumps(_message(session_id, sequence, kind, payload), separators=(",", ":")) + "\n"
        self._socket.sendall(wire.encode("utf-8"))
        return sequence

    def receive(self) -> dict[str, Any] | None:
        """Read one optional reply without blocking a camera frame."""
        if self._socket is None: return None
        if self._pending_replies: return self._pending_replies.pop(0)
        self._socket.setblocking(False)
        try:
            data = self._socket.recv(65_536)
        except BlockingIOError:
            return None
        finally:
            self._socket.setblocking(True)
        if not data:
            # An empty recv is TCP's explicit EOF, not "no reply yet".  The
            # distinction matters: treating it as silence used to turn a
            # MATLAB executor timeout/crash into a later mysterious WinError
            # on the next camera-originated send.
            raise ConnectionError("MATLAB TCP executor closed the connection unexpectedly.")
        self._receive_buffer.extend(data)
        while b"\n" in self._receive_buffer:
            line, _separator, rest = self._receive_buffer.partition(b"\n")
            self._receive_buffer = bytearray(rest)
            if line.strip(): self._pending_replies.append(json.loads(line.decode("utf-8")))
        return self._pending_replies.pop(0) if self._pending_replies else None

    def close(self) -> None:
        if self._socket is not None:
            try: self._socket.close()
            finally: self._socket = None
