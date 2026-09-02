"""Narrow ordered localhost NDJSON protocol for the Banter simulator."""

from __future__ import annotations

import json
import socket
from typing import Any


BANTER_EXECUTION_PROTOCOL_VERSION = "banter_execution_v1"


def execution_message(session_id: str, sequence_id: int, kind: str, payload: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(session_id, str) or not session_id:
        raise ValueError("session_id is required.")
    if not isinstance(sequence_id, int) or sequence_id < 0:
        raise ValueError("sequence_id must be non-negative.")
    if not isinstance(kind, str) or not kind:
        raise ValueError("kind is required.")
    return {
        "schema_version": BANTER_EXECUTION_PROTOCOL_VERSION,
        "session_id": session_id,
        "sequence_id": sequence_id,
        "kind": kind,
        "payload": payload,
    }


class BanterExecutionClient:
    """Small synchronous client; receive is non-blocking for camera frames."""

    def __init__(self, host: str = "127.0.0.1", port: int = 51012, timeout_s: float = 2.0) -> None:
        self.host, self.port, self.timeout_s = host, int(port), float(timeout_s)
        self._socket: socket.socket | None = None
        self._sequence = 0
        self._receive_buffer = bytearray()
        self._pending: list[dict[str, Any]] = []

    def connect(self, session_id: str) -> None:
        if self._socket is not None:
            return
        self._socket = socket.create_connection((self.host, self.port), self.timeout_s)
        self.send(session_id, "HELLO", {"role": "python_banter_runtime"})

    def send(self, session_id: str, kind: str, payload: dict[str, Any]) -> int:
        if self._socket is None:
            raise RuntimeError("Banter execution client is not connected.")
        sequence = self._sequence
        self._sequence += 1
        wire = json.dumps(execution_message(session_id, sequence, kind, payload), separators=(",", ":")) + "\n"
        self._socket.sendall(wire.encode("utf-8"))
        return sequence

    def receive(self) -> dict[str, Any] | None:
        if self._socket is None:
            return None
        if self._pending:
            return self._pending.pop(0)
        self._socket.setblocking(False)
        try:
            data = self._socket.recv(65_536)
        except BlockingIOError:
            return None
        finally:
            self._socket.setblocking(True)
        if not data:
            raise ConnectionError("MATLAB Banter executor closed the connection unexpectedly.")
        self._receive_buffer.extend(data)
        while b"\n" in self._receive_buffer:
            line, _separator, rest = self._receive_buffer.partition(b"\n")
            self._receive_buffer = bytearray(rest)
            if line.strip():
                decoded = json.loads(line.decode("utf-8"))
                if not isinstance(decoded, dict):
                    raise ValueError("Execution reply must be a JSON object.")
                self._pending.append(decoded)
        return self._pending.pop(0) if self._pending else None

    def close(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close()
            finally:
                self._socket = None
