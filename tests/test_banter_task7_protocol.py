"""Task 7 protocol framing and immutable execution contract tests."""

from __future__ import annotations

import socket

from banter.execution_protocol import BANTER_EXECUTION_PROTOCOL_VERSION, BanterExecutionClient, execution_message


def test_execution_message_has_independent_schema_and_strict_core_fields() -> None:
    message = execution_message("task7", 3, "EXECUTE_STYLED_MOTION", {"execution_id": 1})
    assert message == {
        "schema_version": BANTER_EXECUTION_PROTOCOL_VERSION,
        "session_id": "task7",
        "sequence_id": 3,
        "kind": "EXECUTE_STYLED_MOTION",
        "payload": {"execution_id": 1},
    }


def test_client_writes_one_line_hello() -> None:
    listener = socket.socket(); listener.bind(("127.0.0.1", 0)); listener.listen(1)
    port = listener.getsockname()[1]
    client = BanterExecutionClient(port=port)
    client.connect("task7-wire")
    connection, _ = listener.accept()
    received = connection.recv(4096).decode("utf-8")
    connection.close(); listener.close(); client.close()
    assert received.endswith("\n")
    assert BANTER_EXECUTION_PROTOCOL_VERSION in received and '"kind":"HELLO"' in received
