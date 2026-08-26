"""Deterministic TCP rehearsal: recorded triangle -> live MATLAB executor.

This is not the final camera demo.  It sends the already-derived Stage 3.4
facts with their recorded causal order, so Python/MATLAB transport and the
MATLAB q(t) FIFO can be checked without C920 or MediaPipe.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from typing import Any, Sequence

from .live_protocol import LiveProtocolClient


def _wait_for_reply(client: LiveProtocolClient, expected_kind: str, timeout_s: float) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        reply = client.receive()
        if reply is not None and reply.get("kind") == expected_kind:
            return reply
        time.sleep(0.005)
    raise TimeoutError(f"MATLAB did not send {expected_kind} within {timeout_s:.1f} s.")


def replay_trace_to_live_executor(
    trace_path: str | Path, *, port: int = 51011, speed: float = 1.0
) -> dict[str, int]:
    """Release each fact only when its recorded source/segment time is due."""
    if speed <= 0.0:
        raise ValueError("speed must be positive.")
    trace = json.loads(Path(trace_path).read_text(encoding="utf-8"))
    events = list(trace["source_events"])
    if not events:
        raise ValueError("Trace contains no source events.")
    session_id = "deterministic_replay_tcp"
    first_replay_ms = float(events[0]["replay_t_ms"])
    client = LiveProtocolClient(port=port)
    client.connect(session_id)
    try:
        _wait_for_reply(client, "CONNECTED", 3.0)
        p0 = events[0]
        client.send(session_id, "START_TARGET", {"x_mm": p0["x_mm"], "y_mm": p0["y_mm"]})
        _wait_for_reply(client, "START_READY", 5.0)

        # A source point and a committed segment have different ready times.
        # Sorting this list makes the recorded look-ahead visible again: the
        # MATLAB executor never receives a future segment before it was ready.
        facts: list[tuple[float, int, str, dict[str, Any]]] = []
        for event in events:
            facts.append((float(event["replay_t_ms"]) - first_replay_ms, 0, "SOURCE_EVENT", event))
        for segment in trace["planned_segments"]:
            ready = float(segment["time_contract"]["available_replay_t_ms"]) - first_replay_ms
            facts.append((ready, 1, "PLANNED_SEGMENT", segment))
        for barrier in trace.get("barriers", []):
            facts.append((float(barrier["replay_t_ms"]) - first_replay_ms, 2, "BARRIER", barrier))
        facts.sort(key=lambda item: (item[0], item[1]))

        wall_origin = time.monotonic()
        for due_ms, _priority, kind, payload in facts:
            while time.monotonic() - wall_origin < due_ms / 1_000.0 / speed:
                # Replies are currently only observability; a HOLD is fatal.
                reply = client.receive()
                if reply is not None and reply.get("kind") == "HOLD":
                    raise RuntimeError(f"MATLAB entered HOLD: {reply.get('payload')}")
                time.sleep(0.002)
            client.send(session_id, kind, payload)
        client.send(session_id, "END_OF_STREAM", {"available_t_ms": events[-1]["replay_t_ms"]})
        _wait_for_reply(client, "FINISHED", 30.0)
        return {"source_events": len(events), "planned_segments": len(trace["planned_segments"])}
    finally:
        client.close()


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", help="Stage 3.4 replay trace JSON")
    parser.add_argument("--port", type=int, default=51011)
    parser.add_argument("--speed", type=float, default=1.0, help="Wall replay multiplier; 1 preserves trace timing.")
    args = parser.parse_args(argv)
    try:
        result = replay_trace_to_live_executor(args.trace, port=args.port, speed=args.speed)
        print(f"TCP rehearsal complete: {result['source_events']} source events, {result['planned_segments']} planned segments.")
        return 0
    except (OSError, RuntimeError, TimeoutError, ValueError, KeyError, json.JSONDecodeError) as error:
        print(f"TCP rehearsal failed: {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
