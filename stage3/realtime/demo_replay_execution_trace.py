"""Create the finite ReplaySource trace consumed by the MATLAB Task 5/6 demo."""

from __future__ import annotations

from pathlib import Path

from stage2.workspace_trajectory_io import load_workspace_trajectory_json

from .replay_execution_trace import (
    ReplayExecutionTrace,
    build_replay_execution_trace,
    write_replay_execution_trace_json,
)
from .startup_sync import StartupReplaySchedule


def default_real_triangle_path() -> Path:
    """Return the repository's one recorded C920 workspace trajectory."""
    root = Path(__file__).resolve().parents[2]
    candidates = sorted((root / "data" / "workspace_trajectories").glob("*_workspace_2d_*.json"))
    if len(candidates) != 1:
        raise ValueError("Expected exactly one real WorkspaceTrajectory; pass an explicit path.")
    return candidates[0]


def run_demo(
    input_path: str | Path | None = None,
    output_path: str | Path | None = None,
    *,
    playback_speed: float = 1.0,
    startup_schedule: StartupReplaySchedule | None = StartupReplaySchedule(),
) -> ReplayExecutionTrace:
    """Build a trace from the recorded triangle and optionally save it as JSON."""
    trajectory_path = default_real_triangle_path() if input_path is None else Path(input_path)
    trace = build_replay_execution_trace(
        load_workspace_trajectory_json(trajectory_path),
        playback_speed=playback_speed,
        startup_schedule=startup_schedule,
    )
    if output_path is not None:
        write_replay_execution_trace_json(trace, output_path)
    return trace


if __name__ == "__main__":
    artifact = Path("outputs") / "stage3_4" / "real_triangle_replay_trace.json"
    trace = run_demo(output_path=artifact)
    print(
        f"wrote {artifact} | events={len(trace.source_events)} "
        f"segments={len(trace.planned_segments)} barriers={len(trace.barriers)}"
    )
