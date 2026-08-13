"""Convert one saved Stage 2.2 trajectory into a workspace-mm JSON artifact."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

from .camera_calibration import load_camera_calibration
from .trajectory_io import load_trajectory_json
from .workspace_calibration_io import load_workspace_calibration_json
from .workspace_trajectory import trajectory_to_workspace
from .workspace_trajectory_io import (
    DEFAULT_WORKSPACE_TRAJECTORY_DIRECTORY,
    save_workspace_trajectory_json,
)


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", required=True, help="Stage 2.2 trajectory JSON")
    parser.add_argument("--camera-calibration", required=True)
    parser.add_argument("--workspace-calibration", required=True)
    parser.add_argument(
        "--output-directory",
        default=str(DEFAULT_WORKSPACE_TRAJECTORY_DIRECTORY),
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Load, derive, and save without embedding conversion logic in the CLI."""
    arguments = _argument_parser().parse_args(argv)
    try:
        loaded_trajectory = load_trajectory_json(arguments.trajectory)
        camera_calibration = load_camera_calibration(arguments.camera_calibration)
        workspace_calibration = load_workspace_calibration_json(
            arguments.workspace_calibration
        )
        workspace_trajectory = trajectory_to_workspace(
            loaded_trajectory.trajectory,
            camera_calibration,
            workspace_calibration,
            quality_config=loaded_trajectory.processing.quality_gate,
            ema_config=loaded_trajectory.processing.ema,
        )
        output_path = save_workspace_trajectory_json(
            workspace_trajectory,
            Path(arguments.output_directory),
        )
    except (OSError, TypeError, ValueError) as error:
        print(f"Workspace trajectory conversion failed: {error}")
        return 1

    print(f"Saved workspace trajectory: {output_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
