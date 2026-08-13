"""JSON persistence for Stage 2.3 image-to-workspace calibrations."""

from __future__ import annotations

import json
from pathlib import Path
import re
from typing import Any

import numpy as np

from .workspace_geometry import (
    WORKSPACE_COORDINATE_FRAME,
    WorkspaceCalibration,
    WorkspaceDefinition,
)


DEFAULT_WORKSPACE_CALIBRATION_DIRECTORY = Path("data") / "calibrations"
IMAGE_POINTS_COORDINATE_SPACE = "undistorted_pixel"
SUPPORTED_SCHEMA_VERSIONS = {"1.0"}


def workspace_calibration_output_path(
    calibration: WorkspaceCalibration,
    output_directory: str | Path = DEFAULT_WORKSPACE_CALIBRATION_DIRECTORY,
) -> Path:
    """Return the deterministic JSON destination for one calibration ID."""
    if not isinstance(calibration, WorkspaceCalibration):
        raise TypeError("calibration must be a WorkspaceCalibration.")
    safe_id = re.sub(
        r"[^A-Za-z0-9._-]+",
        "_",
        calibration.calibration_id,
    ).strip("._")
    if not safe_id:
        safe_id = "workspace_calibration"
    return Path(output_directory) / f"{safe_id}.json"


def save_workspace_calibration_json(
    calibration: WorkspaceCalibration,
    output_directory: str | Path = DEFAULT_WORKSPACE_CALIBRATION_DIRECTORY,
) -> Path:
    """Save one calibration whose image points are undistorted pixels."""
    output_path = workspace_calibration_output_path(calibration, output_directory)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema_version": calibration.schema_version,
        "calibration_id": calibration.calibration_id,
        "frame_width": calibration.frame_width,
        "frame_height": calibration.frame_height,
        "image_points_coordinate_space": IMAGE_POINTS_COORDINATE_SPACE,
        "image_points": calibration.image_points.tolist(),
        "workspace_points_mm": calibration.workspace_points_mm.tolist(),
        "H_image_to_workspace": calibration.H_image_to_workspace.tolist(),
        "camera_calibration_reference": calibration.camera_calibration_reference,
        "workspace": {
            "width_mm": calibration.workspace_definition.width_mm,
            "height_mm": calibration.workspace_definition.height_mm,
            "coordinate_frame": calibration.workspace_definition.coordinate_frame,
        },
    }
    output_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return output_path


def load_workspace_calibration_json(path: str | Path) -> WorkspaceCalibration:
    """Load and strictly validate a workspace-calibration JSON artifact."""
    input_path = Path(path)
    try:
        payload = json.loads(input_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError) as error:
        raise ValueError(
            f"Cannot read workspace calibration JSON {input_path}: {error}"
        ) from error
    except json.JSONDecodeError as error:
        raise ValueError(
            f"Malformed workspace calibration JSON: {error.msg}."
        ) from error

    root = _require_mapping(payload, "root")
    _require_fields(
        root,
        {
            "schema_version",
            "calibration_id",
            "frame_width",
            "frame_height",
            "image_points_coordinate_space",
            "image_points",
            "workspace_points_mm",
            "H_image_to_workspace",
            "camera_calibration_reference",
            "workspace",
        },
        "root",
    )

    schema_version = _require_string(root["schema_version"], "schema_version")
    if schema_version not in SUPPORTED_SCHEMA_VERSIONS:
        raise ValueError(f"Unsupported schema_version: {schema_version!r}.")
    coordinate_space = _require_string(
        root["image_points_coordinate_space"],
        "image_points_coordinate_space",
    )
    if coordinate_space != IMAGE_POINTS_COORDINATE_SPACE:
        raise ValueError(
            "image_points_coordinate_space must be "
            f"{IMAGE_POINTS_COORDINATE_SPACE!r}; got {coordinate_space!r}."
        )

    workspace_data = _require_mapping(root["workspace"], "workspace")
    _require_fields(
        workspace_data,
        {"width_mm", "height_mm", "coordinate_frame"},
        "workspace",
    )
    coordinate_frame = _require_string(
        workspace_data["coordinate_frame"],
        "workspace.coordinate_frame",
    )
    if coordinate_frame != WORKSPACE_COORDINATE_FRAME:
        raise ValueError(
            f"workspace.coordinate_frame must be {WORKSPACE_COORDINATE_FRAME!r}."
        )

    try:
        return WorkspaceCalibration(
            schema_version=schema_version,
            calibration_id=_require_string(
                root["calibration_id"],
                "calibration_id",
            ),
            frame_width=_require_integer(root["frame_width"], "frame_width"),
            frame_height=_require_integer(root["frame_height"], "frame_height"),
            workspace_definition=WorkspaceDefinition(
                width_mm=_require_number(
                    workspace_data["width_mm"],
                    "workspace.width_mm",
                ),
                height_mm=_require_number(
                    workspace_data["height_mm"],
                    "workspace.height_mm",
                ),
                coordinate_frame=coordinate_frame,
            ),
            image_points=_require_numeric_array(root["image_points"], "image_points"),
            workspace_points_mm=_require_numeric_array(
                root["workspace_points_mm"],
                "workspace_points_mm",
            ),
            H_image_to_workspace=_require_numeric_array(
                root["H_image_to_workspace"],
                "H_image_to_workspace",
            ),
            camera_calibration_reference=_require_string(
                root["camera_calibration_reference"],
                "camera_calibration_reference",
            ),
        )
    except (TypeError, ValueError) as error:
        raise ValueError(f"Invalid workspace calibration artifact: {error}") from error


def _require_mapping(value: Any, context: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{context} must be a JSON object.")
    return value


def _require_fields(data: dict[str, Any], fields: set[str], context: str) -> None:
    missing = sorted(fields - data.keys())
    if missing:
        raise ValueError(f"{context} is missing required fields: {', '.join(missing)}.")


def _require_string(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{context} must be a non-empty string.")
    return value


def _require_integer(value: Any, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{context} must be an integer.")
    return value


def _require_number(value: Any, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{context} must be a finite number.")
    numeric_value = float(value)
    if not np.isfinite(numeric_value):
        raise ValueError(f"{context} must be a finite number.")
    return numeric_value


def _require_numeric_array(value: Any, context: str) -> np.ndarray:
    if not isinstance(value, list):
        raise ValueError(f"{context} must be a JSON array.")
    try:
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{context} must contain only numeric values.") from error
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{context} must contain only finite values.")
    return array
