"""Strict JSON persistence for immutable workspace-millimetre trajectories."""

from __future__ import annotations

from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import re
from typing import Any

from .workspace_trajectory import (
    WorkspaceTrajectory2D,
    WorkspaceTrajectoryMetadata,
    WorkspaceTrajectorySample,
)


DEFAULT_WORKSPACE_TRAJECTORY_DIRECTORY = Path("data") / "workspace_trajectories"
SUPPORTED_SCHEMA_VERSIONS = {"1.0"}


def save_workspace_trajectory_json(
    trajectory: WorkspaceTrajectory2D,
    output_directory: str | Path = DEFAULT_WORKSPACE_TRAJECTORY_DIRECTORY,
) -> Path:
    """Save one derived workspace trajectory without changing its samples."""
    if not isinstance(trajectory, WorkspaceTrajectory2D):
        raise TypeError("trajectory must be a WorkspaceTrajectory2D.")
    directory = Path(output_directory)
    directory.mkdir(parents=True, exist_ok=True)
    output_path = _new_output_path(directory, trajectory.metadata)
    payload = {
        "schema_version": "1.0",
        "metadata": asdict(trajectory.metadata),
        "samples": [asdict(sample) for sample in trajectory.samples],
    }
    output_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return output_path


def load_workspace_trajectory_json(path: str | Path) -> WorkspaceTrajectory2D:
    """Load a strict workspace trajectory JSON artifact without inference or defaults."""
    input_path = Path(path)
    try:
        payload = json.loads(input_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError) as error:
        raise ValueError(f"Cannot read workspace trajectory JSON {input_path}: {error}") from error
    except json.JSONDecodeError as error:
        raise ValueError(f"Malformed workspace trajectory JSON: {error.msg}.") from error

    root = _require_mapping(payload, "root")
    _require_fields(root, {"schema_version", "metadata", "samples"}, "root")
    schema_version = _require_string(root["schema_version"], "schema_version")
    if schema_version not in SUPPORTED_SCHEMA_VERSIONS:
        raise ValueError(f"Unsupported schema_version: {schema_version!r}.")

    metadata_data = _require_mapping(root["metadata"], "metadata")
    _require_fields(
        metadata_data,
        {
            "source_trajectory_id",
            "workspace_calibration_id",
            "coordinate_frame",
            "width_mm",
            "height_mm",
        },
        "metadata",
    )
    metadata = WorkspaceTrajectoryMetadata(
        source_trajectory_id=_require_string(
            metadata_data["source_trajectory_id"],
            "metadata.source_trajectory_id",
        ),
        workspace_calibration_id=_require_string(
            metadata_data["workspace_calibration_id"],
            "metadata.workspace_calibration_id",
        ),
        coordinate_frame=_require_string(
            metadata_data["coordinate_frame"],
            "metadata.coordinate_frame",
        ),
        width_mm=_require_number(metadata_data["width_mm"], "metadata.width_mm"),
        height_mm=_require_number(metadata_data["height_mm"], "metadata.height_mm"),
    )

    samples_data = root["samples"]
    if not isinstance(samples_data, list):
        raise ValueError("samples must be a JSON array.")
    samples = tuple(_load_sample(value, index) for index, value in enumerate(samples_data))
    return WorkspaceTrajectory2D(metadata=metadata, samples=samples)


def _new_output_path(directory: Path, metadata: WorkspaceTrajectoryMetadata) -> Path:
    source = _safe_filename_component(metadata.source_trajectory_id, "trajectory")
    calibration = _safe_filename_component(
        metadata.workspace_calibration_id,
        "workspace_calibration",
    )
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    candidate = directory / f"{source}_{calibration}_{timestamp}.json"
    suffix = 1
    while candidate.exists():
        candidate = directory / f"{source}_{calibration}_{timestamp}_{suffix}.json"
        suffix += 1
    return candidate


def _safe_filename_component(value: str, fallback: str) -> str:
    safe_value = re.sub(r"[^A-Za-z0-9._-]+", "_", value).strip("._")
    return safe_value or fallback


def _load_sample(value: Any, index: int) -> WorkspaceTrajectorySample:
    context = f"samples[{index}]"
    data = _require_mapping(value, context)
    _require_fields(
        data,
        {"t_ms", "valid", "x_mm", "y_mm", "inside_workspace", "invalid_reason"},
        context,
    )
    valid = data["valid"]
    inside = data["inside_workspace"]
    if not isinstance(valid, bool):
        raise ValueError(f"{context}.valid must be a boolean.")
    if not isinstance(inside, bool):
        raise ValueError(f"{context}.inside_workspace must be a boolean.")
    return WorkspaceTrajectorySample(
        t_ms=_require_number(data["t_ms"], f"{context}.t_ms"),
        valid=valid,
        x_mm=_optional_number(data["x_mm"], f"{context}.x_mm"),
        y_mm=_optional_number(data["y_mm"], f"{context}.y_mm"),
        inside_workspace=inside,
        invalid_reason=_optional_string(
            data["invalid_reason"],
            f"{context}.invalid_reason",
        ),
    )


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


def _optional_string(value: Any, context: str) -> str | None:
    if value is None:
        return None
    return _require_string(value, context)


def _require_number(value: Any, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{context} must be a number.")
    return float(value)


def _optional_number(value: Any, context: str) -> float | None:
    if value is None:
        return None
    return _require_number(value, context)
