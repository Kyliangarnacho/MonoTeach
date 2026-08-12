"""JSON persistence for canonical Stage 2.2 trajectories."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
import re
from typing import Any

from .trajectory import Trajectory2D, TrajectoryMetadata, TrajectorySample
from .trajectory_filter import EMAConfig
from .trajectory_quality import QualityGateConfig


DEFAULT_TRAJECTORY_DIRECTORY = Path("data") / "trajectories"
SUPPORTED_SCHEMA_VERSIONS = {"1.0"}


@dataclass(frozen=True)
class TrajectoryProcessingConfig:
    """Processing parameters saved alongside canonical raw samples."""

    quality_gate: QualityGateConfig
    ema: EMAConfig


@dataclass(frozen=True)
class LoadedTrajectory:
    """A loaded trajectory and the configuration used for its derivatives."""

    trajectory: Trajectory2D
    processing: TrajectoryProcessingConfig


def save_trajectory_json(
    trajectory: Trajectory2D,
    quality_config: QualityGateConfig = QualityGateConfig(),
    ema_config: EMAConfig = EMAConfig(),
    output_directory: str | Path = DEFAULT_TRAJECTORY_DIRECTORY,
) -> Path:
    """Save metadata, canonical raw samples, and processing configuration."""
    directory = Path(output_directory)
    directory.mkdir(parents=True, exist_ok=True)
    output_path = _new_output_path(directory, trajectory.metadata.trajectory_id)

    payload = {
        "schema_version": trajectory.metadata.schema_version,
        "trajectory_id": trajectory.metadata.trajectory_id,
        "metadata": asdict(trajectory.metadata),
        "raw_samples": [asdict(sample) for sample in trajectory.raw_samples],
        "processing": {
            "quality_gate": {
                "max_normalized_speed": quality_config.max_normalized_speed,
            },
            "ema": {"alpha": ema_config.alpha},
        },
    }
    output_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return output_path


def load_trajectory_json(path: str | Path) -> LoadedTrajectory:
    """Load and validate one trajectory JSON without deriving processed samples."""
    input_path = Path(path)
    try:
        payload = json.loads(input_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ValueError(f"Malformed trajectory JSON: {error.msg}.") from error

    root = _require_mapping(payload, "root")
    _require_fields(
        root,
        {"schema_version", "trajectory_id", "metadata", "raw_samples", "processing"},
        "root",
    )

    schema_version = _require_string(root["schema_version"], "schema_version")
    if schema_version not in SUPPORTED_SCHEMA_VERSIONS:
        raise ValueError(f"Unsupported schema_version: {schema_version!r}.")
    trajectory_id = _require_string(root["trajectory_id"], "trajectory_id")

    metadata_data = _require_mapping(root["metadata"], "metadata")
    metadata_fields = {
        "schema_version",
        "trajectory_id",
        "frame_width",
        "frame_height",
        "camera_index",
        "recording_start_timestamp_ms",
        "source",
        "coordinate_space",
    }
    _require_fields(metadata_data, metadata_fields, "metadata")
    metadata = TrajectoryMetadata(
        schema_version=_require_string(
            metadata_data["schema_version"], "metadata.schema_version"
        ),
        trajectory_id=_require_string(
            metadata_data["trajectory_id"], "metadata.trajectory_id"
        ),
        frame_width=_require_integer(
            metadata_data["frame_width"], "metadata.frame_width"
        ),
        frame_height=_require_integer(
            metadata_data["frame_height"], "metadata.frame_height"
        ),
        camera_index=_require_integer(
            metadata_data["camera_index"], "metadata.camera_index"
        ),
        recording_start_timestamp_ms=_require_number(
            metadata_data["recording_start_timestamp_ms"],
            "metadata.recording_start_timestamp_ms",
        ),
        source=_require_string(metadata_data["source"], "metadata.source"),
        coordinate_space=_require_string(
            metadata_data["coordinate_space"], "metadata.coordinate_space"
        ),
    )
    if metadata.schema_version != schema_version:
        raise ValueError("Root and metadata schema_version values must match.")
    if metadata.trajectory_id != trajectory_id:
        raise ValueError("Root and metadata trajectory_id values must match.")

    raw_sample_data = root["raw_samples"]
    if not isinstance(raw_sample_data, list):
        raise ValueError("raw_samples must be a JSON array.")
    raw_samples = tuple(
        _load_sample(sample_data, index)
        for index, sample_data in enumerate(raw_sample_data)
    )

    processing_data = _require_mapping(root["processing"], "processing")
    _require_fields(processing_data, {"quality_gate", "ema"}, "processing")
    quality_data = _require_mapping(
        processing_data["quality_gate"], "processing.quality_gate"
    )
    ema_data = _require_mapping(processing_data["ema"], "processing.ema")
    _require_fields(
        quality_data,
        {"max_normalized_speed"},
        "processing.quality_gate",
    )
    _require_fields(ema_data, {"alpha"}, "processing.ema")
    processing = TrajectoryProcessingConfig(
        quality_gate=QualityGateConfig(
            max_normalized_speed=_require_number(
                quality_data["max_normalized_speed"],
                "processing.quality_gate.max_normalized_speed",
            )
        ),
        ema=EMAConfig(
            alpha=_require_number(ema_data["alpha"], "processing.ema.alpha")
        ),
    )

    return LoadedTrajectory(
        trajectory=Trajectory2D(metadata=metadata, raw_samples=raw_samples),
        processing=processing,
    )


def _new_output_path(directory: Path, trajectory_id: str) -> Path:
    safe_id = re.sub(r"[^A-Za-z0-9._-]+", "_", trajectory_id).strip("._")
    if not safe_id:
        safe_id = "trajectory"
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    candidate = directory / f"{safe_id}_{timestamp}.json"
    suffix = 1
    while candidate.exists():
        candidate = directory / f"{safe_id}_{timestamp}_{suffix}.json"
        suffix += 1
    return candidate


def _load_sample(value: Any, index: int) -> TrajectorySample:
    context = f"raw_samples[{index}]"
    sample = _require_mapping(value, context)
    fields = {"t_ms", "valid", "x_norm", "y_norm", "u", "v", "invalid_reason"}
    _require_fields(sample, fields, context)
    valid = sample["valid"]
    if not isinstance(valid, bool):
        raise ValueError(f"{context}.valid must be a boolean.")
    return TrajectorySample(
        t_ms=_require_number(sample["t_ms"], f"{context}.t_ms"),
        valid=valid,
        x_norm=_optional_number(sample["x_norm"], f"{context}.x_norm"),
        y_norm=_optional_number(sample["y_norm"], f"{context}.y_norm"),
        u=_optional_integer(sample["u"], f"{context}.u"),
        v=_optional_integer(sample["v"], f"{context}.v"),
        invalid_reason=_optional_string(
            sample["invalid_reason"], f"{context}.invalid_reason"
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


def _require_integer(value: Any, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{context} must be an integer.")
    return value


def _optional_integer(value: Any, context: str) -> int | None:
    if value is None:
        return None
    return _require_integer(value, context)
