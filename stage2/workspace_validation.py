"""Independent pointwise validation for an existing workspace calibration."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
import re
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .camera_calibration import CameraCalibration, undistort_image_points
from .workspace_geometry import WorkspaceCalibration, image_points_to_workspace


DEFAULT_WORKSPACE_VALIDATION_DIRECTORY = Path("data") / "validations"


def _finite_number(value: object, name: str) -> float:
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(f"{name} must be a finite number.")
    try:
        numeric_value = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be a finite number.") from error
    if not np.isfinite(numeric_value):
        raise ValueError(f"{name} must be a finite number.")
    return numeric_value


def _points_array(points: ArrayLike, name: str) -> NDArray[np.float64]:
    try:
        array = np.asarray(points, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be numeric with shape (N, 2).") from error
    if array.ndim != 2 or array.shape[1] != 2:
        raise ValueError(f"{name} must have shape (N, 2); got {array.shape}.")
    if array.shape[0] == 0:
        raise ValueError(f"{name} must contain at least one point.")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values.")
    return np.array(array, dtype=np.float64, copy=True, order="C")


@dataclass(frozen=True)
class ValidationPoint:
    """One independent workspace measurement and its mapping error in millimetres."""

    true_x_mm: float
    true_y_mm: float
    predicted_x_mm: float
    predicted_y_mm: float
    error_x_mm: float
    error_y_mm: float
    error_mm: float

    def __post_init__(self) -> None:
        for name in (
            "true_x_mm",
            "true_y_mm",
            "predicted_x_mm",
            "predicted_y_mm",
            "error_x_mm",
            "error_y_mm",
            "error_mm",
        ):
            object.__setattr__(self, name, _finite_number(getattr(self, name), name))
        if self.error_mm < 0.0:
            raise ValueError("error_mm must be non-negative.")


@dataclass(frozen=True)
class ValidationReport:
    """Immutable per-point and aggregate error statistics in millimetres."""

    points: tuple[ValidationPoint, ...]
    mean_error_mm: float
    rms_error_mm: float
    max_error_mm: float

    def __post_init__(self) -> None:
        if not isinstance(self.points, tuple) or not self.points:
            raise ValueError("points must be a non-empty immutable tuple.")
        if not all(isinstance(point, ValidationPoint) for point in self.points):
            raise TypeError("points must contain only ValidationPoint values.")
        for name in ("mean_error_mm", "rms_error_mm", "max_error_mm"):
            object.__setattr__(self, name, _finite_number(getattr(self, name), name))
        if (
            self.mean_error_mm < 0.0
            or self.rms_error_mm < 0.0
            or self.max_error_mm < 0.0
        ):
            raise ValueError("Validation error statistics must be non-negative.")


def evaluate_workspace_points(
    true_points_mm: ArrayLike,
    raw_image_points: ArrayLike,
    camera_calibration: CameraCalibration,
    workspace_calibration: WorkspaceCalibration,
) -> ValidationReport:
    """Evaluate independent points with the fixed raw→undistort→H mapping chain."""
    if not isinstance(camera_calibration, CameraCalibration):
        raise TypeError("camera_calibration must be a CameraCalibration.")
    if not isinstance(workspace_calibration, WorkspaceCalibration):
        raise TypeError("workspace_calibration must be a WorkspaceCalibration.")
    if (
        camera_calibration.frame_width != workspace_calibration.frame_width
        or camera_calibration.frame_height != workspace_calibration.frame_height
    ):
        raise ValueError(
            "Camera calibration and workspace calibration resolutions must match."
        )

    true_points = _points_array(true_points_mm, "true_points_mm")
    raw_points = _points_array(raw_image_points, "raw_image_points")
    if true_points.shape[0] != raw_points.shape[0]:
        raise ValueError(
            "true_points_mm and raw_image_points must contain the same number of "
            "points."
        )

    undistorted_points = undistort_image_points(raw_points, camera_calibration)
    predicted_points = image_points_to_workspace(
        undistorted_points,
        workspace_calibration.H_image_to_workspace,
    )
    errors_xy = predicted_points - true_points
    errors_mm = np.linalg.norm(errors_xy, axis=1)
    points = tuple(
        ValidationPoint(
            true_x_mm=true_x,
            true_y_mm=true_y,
            predicted_x_mm=predicted_x,
            predicted_y_mm=predicted_y,
            error_x_mm=error_x,
            error_y_mm=error_y,
            error_mm=error_mm,
        )
        for (true_x, true_y), (predicted_x, predicted_y), (error_x, error_y), error_mm in zip(
            true_points,
            predicted_points,
            errors_xy,
            errors_mm,
            strict=True,
        )
    )
    return ValidationReport(
        points=points,
        mean_error_mm=float(np.mean(errors_mm)),
        rms_error_mm=float(np.sqrt(np.mean(np.square(errors_mm)))),
        max_error_mm=float(np.max(errors_mm)),
    )


def save_workspace_validation_json(
    report: ValidationReport,
    raw_image_points: ArrayLike,
    workspace_calibration: WorkspaceCalibration,
    output_directory: str | Path = DEFAULT_WORKSPACE_VALIDATION_DIRECTORY,
) -> Path:
    """Save an independent validation report without changing its calibration."""
    if not isinstance(report, ValidationReport):
        raise TypeError("report must be a ValidationReport.")
    if not isinstance(workspace_calibration, WorkspaceCalibration):
        raise TypeError("workspace_calibration must be a WorkspaceCalibration.")
    raw_points = _points_array(raw_image_points, "raw_image_points")
    if raw_points.shape[0] != len(report.points):
        raise ValueError("raw_image_points must have one entry for every report point.")

    safe_id = re.sub(r"[^A-Za-z0-9._-]+", "_", workspace_calibration.calibration_id)
    output_path = Path(output_directory) / f"{safe_id}_validation.json"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    payload: dict[str, Any] = {
        "workspace_calibration_id": workspace_calibration.calibration_id,
        "camera_calibration_reference": workspace_calibration.camera_calibration_reference,
        "raw_image_points": raw_points.tolist(),
        "points": [asdict(point) for point in report.points],
        "mean_error_mm": report.mean_error_mm,
        "rms_error_mm": report.rms_error_mm,
        "max_error_mm": report.max_error_mm,
    }
    output_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return output_path
