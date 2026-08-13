"""Derived workspace-millimetre trajectories from immutable Stage 2.2 samples."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .camera_calibration import CameraCalibration, undistort_image_points
from .trajectory import Trajectory2D, TrajectorySample
from .trajectory_filter import EMAConfig, apply_ema_filter
from .trajectory_quality import QualityGateConfig, apply_quality_gate
from .workspace_geometry import (
    WORKSPACE_COORDINATE_FRAME,
    WorkspaceCalibration,
    inside_workspace,
    image_points_to_workspace,
)


def _finite_nonnegative(value: object, name: str) -> float:
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(f"{name} must be finite and non-negative.")
    try:
        numeric_value = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be finite and non-negative.") from error
    if not np.isfinite(numeric_value) or numeric_value < 0.0:
        raise ValueError(f"{name} must be finite and non-negative.")
    return numeric_value


@dataclass(frozen=True)
class WorkspaceTrajectorySample:
    """One derived workspace sample; validity and workspace inclusion are distinct."""

    t_ms: float
    valid: bool
    x_mm: float | None
    y_mm: float | None
    inside_workspace: bool
    invalid_reason: str | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "t_ms", _finite_nonnegative(self.t_ms, "t_ms"))
        if not isinstance(self.valid, (bool, np.bool_)):
            raise TypeError("valid must be a boolean.")
        if not isinstance(self.inside_workspace, (bool, np.bool_)):
            raise TypeError("inside_workspace must be a boolean.")
        object.__setattr__(self, "valid", bool(self.valid))
        object.__setattr__(self, "inside_workspace", bool(self.inside_workspace))

        if self.valid:
            if self.x_mm is None or self.y_mm is None:
                raise ValueError("A valid workspace sample must include x_mm and y_mm.")
            if self.invalid_reason is not None:
                raise ValueError("A valid workspace sample cannot have an invalid_reason.")
            object.__setattr__(self, "x_mm", _finite_nonnegative_or_negative(self.x_mm, "x_mm"))
            object.__setattr__(self, "y_mm", _finite_nonnegative_or_negative(self.y_mm, "y_mm"))
            return

        if self.x_mm is not None or self.y_mm is not None:
            raise ValueError("An invalid workspace sample cannot include millimetre coordinates.")
        if self.inside_workspace:
            raise ValueError("An invalid workspace sample cannot be inside_workspace.")
        if not isinstance(self.invalid_reason, str) or not self.invalid_reason:
            raise ValueError("An invalid workspace sample must include an invalid_reason.")


def _finite_nonnegative_or_negative(value: object, name: str) -> float:
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(f"{name} must be finite.")
    try:
        numeric_value = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be finite.") from error
    if not np.isfinite(numeric_value):
        raise ValueError(f"{name} must be finite.")
    return numeric_value


@dataclass(frozen=True)
class WorkspaceTrajectoryMetadata:
    """Identity and physical coordinate context for a derived trajectory."""

    source_trajectory_id: str
    workspace_calibration_id: str
    coordinate_frame: str
    width_mm: float
    height_mm: float

    def __post_init__(self) -> None:
        for name in ("source_trajectory_id", "workspace_calibration_id", "coordinate_frame"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value:
                raise ValueError(f"{name} must be a non-empty string.")
        if self.coordinate_frame != WORKSPACE_COORDINATE_FRAME:
            raise ValueError(
                f"coordinate_frame must be {WORKSPACE_COORDINATE_FRAME!r}."
            )
        for name in ("width_mm", "height_mm"):
            value = _finite_nonnegative(getattr(self, name), name)
            if value <= 0.0:
                raise ValueError(f"{name} must be positive.")
            object.__setattr__(self, name, value)


@dataclass(frozen=True)
class WorkspaceTrajectory2D:
    """Immutable workspace-mm derivative of a canonical image-plane trajectory."""

    metadata: WorkspaceTrajectoryMetadata
    samples: tuple[WorkspaceTrajectorySample, ...] = ()

    def __post_init__(self) -> None:
        if not isinstance(self.metadata, WorkspaceTrajectoryMetadata):
            raise TypeError("metadata must be a WorkspaceTrajectoryMetadata.")
        if not isinstance(self.samples, tuple):
            raise TypeError("samples must be an immutable tuple.")
        if not all(isinstance(sample, WorkspaceTrajectorySample) for sample in self.samples):
            raise TypeError("samples must contain only WorkspaceTrajectorySample values.")


def _invalid_workspace_sample(sample: TrajectorySample) -> WorkspaceTrajectorySample:
    if sample.invalid_reason is None:
        raise ValueError("An invalid processed trajectory sample must include invalid_reason.")
    return WorkspaceTrajectorySample(
        t_ms=sample.t_ms,
        valid=False,
        x_mm=None,
        y_mm=None,
        inside_workspace=False,
        invalid_reason=sample.invalid_reason,
    )


def trajectory_to_workspace(
    trajectory: Trajectory2D,
    camera_calibration: CameraCalibration,
    workspace_calibration: WorkspaceCalibration,
    *,
    quality_config: QualityGateConfig = QualityGateConfig(),
    ema_config: EMAConfig = EMAConfig(),
) -> WorkspaceTrajectory2D:
    """Create a workspace-mm derivative without changing canonical raw samples."""
    if not isinstance(trajectory, Trajectory2D):
        raise TypeError("trajectory must be a Trajectory2D.")
    if not isinstance(camera_calibration, CameraCalibration):
        raise TypeError("camera_calibration must be a CameraCalibration.")
    if not isinstance(workspace_calibration, WorkspaceCalibration):
        raise TypeError("workspace_calibration must be a WorkspaceCalibration.")
    if not isinstance(quality_config, QualityGateConfig):
        raise TypeError("quality_config must be a QualityGateConfig.")
    if not isinstance(ema_config, EMAConfig):
        raise TypeError("ema_config must be an EMAConfig.")
    if (
        trajectory.metadata.frame_width != camera_calibration.frame_width
        or trajectory.metadata.frame_height != camera_calibration.frame_height
        or trajectory.metadata.frame_width != workspace_calibration.frame_width
        or trajectory.metadata.frame_height != workspace_calibration.frame_height
    ):
        raise ValueError(
            "Trajectory, camera calibration, and workspace calibration resolutions "
            "must match."
        )

    quality_result = apply_quality_gate(trajectory.raw_samples, quality_config)
    processed_samples = apply_ema_filter(
        quality_result.processed_samples,
        trajectory.metadata,
        ema_config,
    )
    workspace_samples: list[WorkspaceTrajectorySample] = []
    for sample in processed_samples:
        if not sample.valid:
            workspace_samples.append(_invalid_workspace_sample(sample))
            continue

        if sample.u is None or sample.v is None:
            raise ValueError("A valid processed trajectory sample must include u and v.")
        undistorted_point = undistort_image_points(
            [[float(sample.u), float(sample.v)]],
            camera_calibration,
        )
        x_mm, y_mm = image_points_to_workspace(
            undistorted_point,
            workspace_calibration.H_image_to_workspace,
        )[0]
        workspace_samples.append(
            WorkspaceTrajectorySample(
                t_ms=sample.t_ms,
                valid=True,
                x_mm=x_mm,
                y_mm=y_mm,
                inside_workspace=inside_workspace(
                    x_mm,
                    y_mm,
                    workspace_calibration.workspace_definition,
                ),
                invalid_reason=None,
            )
        )

    return WorkspaceTrajectory2D(
        metadata=WorkspaceTrajectoryMetadata(
            source_trajectory_id=trajectory.metadata.trajectory_id,
            workspace_calibration_id=workspace_calibration.calibration_id,
            coordinate_frame=workspace_calibration.workspace_definition.coordinate_frame,
            width_mm=workspace_calibration.workspace_definition.width_mm,
            height_mm=workspace_calibration.workspace_definition.height_mm,
        ),
        samples=tuple(workspace_samples),
    )
