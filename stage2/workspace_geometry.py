"""Stage 2.3 workspace-plane data contracts and pure geometry helpers."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
from numpy.typing import ArrayLike, NDArray


WORKSPACE_COORDINATE_FRAME = "workspace_2d"


def _finite_positive(value: float, name: str) -> float:
    value = float(value)
    if not np.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive.")
    return value


def _points_array(
    points: ArrayLike,
    name: str,
    *,
    minimum_count: int = 1,
) -> NDArray[np.float64]:
    try:
        array = np.asarray(points, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be numeric with shape (N, 2).") from error

    if array.ndim != 2 or array.shape[1] != 2:
        raise ValueError(f"{name} must have shape (N, 2); got {array.shape}.")
    if array.shape[0] < minimum_count:
        raise ValueError(
            f"{name} must contain at least {minimum_count} point(s); "
            f"got {array.shape[0]}."
        )
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values.")

    return np.array(array, dtype=np.float64, copy=True, order="C")


def _homography_array(H_image_to_workspace: ArrayLike) -> NDArray[np.float64]:
    try:
        matrix = np.asarray(H_image_to_workspace, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError(
            "H_image_to_workspace must be numeric with shape (3, 3)."
        ) from error

    if matrix.shape != (3, 3):
        raise ValueError(
            "H_image_to_workspace must have shape (3, 3); "
            f"got {matrix.shape}."
        )
    if not np.all(np.isfinite(matrix)):
        raise ValueError("H_image_to_workspace must contain only finite values.")
    if np.linalg.matrix_rank(matrix) < 3:
        raise ValueError("H_image_to_workspace must be non-singular.")

    return np.array(matrix, dtype=np.float64, copy=True, order="C")


def _readonly_copy(array: NDArray[np.float64]) -> NDArray[np.float64]:
    copied = np.array(array, dtype=np.float64, copy=True, order="C")
    copied.setflags(write=False)
    return copied


@dataclass(frozen=True)
class WorkspaceDefinition:
    """Physical dimensions and coordinate-frame name of a planar workspace."""

    width_mm: float
    height_mm: float
    coordinate_frame: str = WORKSPACE_COORDINATE_FRAME

    def __post_init__(self) -> None:
        object.__setattr__(self, "width_mm", _finite_positive(self.width_mm, "width_mm"))
        object.__setattr__(
            self,
            "height_mm",
            _finite_positive(self.height_mm, "height_mm"),
        )
        if not isinstance(self.coordinate_frame, str) or not self.coordinate_frame:
            raise ValueError("coordinate_frame must be a non-empty string.")


@dataclass(frozen=True)
class WorkspacePoint:
    """One point expressed in millimetres in the workspace coordinate frame."""

    x_mm: float
    y_mm: float
    inside_workspace: bool

    def __post_init__(self) -> None:
        if not np.isfinite(self.x_mm) or not np.isfinite(self.y_mm):
            raise ValueError("x_mm and y_mm must be finite.")
        if not isinstance(self.inside_workspace, (bool, np.bool_)):
            raise TypeError("inside_workspace must be a boolean.")
        object.__setattr__(self, "x_mm", float(self.x_mm))
        object.__setattr__(self, "y_mm", float(self.y_mm))
        object.__setattr__(self, "inside_workspace", bool(self.inside_workspace))


@dataclass(frozen=True)
class WorkspaceCalibration:
    """Image-to-workspace calibration and the context needed to interpret it."""

    schema_version: str
    calibration_id: str
    frame_width: int
    frame_height: int
    workspace_definition: WorkspaceDefinition
    image_points: ArrayLike
    workspace_points_mm: ArrayLike
    H_image_to_workspace: ArrayLike
    camera_calibration_reference: str

    def __post_init__(self) -> None:
        if not isinstance(self.schema_version, str) or not self.schema_version:
            raise ValueError("schema_version must be a non-empty string.")
        if not isinstance(self.calibration_id, str) or not self.calibration_id:
            raise ValueError("calibration_id must be a non-empty string.")
        if (
            isinstance(self.frame_width, (bool, np.bool_))
            or isinstance(self.frame_height, (bool, np.bool_))
            or not isinstance(self.frame_width, (int, np.integer))
            or not isinstance(self.frame_height, (int, np.integer))
            or self.frame_width <= 0
            or self.frame_height <= 0
        ):
            raise ValueError("frame_width and frame_height must be positive integers.")
        if (
            not isinstance(self.camera_calibration_reference, str)
            or not self.camera_calibration_reference
        ):
            raise ValueError("camera_calibration_reference must be a non-empty string.")
        if not isinstance(self.workspace_definition, WorkspaceDefinition):
            raise TypeError("workspace_definition must be a WorkspaceDefinition.")

        image_points = _points_array(self.image_points, "image_points")
        workspace_points = _points_array(
            self.workspace_points_mm,
            "workspace_points_mm",
        )
        if image_points.shape[0] != workspace_points.shape[0]:
            raise ValueError(
                "image_points and workspace_points_mm must contain the same "
                "number of points."
            )
        if image_points.shape[0] < 4:
            raise ValueError(
                "image_points and workspace_points_mm must contain at least "
                "4 corresponding points."
            )

        object.__setattr__(self, "frame_width", int(self.frame_width))
        object.__setattr__(self, "frame_height", int(self.frame_height))
        object.__setattr__(self, "image_points", _readonly_copy(image_points))
        object.__setattr__(
            self,
            "workspace_points_mm",
            _readonly_copy(workspace_points),
        )
        object.__setattr__(
            self,
            "H_image_to_workspace",
            _readonly_copy(_homography_array(self.H_image_to_workspace)),
        )


def compute_image_to_workspace_homography(
    image_points: ArrayLike,
    workspace_points_mm: ArrayLike,
) -> NDArray[np.float64]:
    """Compute a homography mapping image pixels to workspace millimetres."""
    image_array = _points_array(image_points, "image_points")
    workspace_array = _points_array(
        workspace_points_mm,
        "workspace_points_mm",
    )
    if image_array.shape[0] != workspace_array.shape[0]:
        raise ValueError(
            "image_points and workspace_points_mm must contain the same "
            "number of points."
        )
    if image_array.shape[0] < 4:
        raise ValueError(
            "image_points and workspace_points_mm must contain at least "
            "4 corresponding points."
        )

    try:
        H_image_to_workspace, _ = cv2.findHomography(
            image_array,
            workspace_array,
            method=0,
        )
    except cv2.error as error:
        raise ValueError(
            "Cannot compute H_image_to_workspace from the supplied points; "
            "the point configuration may be degenerate."
        ) from error
    if H_image_to_workspace is None:
        raise ValueError(
            "Cannot compute H_image_to_workspace from the supplied points; "
            "the point configuration may be degenerate."
        )

    H_image_to_workspace = _homography_array(H_image_to_workspace)
    scale = H_image_to_workspace[2, 2]
    if not np.isclose(scale, 0.0):
        H_image_to_workspace = H_image_to_workspace / scale
    return H_image_to_workspace


def image_points_to_workspace(
    image_points: ArrayLike,
    H_image_to_workspace: ArrayLike,
) -> NDArray[np.float64]:
    """Map N image-plane points to workspace millimetres without changing inputs."""
    image_array = _points_array(image_points, "image_points")
    homography = _homography_array(H_image_to_workspace)

    workspace_points = cv2.perspectiveTransform(
        image_array.reshape(-1, 1, 2),
        homography,
    ).reshape(-1, 2)
    if not np.all(np.isfinite(workspace_points)):
        raise ValueError(
            "H_image_to_workspace maps at least one image point to a non-finite value."
        )
    return np.array(workspace_points, dtype=np.float64, copy=True)


def inside_workspace(
    X_mm: float,
    Y_mm: float,
    workspace_definition: WorkspaceDefinition,
) -> bool:
    """Return whether a finite point is on or within the workspace boundary."""
    if not isinstance(workspace_definition, WorkspaceDefinition):
        raise TypeError("workspace_definition must be a WorkspaceDefinition.")
    if not np.isfinite(X_mm) or not np.isfinite(Y_mm):
        raise ValueError("X_mm and Y_mm must be finite.")

    return bool(
        0.0 <= float(X_mm) <= workspace_definition.width_mm
        and 0.0 <= float(Y_mm) <= workspace_definition.height_mm
    )
