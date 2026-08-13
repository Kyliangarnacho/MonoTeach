"""Camera-intrinsic loading and sparse image-point undistortion for Stage 2.3."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import cv2
import numpy as np
from numpy.typing import ArrayLike, NDArray


_NPZ_REQUIRED_FIELDS = frozenset(
    {"camera_matrix", "dist_coeffs", "image_width", "image_height"}
)
_YAML_REQUIRED_FIELDS = tuple(sorted(_NPZ_REQUIRED_FIELDS))
_VALID_DISTORTION_COEFFICIENT_COUNTS = frozenset({4, 5, 8, 12, 14})


def _readonly_float64_copy(array: NDArray[np.float64]) -> NDArray[np.float64]:
    copied = np.array(array, dtype=np.float64, copy=True, order="C")
    copied.setflags(write=False)
    return copied


def _camera_matrix(K: ArrayLike) -> NDArray[np.float64]:
    try:
        matrix = np.asarray(K, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError("K must be numeric with shape (3, 3).") from error

    if matrix.shape != (3, 3):
        raise ValueError(f"K must have shape (3, 3); got {matrix.shape}.")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("K must contain only finite values.")
    return _readonly_float64_copy(matrix)


def _distortion_coefficients(D: ArrayLike) -> NDArray[np.float64]:
    try:
        coefficients = np.asarray(D, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError("D must be a numeric OpenCV distortion vector.") from error

    if coefficients.ndim == 1:
        flattened = coefficients
    elif coefficients.ndim == 2 and 1 in coefficients.shape:
        flattened = coefficients.reshape(-1)
    else:
        raise ValueError(
            "D must have shape (N,), (1, N), or (N, 1); "
            f"got {coefficients.shape}."
        )

    if flattened.size not in _VALID_DISTORTION_COEFFICIENT_COUNTS:
        valid_counts = sorted(_VALID_DISTORTION_COEFFICIENT_COUNTS)
        raise ValueError(
            f"D must contain one of {valid_counts} OpenCV distortion "
            f"coefficient counts; got {flattened.size}."
        )
    if not np.all(np.isfinite(flattened)):
        raise ValueError("D must contain only finite values.")

    return _readonly_float64_copy(flattened.reshape(1, -1))


def _positive_integer(value: object, name: str) -> int:
    if isinstance(value, np.ndarray):
        if value.shape != ():
            raise ValueError(f"{name} must be a scalar positive integer.")
        value = value.item()
    if isinstance(value, (bool, np.bool_)) or not isinstance(
        value,
        (int, np.integer),
    ):
        raise ValueError(f"{name} must be a positive integer.")
    if int(value) <= 0:
        raise ValueError(f"{name} must be a positive integer.")
    return int(value)


def _yaml_positive_integer(value: float, name: str) -> int:
    if not np.isfinite(value) or not float(value).is_integer():
        raise ValueError(f"{name} must be a positive integer.")
    return _positive_integer(int(value), name)


def _image_points(image_points: ArrayLike) -> NDArray[np.float64]:
    try:
        points = np.asarray(image_points, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError(
            "image_points must be numeric with shape (N, 2)."
        ) from error

    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError(
            f"image_points must have shape (N, 2); got {points.shape}."
        )
    if points.shape[0] < 1:
        raise ValueError("image_points must contain at least one point.")
    if not np.all(np.isfinite(points)):
        raise ValueError("image_points must contain only finite values.")
    return np.array(points, dtype=np.float64, copy=True, order="C")


@dataclass(frozen=True)
class CameraCalibration:
    """Validated camera intrinsics with readonly float64 K and D arrays.

    ``D`` is stored uniformly as an OpenCV-compatible row vector with shape
    ``(1, N)`` regardless of whether the source used a flat, row, or column vector.
    """

    K: ArrayLike
    D: ArrayLike
    frame_width: int
    frame_height: int
    source_path: str
    source_format: str

    def __post_init__(self) -> None:
        frame_width = _positive_integer(self.frame_width, "frame_width")
        frame_height = _positive_integer(self.frame_height, "frame_height")
        if not isinstance(self.source_path, str) or not self.source_path:
            raise ValueError("source_path must be a non-empty string.")
        if not isinstance(self.source_format, str) or not self.source_format:
            raise ValueError("source_format must be a non-empty string.")

        object.__setattr__(self, "K", _camera_matrix(self.K))
        object.__setattr__(self, "D", _distortion_coefficients(self.D))
        object.__setattr__(self, "frame_width", frame_width)
        object.__setattr__(self, "frame_height", frame_height)


def _load_npz(path: Path) -> CameraCalibration:
    try:
        loaded = np.load(path, allow_pickle=False)
    except Exception as error:
        raise ValueError(f"Cannot read NumPy calibration file {path}: {error}") from error

    if not isinstance(loaded, np.lib.npyio.NpzFile):
        raise ValueError(f"NumPy calibration file must be an NPZ archive: {path}")

    try:
        missing = sorted(_NPZ_REQUIRED_FIELDS.difference(loaded.files))
        if missing:
            raise ValueError(
                f"NPZ calibration file is missing required field(s): {missing}."
            )
        return CameraCalibration(
            K=loaded["camera_matrix"],
            D=loaded["dist_coeffs"],
            frame_width=_positive_integer(loaded["image_width"], "image_width"),
            frame_height=_positive_integer(loaded["image_height"], "image_height"),
            source_path=str(path),
            source_format="npz",
        )
    except ValueError:
        raise
    except Exception as error:
        raise ValueError(f"Malformed NPZ calibration file {path}: {error}") from error
    finally:
        loaded.close()


def _yaml_matrix(
    file_storage: cv2.FileStorage,
    field_name: str,
) -> NDArray[np.float64]:
    node = file_storage.getNode(field_name)
    if node.empty():
        raise ValueError(
            f"OpenCV YAML calibration file is missing required field: {field_name}."
        )
    try:
        matrix = node.mat()
    except cv2.error as error:
        raise ValueError(
            f"OpenCV YAML field {field_name} is not a valid matrix."
        ) from error
    if matrix is None:
        raise ValueError(f"OpenCV YAML field {field_name} is not a valid matrix.")
    return matrix


def _yaml_scalar(file_storage: cv2.FileStorage, field_name: str) -> float:
    node = file_storage.getNode(field_name)
    if node.empty():
        raise ValueError(
            f"OpenCV YAML calibration file is missing required field: {field_name}."
        )
    try:
        return float(node.real())
    except (TypeError, ValueError, cv2.error) as error:
        raise ValueError(
            f"OpenCV YAML field {field_name} is not a valid scalar."
        ) from error


def _load_opencv_yaml(path: Path) -> CameraCalibration:
    try:
        file_storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    except (cv2.error, SystemError) as error:
        raise ValueError(
            f"Cannot parse OpenCV YAML calibration file: {path}"
        ) from error

    if not file_storage.isOpened():
        file_storage.release()
        raise ValueError(f"Cannot open OpenCV YAML calibration file: {path}")

    try:
        for field_name in _YAML_REQUIRED_FIELDS:
            if file_storage.getNode(field_name).empty():
                raise ValueError(
                    "OpenCV YAML calibration file is missing required field: "
                    f"{field_name}."
                )
        return CameraCalibration(
            K=_yaml_matrix(file_storage, "camera_matrix"),
            D=_yaml_matrix(file_storage, "dist_coeffs"),
            frame_width=_yaml_positive_integer(
                _yaml_scalar(file_storage, "image_width"),
                "image_width",
            ),
            frame_height=_yaml_positive_integer(
                _yaml_scalar(file_storage, "image_height"),
                "image_height",
            ),
            source_path=str(path),
            source_format="opencv_yaml",
        )
    except ValueError:
        raise
    except Exception as error:
        raise ValueError(f"Malformed OpenCV YAML calibration file {path}: {error}") from error
    finally:
        file_storage.release()


def load_camera_calibration(path: str | Path) -> CameraCalibration:
    """Load required intrinsics from a NumPy NPZ or OpenCV YAML asset."""
    calibration_path = Path(path).expanduser().resolve()
    if not calibration_path.is_file():
        raise ValueError(f"Camera calibration file does not exist: {calibration_path}")

    suffix = calibration_path.suffix.lower()
    if suffix == ".npz":
        return _load_npz(calibration_path)
    if suffix in {".yaml", ".yml"}:
        return _load_opencv_yaml(calibration_path)
    raise ValueError(
        "Unsupported camera calibration format "
        f"{suffix!r}; expected .npz, .yaml, or .yml."
    )


def validate_calibration_resolution(
    calibration: CameraCalibration,
    frame_width: int,
    frame_height: int,
) -> None:
    """Require an exact frame/calibration resolution match without scaling K."""
    if not isinstance(calibration, CameraCalibration):
        raise TypeError("calibration must be a CameraCalibration.")
    actual_width = _positive_integer(frame_width, "frame_width")
    actual_height = _positive_integer(frame_height, "frame_height")
    if (
        actual_width != calibration.frame_width
        or actual_height != calibration.frame_height
    ):
        raise ValueError(
            "Frame resolution does not match camera calibration: "
            f"frame={actual_width}x{actual_height}, "
            f"calibration={calibration.frame_width}x{calibration.frame_height}. "
            "Automatic K scaling is not supported."
        )


def undistort_image_points(
    image_points: ArrayLike,
    calibration: CameraCalibration,
) -> NDArray[np.float64]:
    """Convert N distorted pixel points to undistorted pixel coordinates."""
    if not isinstance(calibration, CameraCalibration):
        raise TypeError("calibration must be a CameraCalibration.")
    distorted_points = _image_points(image_points)

    try:
        undistorted_points = cv2.undistortPoints(
            distorted_points.reshape(-1, 1, 2),
            cameraMatrix=calibration.K,
            distCoeffs=calibration.D,
            P=calibration.K,
        ).reshape(-1, 2)
    except cv2.error as error:
        raise ValueError(f"OpenCV could not undistort image_points: {error}") from error

    if not np.all(np.isfinite(undistorted_points)):
        raise ValueError("Undistortion produced non-finite pixel coordinates.")
    return np.array(undistorted_points, dtype=np.float64, copy=True)


def main(argv: Sequence[str] | None = None) -> int:
    """Print one calibration asset without opening a camera."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", help="Path to camera_params.npz/.yaml/.yml")
    arguments = parser.parse_args(argv)

    calibration = load_camera_calibration(arguments.path)
    print(f"resolution: {calibration.frame_width} x {calibration.frame_height}")
    print("K:")
    print(calibration.K)
    print("D:")
    print(calibration.D)
    print(f"source: {calibration.source_path} ({calibration.source_format})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
