"""Minimal OpenCV camera access for Stage 2.1."""

from __future__ import annotations

from dataclasses import dataclass
import time

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraConfig:
    """Requested camera selection and capture profile."""

    index: int = 1
    backend: int = cv2.CAP_DSHOW
    width: int = 1280
    height: int = 720
    fps: float = 30.0


@dataclass(frozen=True)
class CameraProfile:
    """Profile actually reported by an opened camera."""

    backend: str
    width: int
    height: int
    fps: float


class CameraStream:
    """Own one OpenCV capture handle and expose timestamped frame reads."""

    def __init__(self, config: CameraConfig) -> None:
        self.config = config
        self._capture: cv2.VideoCapture | None = None
        self._profile: CameraProfile | None = None

    @property
    def is_open(self) -> bool:
        """Whether the underlying capture is currently ready for reads."""
        return self._capture is not None and self._capture.isOpened()

    @property
    def profile(self) -> CameraProfile:
        """Return the actual profile after :meth:`open` succeeds."""
        if self._profile is None:
            raise RuntimeError("Camera profile is unavailable before a successful open().")
        return self._profile

    def open(self) -> CameraProfile:
        """Open the configured camera, request its profile, and return actual values."""
        if self.is_open:
            return self.profile

        capture = cv2.VideoCapture(self.config.index, self.config.backend)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(
                "Could not open camera "
                f"index={self.config.index} backend={self.config.backend}."
            )

        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        capture.set(cv2.CAP_PROP_FPS, self.config.fps)

        self._capture = capture
        self._profile = CameraProfile(
            backend=capture.getBackendName(),
            width=int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
            height=int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            fps=float(capture.get(cv2.CAP_PROP_FPS)),
        )
        return self._profile

    def read(self) -> tuple[np.ndarray, float]:
        """Return one BGR frame and a monotonically increasing timestamp in ms."""
        if not self.is_open or self._capture is None:
            raise RuntimeError("Cannot read frame: camera is not open.")

        ok, frame = self._capture.read()
        if not ok or frame is None:
            raise RuntimeError("Camera frame read failed.")
        return frame, time.monotonic() * 1_000.0

    def release(self) -> None:
        """Release the capture handle; calling this repeatedly is safe."""
        if self._capture is not None:
            self._capture.release()
        self._capture = None
        self._profile = None
