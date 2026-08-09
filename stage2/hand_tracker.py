"""MediaPipe HandLandmarker adapter for one BGR camera frame at a time."""

from __future__ import annotations

from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from .fingertip import normalized_to_pixel
from .hand_observation import HandObservation


DEFAULT_MODEL_PATH = Path(__file__).resolve().parent / "models" / "hand_landmarker.task"
INDEX_FINGER_TIP = 8


class HandTracker:
    """Convert BGR frames into first-hand observations using VIDEO mode."""

    def __init__(self, model_path: str | Path = DEFAULT_MODEL_PATH) -> None:
        self.model_path = Path(model_path)
        if not self.model_path.is_file():
            raise FileNotFoundError(
                "HandLandmarker model asset is missing: "
                f"{self.model_path}. See stage2/models/README.md."
            )

        options = vision.HandLandmarkerOptions(
            base_options=python.BaseOptions(model_asset_path=str(self.model_path)),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self._landmarker = vision.HandLandmarker.create_from_options(options)

    def process(self, bgr_frame: np.ndarray, timestamp_ms: float) -> HandObservation:
        """Return the first detected hand, or a clear no-hand observation."""
        if bgr_frame.ndim != 3 or bgr_frame.shape[2] != 3:
            raise ValueError("Expected a BGR frame with shape (height, width, 3).")

        timestamp_int_ms = int(timestamp_ms)
        rgb_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
        result = self._landmarker.detect_for_video(image, timestamp_int_ms)

        if not result.hand_landmarks:
            return HandObservation(timestamp_ms=timestamp_ms, detected=False)

        landmarks = result.hand_landmarks[0]
        landmarks_norm = tuple(
            (float(landmark.x), float(landmark.y), float(landmark.z))
            for landmark in landmarks
        )
        index_tip = landmarks[INDEX_FINGER_TIP]
        index_tip_norm = (float(index_tip.x), float(index_tip.y))
        index_tip_px = normalized_to_pixel(
            *index_tip_norm,
            width=bgr_frame.shape[1],
            height=bgr_frame.shape[0],
        )

        handedness = result.handedness[0][0]
        return HandObservation(
            timestamp_ms=timestamp_ms,
            detected=True,
            handedness=handedness.category_name,
            handedness_score=float(handedness.score),
            landmarks_norm=landmarks_norm,
            index_tip_norm=index_tip_norm,
            index_tip_px=index_tip_px,
        )

    def close(self) -> None:
        """Release MediaPipe resources; calling this repeatedly is safe."""
        if self._landmarker is not None:
            self._landmarker.close()
            self._landmarker = None

    def __enter__(self) -> "HandTracker":
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback) -> None:
        self.close()
