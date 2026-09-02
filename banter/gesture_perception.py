"""Independent 0/1/2-hand landmark perception for the Banter branch."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from pathlib import Path
import time
from typing import Any

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from stage2.fingertip import normalized_to_pixel
from stage2.hand_observation import HandObservation
from stage2.hand_tracker import DEFAULT_MODEL_PATH as STAGE2_HAND_MODEL_PATH

from .contracts import (
    GestureEvidence,
    GestureKind,
    GesturePerception,
    GestureSource,
    HandFrame,
    HandKey,
)
from .landmark_gestures import classify_landmarks


# Banter deliberately reuses the established Stage 2 model asset, but owns a
# separate two-hand instance and never changes the frozen Stage 2 tracker.
DEFAULT_MODEL_PATH = STAGE2_HAND_MODEL_PATH


@dataclass(frozen=True)
class GesturePerceptionConfig:
    """Presence/side gates for Banter; gesture acceptance is geometric."""

    # Presence is deliberately less strict than command classification.  A
    # marginal detected hand becomes UNKNOWN, never a command, which makes the
    # HUD truthful for unsupported poses without weakening event safety.
    min_hand_detection_confidence: float = 0.4
    min_hand_presence_confidence: float = 0.4
    min_tracking_confidence: float = 0.5
    min_handedness_score: float = 0.6

    def __post_init__(self) -> None:
        for name in (
            "min_hand_detection_confidence",
            "min_hand_presence_confidence",
            "min_tracking_confidence",
            "min_handedness_score",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or not 0.0 < value <= 1.0:
                raise ValueError(f"{name} must lie in (0, 1].")
            object.__setattr__(self, name, value)


def _best_category(categories: object) -> tuple[str | None, float | None]:
    if not categories:
        return None, None
    try:
        category = categories[0]
    except (IndexError, KeyError, TypeError):
        return None, None
    name = getattr(category, "category_name", None)
    score = getattr(category, "score", None)
    if not isinstance(name, str) or not name or score is None:
        return None, None
    try:
        numeric = float(score)
    except (TypeError, ValueError):
        return None, None
    return name, numeric if math.isfinite(numeric) else None


def _hand_key(handedness: str | None, score: float | None, minimum: float) -> HandKey:
    if score is None or score < minimum:
        return HandKey.UNKNOWN
    normalized = (handedness or "").upper()
    if normalized == "LEFT":
        return HandKey.LEFT
    if normalized == "RIGHT":
        return HandKey.RIGHT
    return HandKey.UNKNOWN


def _result_sequence(result: Any, attribute: str) -> tuple[Any, ...]:
    """Read one result field without letting a malformed frame escape."""
    value = getattr(result, attribute, ())
    if value is None:
        return ()
    try:
        return tuple(value)
    except TypeError:
        return ()


def perception_from_hand_landmarker_result(
    result: Any,
    *,
    frame_index: int,
    capture_t_ms: float,
    available_t_ms: float,
    frame_width: int,
    frame_height: int,
    config: GesturePerceptionConfig = GesturePerceptionConfig(),
) -> GesturePerception:
    """Map one HandLandmarker result to immutable Banter contracts.

    Landmark rows, not a gesture classifier's output, define 0/1/2 presence.
    Every structurally usable detected hand produces exactly one evidence row;
    unsupported or ambiguous geometry remains safe ``UNKNOWN`` evidence.
    """
    if frame_width <= 0 or frame_height <= 0:
        raise ValueError("frame dimensions must be positive.")
    handedness_by_hand = _result_sequence(result, "handedness")
    # Extra malformed rows are ignored rather than violating HandFrame's 0/1/2
    # contract or terminating a live camera loop.
    landmarks_by_hand = _result_sequence(result, "hand_landmarks")[:2]

    temporary: list[tuple[HandObservation, HandKey, GestureKind, float | None, str | None]] = []
    for hand_index, raw_landmarks in enumerate(landmarks_by_hand):
        incomplete_fields: list[str] = []
        if hand_index >= len(handedness_by_hand):
            incomplete_fields.append("handedness")
            hand_categories = ()
        else:
            hand_categories = handedness_by_hand[hand_index]
        try:
            landmarks = tuple((float(point.x), float(point.y), float(point.z)) for point in raw_landmarks)
        except (AttributeError, TypeError, ValueError):
            landmarks = ()
            incomplete_fields.append("hand_landmarks")

        hand_name, hand_score = _best_category(hand_categories)
        key = _hand_key(hand_name, hand_score, config.min_handedness_score)
        index_tip_norm = (landmarks[8][0], landmarks[8][1]) if len(landmarks) > 8 else None
        index_tip_px = (
            normalized_to_pixel(*index_tip_norm, width=frame_width, height=frame_height)
            if index_tip_norm is not None
            else None
        )
        hand = HandObservation(
            timestamp_ms=capture_t_ms,
            detected=True,
            handedness=hand_name,
            handedness_score=hand_score,
            landmarks_norm=landmarks,
            index_tip_norm=index_tip_norm,
            index_tip_px=index_tip_px,
        )
        decision = classify_landmarks(landmarks)
        reason = decision.rejection_reason
        if incomplete_fields:
            # A missing side row does not erase a detected hand.  It only makes
            # the hand role unusable for event/grammar output.
            key = HandKey.UNKNOWN
            reason = "incomplete_mediapipe_result:" + ",".join(incomplete_fields)
        elif key is HandKey.UNKNOWN and reason is None:
            reason = "handedness_below_threshold_or_unknown"
        temporary.append((hand, key, decision.gesture, decision.score, reason))

    side_counts = {
        side: sum(key is side for _hand, key, _kind, _score, _reason in temporary)
        for side in (HandKey.LEFT, HandKey.RIGHT)
    }
    evidence: list[GestureEvidence] = []
    for hand, key, kind, score, reason in temporary:
        if key in {HandKey.LEFT, HandKey.RIGHT} and side_counts[key] > 1:
            key = HandKey.UNKNOWN
            reason = "duplicate_handedness_in_frame"
        evidence.append(
            GestureEvidence(
                frame_index=frame_index,
                capture_t_ms=capture_t_ms,
                available_t_ms=available_t_ms,
                hand=hand,
                hand_key=key,
                gesture=kind,
                recognizer_score=score,
                source=GestureSource.LANDMARK_RULE,
                rejection_reason=reason,
            )
        )
    frame = HandFrame(
        frame_index=frame_index,
        capture_t_ms=capture_t_ms,
        available_t_ms=available_t_ms,
        hands=tuple(item[0] for item in temporary),
    )
    return GesturePerception(frame=frame, evidence=tuple(evidence))


def _with_available_t_ms(perception: GesturePerception, available_t_ms: float) -> GesturePerception:
    """Stamp a fully prepared perception at the first downstream-usable time."""
    frame = replace(perception.frame, available_t_ms=available_t_ms)
    evidence = tuple(replace(item, available_t_ms=available_t_ms) for item in perception.evidence)
    return GesturePerception(frame=frame, evidence=evidence)


class LandmarkGesturePerceiver:
    """Synchronous VIDEO-mode 0/1/2-hand perceiver for Banter only."""

    def __init__(
        self,
        model_path: str | Path = DEFAULT_MODEL_PATH,
        config: GesturePerceptionConfig = GesturePerceptionConfig(),
    ) -> None:
        if not isinstance(config, GesturePerceptionConfig):
            raise TypeError("config must be a GesturePerceptionConfig.")
        self.model_path = Path(model_path)
        if not self.model_path.is_file():
            raise FileNotFoundError(
                "HandLandmarker model asset is missing: "
                f"{self.model_path}. See stage2/models/README.md."
            )
        options = vision.HandLandmarkerOptions(
            base_options=python.BaseOptions(model_asset_path=str(self.model_path)),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=2,
            min_hand_detection_confidence=config.min_hand_detection_confidence,
            min_hand_presence_confidence=config.min_hand_presence_confidence,
            min_tracking_confidence=config.min_tracking_confidence,
        )
        self.config = config
        self._landmarker = vision.HandLandmarker.create_from_options(options)
        self._frame_index = 0
        self._last_capture_t_ms: float | None = None
        self._last_video_timestamp_ms: int | None = None

    def process(self, bgr_frame: np.ndarray, capture_t_ms: float) -> GesturePerception:
        """Perceive one BGR frame; availability is sampled after inference."""
        if bgr_frame.ndim != 3 or bgr_frame.shape[2] != 3:
            raise ValueError("Expected a BGR frame with shape (height, width, 3).")
        capture = float(capture_t_ms)
        if not math.isfinite(capture) or capture < 0.0:
            raise ValueError("capture_t_ms must be finite and non-negative.")
        video_timestamp = int(capture)
        if self._last_capture_t_ms is not None and capture <= self._last_capture_t_ms:
            raise ValueError("HandLandmarker capture timestamps must strictly increase.")
        if self._last_video_timestamp_ms is not None and video_timestamp <= self._last_video_timestamp_ms:
            raise ValueError("HandLandmarker VIDEO timestamps must increase by at least one millisecond.")
        rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        result = self._landmarker.detect_for_video(image, video_timestamp)
        # First complete all local landmark-to-gesture work.  Only then sample
        # availability: this timestamp means the complete perception object can
        # be consumed downstream, not merely that MediaPipe returned.
        prepared = perception_from_hand_landmarker_result(
            result,
            frame_index=self._frame_index,
            capture_t_ms=capture,
            available_t_ms=capture,
            frame_width=bgr_frame.shape[1],
            frame_height=bgr_frame.shape[0],
            config=self.config,
        )
        available = time.monotonic() * 1_000.0
        if not math.isfinite(available) or available < capture:
            raise ValueError("Post-inference available_t_ms must not precede capture_t_ms.")
        perception = _with_available_t_ms(prepared, available)
        self._last_capture_t_ms = capture
        self._last_video_timestamp_ms = video_timestamp
        self._frame_index += 1
        return perception

    def close(self) -> None:
        if self._landmarker is not None:
            self._landmarker.close()
            self._landmarker = None

    def __enter__(self) -> "LandmarkGesturePerceiver":
        return self

    def __exit__(self, _exc_type: object, _exc_value: object, _traceback: object) -> None:
        self.close()


# Short-lived compatibility aliases for callers written before the landmark
# path existed.  New Banter code uses the accurate names above.
GestureRecognizerPerceiver = LandmarkGesturePerceiver
perception_from_mediapipe_result = perception_from_hand_landmarker_result
