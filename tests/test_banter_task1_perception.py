"""Camera-free structural tests for Banter's HandLandmarker adapter."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from banter.contracts import GestureKind, GestureSource, HandKey
from banter.demo_gesture_events import _label, status_lines
import banter.gesture_perception as gesture_perception_module
from banter.gesture_perception import (
    GesturePerceptionConfig,
    LandmarkGesturePerceiver,
    perception_from_hand_landmarker_result,
)


_CHAINS = {
    "thumb": (1, 2, 3, 4),
    "index": (5, 6, 7, 8),
    "middle": (9, 10, 11, 12),
    "ring": (13, 14, 15, 16),
    "pinky": (17, 18, 19, 20),
}
_BASE_X = {"thumb": 0.28, "index": 0.40, "middle": 0.50, "ring": 0.60, "pinky": 0.70}


def _category(name: str, score: float) -> SimpleNamespace:
    return SimpleNamespace(category_name=name, score=score)


def _landmarks(*, extended: set[str]) -> list[SimpleNamespace]:
    points = [(0.5, 0.85, 0.0) for _ in range(21)]
    for name, chain in _CHAINS.items():
        x = _BASE_X[name]
        if name in extended:
            chain_points = ((x, 0.65, 0.0), (x, 0.50, 0.0), (x, 0.35, 0.0), (x, 0.20, 0.0))
        else:
            chain_points = ((x, 0.65, 0.0), (x, 0.50, 0.0), (x + 0.02, 0.62, 0.0), (x - 0.02, 0.66, 0.0))
        for index, point in zip(chain, chain_points):
            points[index] = point
    return [SimpleNamespace(x=x, y=y, z=z) for x, y, z in points]


def _result(rows: list[tuple[str, float, set[str]]]) -> SimpleNamespace:
    return SimpleNamespace(
        handedness=[[_category(hand, score)] for hand, score, _extended in rows],
        hand_landmarks=[_landmarks(extended=extended) for _hand, _score, extended in rows],
    )


def _perceive(rows: list[tuple[str, float, set[str]]]):
    return perception_from_hand_landmarker_result(
        _result(rows),
        frame_index=7,
        capture_t_ms=100.0,
        available_t_ms=102.0,
        frame_width=640,
        frame_height=480,
    )


def test_perception_keeps_zero_one_or_two_detected_hands_and_rule_labels() -> None:
    assert _perceive([]).frame.hands == ()

    one = _perceive([("Left", 0.9, {"index"})])
    assert len(one.frame.hands) == len(one.evidence) == 1
    assert one.evidence[0].hand_key is HandKey.LEFT
    assert one.evidence[0].gesture is GestureKind.POINT_ONE
    assert one.evidence[0].source is GestureSource.LANDMARK_RULE
    assert one.evidence[0].hand.index_tip_px == (256, 96)

    two = _perceive([
        ("Right", 0.9, set()),
        ("Left", 0.91, {"index", "middle"}),
    ])
    assert [item.hand_key for item in two.evidence] == [HandKey.RIGHT, HandKey.LEFT]
    assert [item.gesture for item in two.evidence] == [GestureKind.FIST, GestureKind.VICTORY_TWO]


def test_unsupported_three_or_four_retains_detected_hand_as_unknown() -> None:
    for extended in ({"index", "middle", "ring"}, {"index", "middle", "ring", "pinky"}):
        perceived = _perceive([("Left", 0.9, extended)])
        assert len(perceived.frame.hands) == len(perceived.evidence) == 1
        evidence = perceived.evidence[0]
        assert evidence.hand_key is HandKey.LEFT
        assert evidence.gesture is GestureKind.UNKNOWN
        assert evidence.rejection_reason == "unsupported_landmark_gesture"


def test_adapter_retains_hand_when_handedness_is_missing_or_ambiguous() -> None:
    missing_handedness = SimpleNamespace(
        handedness=[],
        hand_landmarks=[_landmarks(extended=set())],
    )
    degraded = perception_from_hand_landmarker_result(
        missing_handedness,
        frame_index=0,
        capture_t_ms=1.0,
        available_t_ms=2.0,
        frame_width=10,
        frame_height=10,
        config=GesturePerceptionConfig(),
    )
    assert len(degraded.frame.hands) == len(degraded.evidence) == 1
    assert degraded.evidence[0].hand_key is HandKey.UNKNOWN
    assert degraded.evidence[0].gesture is GestureKind.FIST
    assert degraded.evidence[0].rejection_reason == "incomplete_mediapipe_result:handedness"

    duplicate = _perceive([
        ("Left", 0.9, set()),
        ("Left", 0.95, {"index", "middle"}),
    ])
    assert [item.hand_key for item in duplicate.evidence] == [HandKey.UNKNOWN, HandKey.UNKNOWN]
    assert all(item.rejection_reason == "duplicate_handedness_in_frame" for item in duplicate.evidence)


def test_adapter_safely_handles_malformed_landmarks_and_true_no_hand() -> None:
    malformed = SimpleNamespace(
        handedness=[[_category("Left", 0.9)]],
        hand_landmarks=[[SimpleNamespace(x=0.1, y=0.1, z=0.0)]],
    )
    perceived = perception_from_hand_landmarker_result(
        malformed,
        frame_index=1,
        capture_t_ms=3.0,
        available_t_ms=4.0,
        frame_width=10,
        frame_height=10,
    )
    assert len(perceived.frame.hands) == 1
    assert perceived.evidence[0].gesture is GestureKind.UNKNOWN
    assert perceived.evidence[0].rejection_reason == "invalid_landmarks"

    no_landmarks = SimpleNamespace(
        handedness=[[_category("Left", 0.9)]],
        hand_landmarks=[],
    )
    empty = perception_from_hand_landmarker_result(
        no_landmarks,
        frame_index=2,
        capture_t_ms=5.0,
        available_t_ms=6.0,
        frame_width=10,
        frame_height=10,
    )
    assert empty.frame.hands == ()
    assert empty.evidence == ()


def test_demo_hud_has_only_no_hand_or_concise_per_hand_labels() -> None:
    no_hand = status_lines(_perceive([]), fps=30.0, mirror_preview=True, event_count=0)
    assert no_hand == ("FPS: 30.0", "Events: 0", "Mirror: ON", "No hand")

    unsupported = _perceive([("Left", 0.9, {"index", "middle", "ring"})]).evidence[0]
    assert _label(unsupported) == "LEFT: UNKNOWN"

    unreliable_side = perception_from_hand_landmarker_result(
        SimpleNamespace(handedness=[], hand_landmarks=[_landmarks(extended=set())]),
        frame_index=3,
        capture_t_ms=7.0,
        available_t_ms=8.0,
        frame_width=10,
        frame_height=10,
    ).evidence[0]
    assert _label(unreliable_side) == "HAND?: FIST"


def test_live_perceiver_samples_available_time_only_after_landmarker(monkeypatch: pytest.MonkeyPatch) -> None:
    """The perception availability is completion time, never pre-inference."""
    completed = False
    classified = False

    class FakeLandmarker:
        def detect_for_video(self, _image: object, timestamp_ms: int) -> SimpleNamespace:
            nonlocal completed
            assert timestamp_ms == 1_000
            completed = True
            return SimpleNamespace(
                handedness=[[_category("Left", 0.9)]],
                hand_landmarks=[_landmarks(extended={"index"})],
            )

    def post_inference_clock() -> float:
        assert completed, "available_t_ms was sampled before detect_for_video() completed"
        assert classified, "available_t_ms was sampled before landmark classification completed"
        return 1.250

    original_classifier = gesture_perception_module.classify_landmarks

    def after_landmarker_classifier(landmarks):
        nonlocal classified
        assert completed
        classified = True
        return original_classifier(landmarks)

    perceiver = object.__new__(LandmarkGesturePerceiver)
    perceiver.config = GesturePerceptionConfig()
    perceiver._landmarker = FakeLandmarker()
    perceiver._frame_index = 0
    perceiver._last_capture_t_ms = None
    perceiver._last_video_timestamp_ms = None
    monkeypatch.setattr(gesture_perception_module.time, "monotonic", post_inference_clock)
    monkeypatch.setattr(gesture_perception_module, "classify_landmarks", after_landmarker_classifier)

    perceived = perceiver.process(np.zeros((10, 10, 3), dtype=np.uint8), 1_000.0)
    assert completed
    assert perceived.frame.available_t_ms == 1_250.0
    assert len(perceived.evidence) == 1
    assert perceived.evidence[0].gesture is GestureKind.POINT_ONE
    assert perceived.evidence[0].hand_key is HandKey.LEFT
