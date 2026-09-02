"""Offline verify for Banter Task 1; it never opens a camera."""

from __future__ import annotations

import numpy as np

from stage2.hand_observation import HandObservation

from .contracts import GestureEvidence, GestureKind, GesturePerception, HandFrame, HandKey
from .gesture_events import GestureEventConfig, GestureEventStabilizer
from .gesture_perception import LandmarkGesturePerceiver


def _synthetic_perception(timestamp_ms: float) -> GesturePerception:
    hand = HandObservation(
        timestamp_ms=timestamp_ms,
        detected=True,
        handedness="Left",
        handedness_score=0.9,
        landmarks_norm=tuple((0.0, 0.0, 0.0) for _ in range(21)),
        index_tip_norm=(0.0, 0.0),
        index_tip_px=(0, 0),
    )
    evidence = GestureEvidence(
        frame_index=int(timestamp_ms),
        capture_t_ms=timestamp_ms,
        available_t_ms=timestamp_ms + 2.0,
        hand=hand,
        hand_key=HandKey.LEFT,
        gesture=GestureKind.POINT_ONE,
        recognizer_score=0.9,
    )
    return GesturePerception(
        frame=HandFrame(int(timestamp_ms), timestamp_ms, timestamp_ms + 2.0, (hand,)),
        evidence=(evidence,),
    )


def main() -> int:
    """Check the packaged model can run and a synthetic pose emits once."""
    print("[1/2] Loading HandLandmarker and processing one blank frame...")
    with LandmarkGesturePerceiver() as perceiver:
        blank = np.zeros((480, 640, 3), dtype=np.uint8)
        perception = perceiver.process(blank, 1_000.0)
    assert perception.frame.frame_index == 0
    assert perception.frame.hands == ()

    print("[2/2] Checking offline dwell/release event semantics...")
    stabilizer = GestureEventStabilizer(
        GestureEventConfig(dwell_ms=100.0, release_ms=50.0, dropout_grace_ms=30.0)
    )
    assert stabilizer.update(_synthetic_perception(0.0)) == ()
    events = stabilizer.update(_synthetic_perception(100.0))
    assert len(events) == 1
    assert stabilizer.update(_synthetic_perception(200.0)) == ()
    print("PASS: model path, 0-hand frame, and one-shot gesture event verified.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
