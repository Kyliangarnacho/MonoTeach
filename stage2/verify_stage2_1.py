"""Stage 2.1 software verification without opening a physical camera."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from .fingertip import normalized_to_pixel
from .hand_observation import HandObservation
from .hand_tracker import DEFAULT_MODEL_PATH, HandTracker


def _check_model_asset() -> None:
    if not DEFAULT_MODEL_PATH.is_file():
        raise FileNotFoundError(f"Missing model asset: {DEFAULT_MODEL_PATH}")


def _check_hand_tracker_no_hand() -> None:
    blank_frame = np.zeros((120, 160, 3), dtype=np.uint8)
    with HandTracker() as tracker:
        observation = tracker.process(blank_frame, timestamp_ms=1_000.0)
    if observation != HandObservation(timestamp_ms=1_000.0, detected=False):
        raise AssertionError("Blank frame did not produce the expected no-hand observation.")


def _check_normalized_to_pixel() -> None:
    if normalized_to_pixel(1.0, 0.5, 640, 480) != (639, 240):
        raise AssertionError("Normalized-to-pixel conversion returned an unexpected result.")


def _check_observation_contract() -> None:
    observation = HandObservation(
        timestamp_ms=123.0,
        detected=True,
        handedness="Right",
        handedness_score=0.9,
        landmarks_norm=((0.1, 0.2, 0.0),),
        index_tip_norm=(0.1, 0.2),
        index_tip_px=(64, 96),
    )
    if not observation.detected or observation.index_tip_px != (64, 96):
        raise AssertionError("HandObservation fields were not retained.")


def main() -> int:
    """Run Stage 2.1 software checks and print PASS or FAIL for each check."""
    checks: tuple[tuple[str, Callable[[], None]], ...] = (
        ("model asset path", _check_model_asset),
        ("HandTracker initialization and no-hand frame", _check_hand_tracker_no_hand),
        ("normalized to pixel conversion", _check_normalized_to_pixel),
        ("HandObservation contract", _check_observation_contract),
    )
    failures = 0

    print("Stage 2.1 software verification (no physical camera)")
    for name, check in checks:
        try:
            check()
        except Exception as error:
            failures += 1
            print(f"FAIL: {name} — {error}")
        else:
            print(f"PASS: {name}")

    if failures:
        print(f"Stage 2.1 verification: FAIL ({failures} check(s) failed)")
        return 1

    print("Stage 2.1 verification: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
