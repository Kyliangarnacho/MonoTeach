"""Minimal live-preview entry point for :mod:`stage2.camera_stream`."""

from __future__ import annotations

import argparse
from typing import Sequence

import cv2

from .camera_stream import CameraConfig, CameraStream


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--camera-index",
        type=int,
        default=CameraConfig().index,
        help="OpenCV camera index (default: CameraConfig default)",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    arguments = _argument_parser().parse_args(argv)
    stream = CameraStream(CameraConfig(index=arguments.camera_index))
    mirror_preview = True
    try:
        profile = stream.open()
    except RuntimeError as error:
        print(f"Camera verification failed: {error}")
        return 1

    print(
        "Camera opened: "
        f"backend={profile.backend}, "
        f"resolution={profile.width}x{profile.height}, "
        f"fps={profile.fps:.3f}"
    )
    print("Press M to toggle mirror preview; press Q to exit.")

    try:
        while True:
            frame, _timestamp = stream.read()
            display_frame = cv2.flip(frame, 1) if mirror_preview else frame.copy()
            cv2.putText(
                display_frame,
                f"Mirror: {'ON' if mirror_preview else 'OFF'}",
                (12, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("MonoTeach Stage 2.1 Camera", display_frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                return 0
            if key in (ord("m"), ord("M")):
                mirror_preview = not mirror_preview
    except RuntimeError as error:
        print(f"Camera verification failed: {error}")
        return 1
    finally:
        stream.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
