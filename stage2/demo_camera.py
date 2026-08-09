"""Minimal live-preview entry point for :mod:`stage2.camera_stream`."""

from __future__ import annotations

import cv2

from .camera_stream import CameraConfig, CameraStream


def main() -> int:
    stream = CameraStream(CameraConfig())
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
    print("Press q in the preview window to exit.")

    try:
        while True:
            frame, _timestamp = stream.read()
            cv2.imshow("MonoTeach Stage 2.1 Camera", frame)
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                return 0
    except RuntimeError as error:
        print(f"Camera verification failed: {error}")
        return 1
    finally:
        stream.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
