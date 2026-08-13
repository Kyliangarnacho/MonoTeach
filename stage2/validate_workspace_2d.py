"""Interactive independent five-point validation of a workspace calibration."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

import cv2
import numpy as np

from .camera_calibration import load_camera_calibration, validate_calibration_resolution
from .camera_stream import CameraConfig, CameraStream
from .workspace_calibration_io import load_workspace_calibration_json
from .workspace_validation import (
    DEFAULT_WORKSPACE_VALIDATION_DIRECTORY,
    ValidationReport,
    evaluate_workspace_points,
    save_workspace_validation_json,
)


WINDOW_NAME = "MonoTeach Stage 2.3 Workspace Validation (RAW, NOT MIRRORED)"
VALIDATION_LABELS = ("A", "B", "C", "D", "E")
TRUE_POINTS_MM = np.array(
    [[45.0, 45.0], [90.0, 45.0], [45.0, 135.0], [135.0, 135.0], [90.0, 180.0]],
    dtype=np.float64,
)


def _draw_lines(frame: np.ndarray, lines: Sequence[str]) -> None:
    for index, line in enumerate(lines):
        cv2.putText(
            frame,
            line,
            (12, 28 + index * 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )


def _draw_clicked_points(
    frame: np.ndarray,
    raw_image_points: Sequence[tuple[int, int]],
) -> None:
    for index, (u, v) in enumerate(raw_image_points):
        cv2.circle(frame, (u, v), 7, (0, 255, 255), -1, cv2.LINE_AA)
        cv2.putText(
            frame,
            VALIDATION_LABELS[index],
            (u + 10, v - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )


def _print_report(report: ValidationReport) -> None:
    print("Independent workspace validation (mm)")
    for label, point in zip(VALIDATION_LABELS, report.points, strict=True):
        print(
            f"{label}: true=({point.true_x_mm:.3f}, {point.true_y_mm:.3f}) "
            f"predicted=({point.predicted_x_mm:.3f}, {point.predicted_y_mm:.3f}) "
            f"dx={point.error_x_mm:.3f} dy={point.error_y_mm:.3f} "
            f"error={point.error_mm:.3f}"
        )
    print(f"mean error: {report.mean_error_mm:.3f} mm")
    print(f"RMS error: {report.rms_error_mm:.3f} mm")
    print(f"max error: {report.max_error_mm:.3f} mm")


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-calibration", required=True)
    parser.add_argument("--workspace-calibration", required=True)
    parser.add_argument("--camera-index", type=int, default=1)
    parser.add_argument(
        "--output-directory",
        default=str(DEFAULT_WORKSPACE_VALIDATION_DIRECTORY),
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Click five independent points; S saves a report after all five are measured."""
    arguments = _argument_parser().parse_args(argv)
    try:
        camera_calibration = load_camera_calibration(arguments.camera_calibration)
        workspace_calibration = load_workspace_calibration_json(
            arguments.workspace_calibration
        )
        validate_calibration_resolution(
            camera_calibration,
            workspace_calibration.frame_width,
            workspace_calibration.frame_height,
        )
    except ValueError as error:
        print(f"Workspace validation setup failed: {error}")
        return 1

    stream = CameraStream(
        CameraConfig(
            index=arguments.camera_index,
            width=camera_calibration.frame_width,
            height=camera_calibration.frame_height,
        )
    )
    raw_image_points: list[tuple[int, int]] = []
    report: ValidationReport | None = None

    def on_mouse(event: int, x: int, y: int, _flags: int, _data: object) -> None:
        if event != cv2.EVENT_LBUTTONDOWN or len(raw_image_points) >= len(TRUE_POINTS_MM):
            return
        raw_image_points.append((x, y))
        true_x_mm, true_y_mm = TRUE_POINTS_MM[len(raw_image_points) - 1]
        print(
            f"{VALIDATION_LABELS[len(raw_image_points) - 1]}: "
            f"true=({true_x_mm:g}, {true_y_mm:g}) mm raw=({x}, {y}) px"
        )

    try:
        profile = stream.open()
        validate_calibration_resolution(
            camera_calibration,
            profile.width,
            profile.height,
        )
        print(
            "Camera opened for non-mirrored raw preview: "
            f"index={arguments.camera_index}, resolution={profile.width}x{profile.height}"
        )
        print("Click A, B, C, D, E in order. R resets; Q exits; S saves report.")
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WINDOW_NAME, on_mouse)

        while True:
            frame, _timestamp_ms = stream.read()
            frame_height, frame_width = frame.shape[:2]
            validate_calibration_resolution(
                camera_calibration,
                frame_width,
                frame_height,
            )
            display_frame = frame.copy()

            if len(raw_image_points) == len(TRUE_POINTS_MM) and report is None:
                report = evaluate_workspace_points(
                    TRUE_POINTS_MM,
                    np.asarray(raw_image_points, dtype=np.float64),
                    camera_calibration,
                    workspace_calibration,
                )
                _print_report(report)

            _draw_clicked_points(display_frame, raw_image_points)
            if report is None:
                next_index = len(raw_image_points)
                x_mm, y_mm = TRUE_POINTS_MM[next_index]
                lines = (
                    "RAW preview (not mirrored) | R: reset | Q: quit",
                    f"Click {VALIDATION_LABELS[next_index]} -> ({x_mm:g}, {y_mm:g}) mm",
                    f"Selected: {next_index}/{len(TRUE_POINTS_MM)}",
                )
            else:
                lines = (
                    "Five points complete | S: save report | R: reset | Q: quit",
                    f"mean={report.mean_error_mm:.3f} mm  RMS={report.rms_error_mm:.3f} mm",
                    f"max={report.max_error_mm:.3f} mm",
                )
            _draw_lines(display_frame, lines)
            cv2.imshow(WINDOW_NAME, display_frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                return 0
            if key in (ord("r"), ord("R")):
                raw_image_points.clear()
                report = None
                print("Validation selection reset. Click A, B, C, D, E again.")
            if key in (ord("s"), ord("S")) and report is not None:
                saved_path = save_workspace_validation_json(
                    report,
                    np.asarray(raw_image_points, dtype=np.float64),
                    workspace_calibration,
                    Path(arguments.output_directory),
                )
                print(f"Saved workspace validation: {saved_path.resolve()}")
                return 0
    except (RuntimeError, TypeError, ValueError, cv2.error) as error:
        print(f"Workspace validation failed: {error}")
        return 1
    finally:
        stream.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
