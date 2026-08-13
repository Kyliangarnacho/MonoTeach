"""Interactive four-point calibration of the planar MonoTeach workspace."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path
from typing import Sequence

import cv2
import numpy as np

from .camera_calibration import (
    CameraCalibration,
    load_camera_calibration,
    undistort_image_points,
    validate_calibration_resolution,
)
from .camera_stream import CameraConfig, CameraStream
from .workspace_calibration_io import (
    DEFAULT_WORKSPACE_CALIBRATION_DIRECTORY,
    save_workspace_calibration_json,
    workspace_calibration_output_path,
)
from .workspace_geometry import (
    WorkspaceCalibration,
    WorkspaceDefinition,
    compute_image_to_workspace_homography,
)


WINDOW_NAME = "MonoTeach Stage 2.3 Workspace Calibration (RAW, NOT MIRRORED)"
POINT_LABELS = ("P00", "P10", "P11", "P01")
POINT_INSTRUCTIONS = (
    "P00: click TOP-LEFT -> (0, 0)",
    "P10: click TOP-RIGHT -> (W, 0)",
    "P11: click BOTTOM-RIGHT -> (W, H)",
    "P01: click BOTTOM-LEFT -> (0, H)",
)


def _calibration_id() -> str:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    return f"workspace_2d_{timestamp}"


def _workspace_corners_mm(workspace: WorkspaceDefinition) -> np.ndarray:
    return np.array(
        [
            [0.0, 0.0],
            [workspace.width_mm, 0.0],
            [workspace.width_mm, workspace.height_mm],
            [0.0, workspace.height_mm],
        ],
        dtype=np.float64,
    )


def _create_workspace_calibration(
    undistorted_image_points: np.ndarray,
    workspace: WorkspaceDefinition,
    camera_calibration: CameraCalibration,
    frame_width: int,
    frame_height: int,
) -> WorkspaceCalibration:
    workspace_points_mm = _workspace_corners_mm(workspace)
    H_image_to_workspace = compute_image_to_workspace_homography(
        undistorted_image_points,
        workspace_points_mm,
    )
    return WorkspaceCalibration(
        schema_version="1.0",
        calibration_id=_calibration_id(),
        frame_width=frame_width,
        frame_height=frame_height,
        workspace_definition=workspace,
        image_points=undistorted_image_points,
        workspace_points_mm=workspace_points_mm,
        H_image_to_workspace=H_image_to_workspace,
        camera_calibration_reference=camera_calibration.source_path,
    )


def _draw_lines(
    frame: np.ndarray,
    lines: Sequence[str],
    *,
    start_y: int = 28,
    color: tuple[int, int, int] = (255, 255, 255),
) -> None:
    for index, line in enumerate(lines):
        cv2.putText(
            frame,
            line,
            (12, start_y + index * 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            color,
            2,
            cv2.LINE_AA,
        )


def _draw_selected_points(
    frame: np.ndarray,
    raw_clicked_points: Sequence[tuple[int, int]],
) -> None:
    for index, (u, v) in enumerate(raw_clicked_points):
        cv2.circle(frame, (u, v), 7, (0, 255, 255), -1, cv2.LINE_AA)
        cv2.putText(
            frame,
            POINT_LABELS[index],
            (u + 10, v - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--camera-calibration",
        required=True,
        help="Path to camera_params.npz/.yaml/.yml",
    )
    parser.add_argument("--width-mm", required=True, type=float)
    parser.add_argument("--height-mm", required=True, type=float)
    parser.add_argument("--camera-index", type=int, default=1)
    parser.add_argument(
        "--output-directory",
        default=str(DEFAULT_WORKSPACE_CALIBRATION_DIRECTORY),
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run non-mirrored four-point capture and save only when S is pressed."""
    arguments = _argument_parser().parse_args(argv)
    try:
        workspace = WorkspaceDefinition(arguments.width_mm, arguments.height_mm)
        camera_calibration = load_camera_calibration(arguments.camera_calibration)
    except ValueError as error:
        print(f"Workspace calibration setup failed: {error}")
        return 1

    stream = CameraStream(
        CameraConfig(
            index=arguments.camera_index,
            width=camera_calibration.frame_width,
            height=camera_calibration.frame_height,
        )
    )
    raw_clicked_points: list[tuple[int, int]] = []
    undistorted_image_points: list[tuple[float, float]] = []
    workspace_calibration: WorkspaceCalibration | None = None
    output_directory = Path(arguments.output_directory)

    def on_mouse(event: int, x: int, y: int, _flags: int, _data: object) -> None:
        if event != cv2.EVENT_LBUTTONDOWN or len(raw_clicked_points) >= 4:
            return
        raw_point = np.array([[float(x), float(y)]], dtype=np.float64)
        undistorted_point = undistort_image_points(
            raw_point,
            camera_calibration,
        )[0]
        raw_clicked_points.append((x, y))
        undistorted_image_points.append(
            (float(undistorted_point[0]), float(undistorted_point[1]))
        )
        print(
            f"{POINT_LABELS[len(raw_clicked_points) - 1]}: "
            f"raw=({x}, {y}), "
            f"undistorted=({undistorted_point[0]:.6f}, "
            f"{undistorted_point[1]:.6f})"
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
            f"index={arguments.camera_index}, "
            f"resolution={profile.width}x{profile.height}"
        )
        print("Click P00, P10, P11, P01 in order. R resets; Q exits; S saves.")
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

            if len(raw_clicked_points) == 4 and workspace_calibration is None:
                workspace_calibration = _create_workspace_calibration(
                    np.asarray(undistorted_image_points, dtype=np.float64),
                    workspace,
                    camera_calibration,
                    frame_width,
                    frame_height,
                )
                output_path = workspace_calibration_output_path(
                    workspace_calibration,
                    output_directory,
                )
                print(f"Workspace: {workspace.width_mm} x {workspace.height_mm} mm")
                print("H_image_to_workspace:")
                print(workspace_calibration.H_image_to_workspace)
                print(f"Press S to save: {output_path.resolve()}")

            _draw_selected_points(display_frame, raw_clicked_points)
            if workspace_calibration is None:
                next_index = len(raw_clicked_points)
                lines = (
                    "RAW preview (not mirrored) | R: reset | Q: quit without save",
                    POINT_INSTRUCTIONS[next_index],
                    f"Selected: {next_index}/4",
                )
            else:
                output_path = workspace_calibration_output_path(
                    workspace_calibration,
                    output_directory,
                )
                H = workspace_calibration.H_image_to_workspace
                lines = (
                    "Four points complete | S: save | R: reset | Q: quit without save",
                    f"Workspace: {workspace.width_mm:g} x {workspace.height_mm:g} mm",
                    f"H[0]: {H[0, 0]:.6g} {H[0, 1]:.6g} {H[0, 2]:.6g}",
                    f"H[1]: {H[1, 0]:.6g} {H[1, 1]:.6g} {H[1, 2]:.6g}",
                    f"H[2]: {H[2, 0]:.6g} {H[2, 1]:.6g} {H[2, 2]:.6g}",
                    f"Save: {output_path}",
                )
            _draw_lines(display_frame, lines)
            cv2.imshow(WINDOW_NAME, display_frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                print("Exited without saving workspace calibration.")
                return 0
            if key in (ord("r"), ord("R")):
                raw_clicked_points.clear()
                undistorted_image_points.clear()
                workspace_calibration = None
                print("Selection reset. Click P00, P10, P11, P01 again.")
            if key in (ord("s"), ord("S")) and workspace_calibration is not None:
                saved_path = save_workspace_calibration_json(
                    workspace_calibration,
                    output_directory,
                )
                print(f"Saved workspace calibration: {saved_path.resolve()}")
                return 0
    except (RuntimeError, TypeError, ValueError, cv2.error) as error:
        print(f"Workspace calibration failed: {error}")
        return 1
    finally:
        stream.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
