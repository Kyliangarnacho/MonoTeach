"""Hardware-free synthetic tests for Stage 2.3 workspace geometry."""

import cv2
import json
import numpy as np
import pytest

from stage2.camera_calibration import (
    CameraCalibration,
    load_camera_calibration,
    undistort_image_points,
    validate_calibration_resolution,
)
from stage2.workspace_geometry import (
    WORKSPACE_COORDINATE_FRAME,
    WorkspaceCalibration,
    WorkspaceDefinition,
    WorkspacePoint,
    compute_image_to_workspace_homography,
    image_points_to_workspace,
    inside_workspace,
)
from stage2.workspace_calibration_io import (
    IMAGE_POINTS_COORDINATE_SPACE,
    load_workspace_calibration_json,
    save_workspace_calibration_json,
)
from stage2.workspace_validation import (
    evaluate_workspace_points,
    save_workspace_validation_json,
)
from stage2.workspace_trajectory import (
    WorkspaceTrajectory2D,
    WorkspaceTrajectoryMetadata,
    WorkspaceTrajectorySample,
    trajectory_to_workspace,
)
from stage2.workspace_trajectory_io import (
    load_workspace_trajectory_json,
    save_workspace_trajectory_json,
)
from stage2.trajectory_playback import build_playback_timeline
from stage2.trajectory import Trajectory2D, TrajectoryMetadata, TrajectorySample
from stage2.trajectory_filter import EMAConfig
from stage2.trajectory_quality import QualityGateConfig


IMAGE_CORNERS = np.array(
    [[100.0, 50.0], [500.0, 50.0], [500.0, 250.0], [100.0, 250.0]],
    dtype=np.float64,
)
WORKSPACE_CORNERS_MM = np.array(
    [[0.0, 0.0], [400.0, 0.0], [400.0, 200.0], [0.0, 200.0]],
    dtype=np.float64,
)
SYNTHETIC_K = np.array(
    [[800.0, 0.0, 320.0], [0.0, 810.0, 240.0], [0.0, 0.0, 1.0]],
    dtype=np.float64,
)
ZERO_D = np.zeros((1, 5), dtype=np.float64)


def camera_calibration(D=ZERO_D):
    return CameraCalibration(
        K=SYNTHETIC_K,
        D=D,
        frame_width=640,
        frame_height=480,
        source_path="synthetic",
        source_format="test",
    )


def write_opencv_yaml(path, K=SYNTHETIC_K, D=ZERO_D, width=640, height=480):
    file_storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    assert file_storage.isOpened()
    file_storage.write("image_width", width)
    file_storage.write("image_height", height)
    file_storage.write("camera_matrix", K)
    file_storage.write("dist_coeffs", D)
    file_storage.release()


@pytest.fixture
def H_image_to_workspace():
    return compute_image_to_workspace_homography(
        IMAGE_CORNERS,
        WORKSPACE_CORNERS_MM,
    )


def test_known_rectangle_maps_all_four_corners(H_image_to_workspace):
    mapped = image_points_to_workspace(IMAGE_CORNERS, H_image_to_workspace)

    np.testing.assert_allclose(mapped, WORKSPACE_CORNERS_MM, atol=1e-9)


def test_center_point_maps_to_workspace_center(H_image_to_workspace):
    mapped = image_points_to_workspace([[300.0, 150.0]], H_image_to_workspace)

    np.testing.assert_allclose(mapped, [[200.0, 100.0]], atol=1e-9)


def test_internal_point_maps_to_expected_millimetres(H_image_to_workspace):
    mapped = image_points_to_workspace([[200.0, 100.0]], H_image_to_workspace)

    np.testing.assert_allclose(mapped, [[100.0, 50.0]], atol=1e-9)


def test_homography_direction_is_image_to_workspace(H_image_to_workspace):
    image_point = np.array([[420.0, 210.0]])

    mapped = image_points_to_workspace(image_point, H_image_to_workspace)

    np.testing.assert_allclose(mapped, [[320.0, 160.0]], atol=1e-9)
    assert not np.allclose(mapped, image_point)


@pytest.mark.parametrize(
    ("X_mm", "Y_mm", "expected"),
    [
        (200.0, 100.0, True),
        (0.0, 0.0, True),
        (400.0, 200.0, True),
        (-0.01, 100.0, False),
        (200.0, 200.01, False),
    ],
)
def test_inside_workspace_includes_edges_and_excludes_outside(X_mm, Y_mm, expected):
    workspace = WorkspaceDefinition(width_mm=400.0, height_mm=200.0)

    assert inside_workspace(X_mm, Y_mm, workspace) is expected


@pytest.mark.parametrize(
    ("width_mm", "height_mm"),
    [(0.0, 200.0), (-1.0, 200.0), (400.0, 0.0), (400.0, np.inf)],
)
def test_invalid_workspace_dimensions_are_rejected(width_mm, height_mm):
    with pytest.raises(ValueError, match="finite and positive"):
        WorkspaceDefinition(width_mm=width_mm, height_mm=height_mm)


@pytest.mark.parametrize(
    ("image_points", "workspace_points_mm", "message"),
    [
        (np.zeros((4, 3)), WORKSPACE_CORNERS_MM, r"image_points.*\(N, 2\)"),
        (IMAGE_CORNERS, np.zeros((3, 2)), "same number"),
        (IMAGE_CORNERS[:3], WORKSPACE_CORNERS_MM[:3], "at least 4"),
    ],
)
def test_compute_homography_rejects_invalid_point_dimensions(
    image_points,
    workspace_points_mm,
    message,
):
    with pytest.raises(ValueError, match=message):
        compute_image_to_workspace_homography(image_points, workspace_points_mm)


def test_transform_rejects_invalid_homography_shape():
    with pytest.raises(ValueError, match=r"H_image_to_workspace.*\(3, 3\)"):
        image_points_to_workspace([[100.0, 50.0]], np.eye(2))


def test_geometry_functions_do_not_modify_inputs():
    image_points = IMAGE_CORNERS.copy()
    workspace_points_mm = WORKSPACE_CORNERS_MM.copy()
    image_before = image_points.copy()
    workspace_before = workspace_points_mm.copy()

    H_image_to_workspace = compute_image_to_workspace_homography(
        image_points,
        workspace_points_mm,
    )
    homography_before = H_image_to_workspace.copy()
    mapped = image_points_to_workspace(image_points, H_image_to_workspace)

    np.testing.assert_array_equal(image_points, image_before)
    np.testing.assert_array_equal(workspace_points_mm, workspace_before)
    np.testing.assert_array_equal(H_image_to_workspace, homography_before)
    assert not np.shares_memory(mapped, image_points)


def test_workspace_data_contracts_retain_explicit_direction():
    H_image_to_workspace = compute_image_to_workspace_homography(
        IMAGE_CORNERS,
        WORKSPACE_CORNERS_MM,
    )
    calibration = WorkspaceCalibration(
        schema_version="1.0",
        calibration_id="synthetic-rectangle",
        frame_width=600,
        frame_height=300,
        workspace_definition=WorkspaceDefinition(width_mm=400.0, height_mm=200.0),
        image_points=IMAGE_CORNERS,
        workspace_points_mm=WORKSPACE_CORNERS_MM,
        H_image_to_workspace=H_image_to_workspace,
        camera_calibration_reference="camera_params.npz",
    )
    point = WorkspacePoint(x_mm=100.0, y_mm=50.0, inside_workspace=True)
    definition = WorkspaceDefinition(width_mm=400.0, height_mm=200.0)

    assert definition.coordinate_frame == WORKSPACE_COORDINATE_FRAME
    assert calibration.H_image_to_workspace.shape == (3, 3)
    assert calibration.H_image_to_workspace.flags.writeable is False
    assert point.inside_workspace is True


def test_load_synthetic_npz_calibration(tmp_path):
    path = tmp_path / "camera_params.npz"
    np.savez(
        path,
        camera_matrix=SYNTHETIC_K,
        dist_coeffs=ZERO_D,
        image_width=640,
        image_height=480,
    )

    calibration = load_camera_calibration(path)

    assert calibration.source_path == str(path.resolve())
    assert calibration.source_format == "npz"
    assert (calibration.frame_width, calibration.frame_height) == (640, 480)
    np.testing.assert_array_equal(calibration.K, SYNTHETIC_K)
    np.testing.assert_array_equal(calibration.D, ZERO_D)


def test_load_synthetic_opencv_yaml_calibration(tmp_path):
    path = tmp_path / "camera_params.yaml"
    write_opencv_yaml(path)

    calibration = load_camera_calibration(path)

    assert calibration.source_path == str(path.resolve())
    assert calibration.source_format == "opencv_yaml"
    assert (calibration.frame_width, calibration.frame_height) == (640, 480)
    np.testing.assert_array_equal(calibration.K, SYNTHETIC_K)
    np.testing.assert_array_equal(calibration.D, ZERO_D)


@pytest.mark.parametrize(
    "D",
    [np.zeros(5), np.zeros((1, 5)), np.zeros((5, 1))],
)
def test_calibration_normalizes_K_and_D_shapes_to_readonly_arrays(D):
    input_K = SYNTHETIC_K.copy()
    input_D = D.copy()

    calibration = CameraCalibration(
        K=input_K,
        D=input_D,
        frame_width=640,
        frame_height=480,
        source_path="synthetic",
        source_format="test",
    )
    input_K[0, 0] = -1.0
    input_D[...] = 1.0

    assert calibration.K.shape == (3, 3)
    assert calibration.D.shape == (1, 5)
    assert calibration.K.dtype == np.float64
    assert calibration.D.dtype == np.float64
    assert calibration.K.flags.writeable is False
    assert calibration.D.flags.writeable is False
    assert calibration.K[0, 0] == 800.0
    np.testing.assert_array_equal(calibration.D, ZERO_D)


def test_validate_calibration_resolution_accepts_exact_match():
    validate_calibration_resolution(camera_calibration(), 640, 480)


def test_validate_calibration_resolution_rejects_mismatch_without_scaling_K():
    calibration = camera_calibration()
    K_before = calibration.K.copy()

    with pytest.raises(ValueError, match=r"frame=1280x720.*calibration=640x480"):
        validate_calibration_resolution(calibration, 1280, 720)

    np.testing.assert_array_equal(calibration.K, K_before)


def test_zero_distortion_preserves_multiple_pixel_points():
    distorted_points = np.array(
        [[0.0, 0.0], [320.0, 240.0], [639.0, 479.0], [123.5, 321.25]],
        dtype=np.float64,
    )

    undistorted_points = undistort_image_points(
        distorted_points,
        camera_calibration(),
    )

    assert undistorted_points.shape == (4, 2)
    assert undistorted_points.dtype == np.float64
    np.testing.assert_allclose(undistorted_points, distorted_points, atol=1e-12)


def test_nonzero_distortion_recovers_original_undistorted_pixels():
    D = np.array([[0.12, -0.04, 0.001, -0.002, 0.01]], dtype=np.float64)
    calibration = camera_calibration(D)
    original_pixels = np.array(
        [[100.0, 80.0], [320.0, 240.0], [540.0, 390.0], [250.0, 330.0]],
        dtype=np.float64,
    )
    normalized = np.column_stack(
        (
            (original_pixels[:, 0] - SYNTHETIC_K[0, 2]) / SYNTHETIC_K[0, 0],
            (original_pixels[:, 1] - SYNTHETIC_K[1, 2]) / SYNTHETIC_K[1, 1],
            np.ones(len(original_pixels)),
        )
    )
    distorted_pixels, _ = cv2.projectPoints(
        normalized,
        np.zeros(3),
        np.zeros(3),
        SYNTHETIC_K,
        D,
    )
    distorted_pixels = distorted_pixels.reshape(-1, 2)

    recovered_pixels = undistort_image_points(distorted_pixels, calibration)

    np.testing.assert_allclose(recovered_pixels, original_pixels, atol=1e-5)


def test_undistort_image_points_does_not_modify_input():
    distorted_points = np.array([[50.0, 60.0], [600.0, 400.0]], dtype=np.float64)
    points_before = distorted_points.copy()

    result = undistort_image_points(distorted_points, camera_calibration())

    np.testing.assert_array_equal(distorted_points, points_before)
    assert not np.shares_memory(result, distorted_points)


@pytest.mark.parametrize(
    "image_points",
    [np.zeros((2, 3)), np.zeros(2), np.array([[np.nan, 1.0]])],
)
def test_undistort_rejects_malformed_points(image_points):
    with pytest.raises(ValueError, match="image_points"):
        undistort_image_points(image_points, camera_calibration())


@pytest.mark.parametrize(
    ("K", "D", "message"),
    [
        (np.eye(2), ZERO_D, r"K.*\(3, 3\)"),
        (SYNTHETIC_K, np.zeros((2, 2)), r"D.*shape"),
        (SYNTHETIC_K, np.zeros(6), r"D.*coefficient"),
    ],
)
def test_malformed_calibration_shapes_are_rejected(K, D, message):
    with pytest.raises(ValueError, match=message):
        CameraCalibration(
            K=K,
            D=D,
            frame_width=640,
            frame_height=480,
            source_path="synthetic",
            source_format="test",
        )


def test_npz_missing_required_field_is_rejected(tmp_path):
    path = tmp_path / "missing_distortion.npz"
    np.savez(
        path,
        camera_matrix=SYNTHETIC_K,
        image_width=640,
        image_height=480,
    )

    with pytest.raises(ValueError, match="missing required field.*dist_coeffs"):
        load_camera_calibration(path)


def test_corrupt_yaml_is_rejected(tmp_path):
    path = tmp_path / "corrupt.yaml"
    path.write_text("this is not OpenCV YAML: [", encoding="utf-8")

    with pytest.raises(ValueError, match="OpenCV YAML"):
        load_camera_calibration(path)


def synthetic_workspace_calibration():
    workspace = WorkspaceDefinition(width_mm=400.0, height_mm=200.0)
    H_image_to_workspace = compute_image_to_workspace_homography(
        IMAGE_CORNERS,
        WORKSPACE_CORNERS_MM,
    )
    return WorkspaceCalibration(
        schema_version="1.0",
        calibration_id="synthetic-workspace",
        frame_width=600,
        frame_height=300,
        workspace_definition=workspace,
        image_points=IMAGE_CORNERS,
        workspace_points_mm=WORKSPACE_CORNERS_MM,
        H_image_to_workspace=H_image_to_workspace,
        camera_calibration_reference="fixtures/camera_params.npz",
    )


def test_workspace_calibration_json_round_trip(tmp_path):
    calibration = synthetic_workspace_calibration()

    path = save_workspace_calibration_json(calibration, tmp_path / "nested")
    loaded = load_workspace_calibration_json(path)

    assert loaded.schema_version == calibration.schema_version
    assert loaded.calibration_id == calibration.calibration_id
    assert (loaded.frame_width, loaded.frame_height) == (600, 300)
    np.testing.assert_array_equal(loaded.image_points, calibration.image_points)
    np.testing.assert_array_equal(
        loaded.workspace_points_mm,
        calibration.workspace_points_mm,
    )
    np.testing.assert_array_equal(
        loaded.H_image_to_workspace,
        calibration.H_image_to_workspace,
    )


def test_workspace_artifact_preserves_dimensions_reference_and_point_semantics(tmp_path):
    calibration = synthetic_workspace_calibration()

    path = save_workspace_calibration_json(calibration, tmp_path)
    payload = json.loads(path.read_text(encoding="utf-8"))
    loaded = load_workspace_calibration_json(path)

    assert payload["image_points_coordinate_space"] == IMAGE_POINTS_COORDINATE_SPACE
    assert payload["workspace"] == {
        "width_mm": 400.0,
        "height_mm": 200.0,
        "coordinate_frame": WORKSPACE_COORDINATE_FRAME,
    }
    assert loaded.workspace_definition.width_mm == 400.0
    assert loaded.workspace_definition.height_mm == 200.0
    assert loaded.camera_calibration_reference == "fixtures/camera_params.npz"


def test_workspace_calibration_save_creates_output_directory(tmp_path):
    output_directory = tmp_path / "missing" / "calibrations"
    assert not output_directory.exists()

    path = save_workspace_calibration_json(
        synthetic_workspace_calibration(),
        output_directory,
    )

    assert output_directory.is_dir()
    assert path.is_file()


def test_workspace_calibration_save_does_not_modify_inputs(tmp_path):
    calibration = synthetic_workspace_calibration()
    image_before = calibration.image_points.copy()
    workspace_before = calibration.workspace_points_mm.copy()
    homography_before = calibration.H_image_to_workspace.copy()

    save_workspace_calibration_json(calibration, tmp_path)

    np.testing.assert_array_equal(calibration.image_points, image_before)
    np.testing.assert_array_equal(calibration.workspace_points_mm, workspace_before)
    np.testing.assert_array_equal(calibration.H_image_to_workspace, homography_before)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda payload: payload.pop("H_image_to_workspace"), "missing required"),
        (
            lambda payload: payload.__setitem__(
                "image_points_coordinate_space",
                "raw_distorted_pixel",
            ),
            "image_points_coordinate_space",
        ),
        (
            lambda payload: payload.__setitem__("H_image_to_workspace", np.eye(2).tolist()),
            r"H_image_to_workspace.*\(3, 3\)",
        ),
        (
            lambda payload: payload["workspace"].__setitem__("width_mm", 0.0),
            "width_mm",
        ),
    ],
)
def test_malformed_workspace_artifact_is_rejected(tmp_path, mutation, message):
    valid_path = save_workspace_calibration_json(
        synthetic_workspace_calibration(),
        tmp_path,
    )
    payload = json.loads(valid_path.read_text(encoding="utf-8"))
    mutation(payload)
    malformed_path = tmp_path / "malformed.json"
    malformed_path.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValueError, match=message):
        load_workspace_calibration_json(malformed_path)


def validation_workspace_calibration(H_image_to_workspace=np.eye(3)):
    return WorkspaceCalibration(
        schema_version="1.0",
        calibration_id="validation-workspace",
        frame_width=640,
        frame_height=480,
        workspace_definition=WorkspaceDefinition(width_mm=400.0, height_mm=300.0),
        image_points=IMAGE_CORNERS,
        workspace_points_mm=WORKSPACE_CORNERS_MM,
        H_image_to_workspace=H_image_to_workspace,
        camera_calibration_reference="fixtures/camera_params.npz",
    )


def test_workspace_validation_perfect_mapping_has_zero_error():
    true_points_mm = np.array([[50.0, 50.0], [140.0, 240.0]], dtype=np.float64)

    report = evaluate_workspace_points(
        true_points_mm,
        true_points_mm,
        camera_calibration(),
        validation_workspace_calibration(),
    )

    assert len(report.points) == 2
    assert report.mean_error_mm == pytest.approx(0.0)
    assert report.rms_error_mm == pytest.approx(0.0)
    assert report.max_error_mm == pytest.approx(0.0)


def test_workspace_validation_known_offset_and_rms():
    true_points_mm = np.array([[10.0, 20.0], [30.0, 40.0]], dtype=np.float64)
    raw_image_points = np.array([[13.0, 24.0], [30.0, 45.0]], dtype=np.float64)

    report = evaluate_workspace_points(
        true_points_mm,
        raw_image_points,
        camera_calibration(),
        validation_workspace_calibration(),
    )

    first, second = report.points
    assert (first.error_x_mm, first.error_y_mm, first.error_mm) == pytest.approx(
        (3.0, 4.0, 5.0)
    )
    assert (second.error_x_mm, second.error_y_mm, second.error_mm) == pytest.approx(
        (0.0, 5.0, 5.0)
    )
    assert report.mean_error_mm == pytest.approx(5.0)
    assert report.rms_error_mm == pytest.approx(5.0)
    assert report.max_error_mm == pytest.approx(5.0)


def test_workspace_validation_undistorts_raw_pixels_before_homography():
    D = np.array([[0.12, -0.04, 0.001, -0.002, 0.01]], dtype=np.float64)
    calibration = camera_calibration(D)
    true_points_mm = np.array([[100.0, 80.0], [540.0, 390.0]], dtype=np.float64)
    normalized = np.column_stack(
        (
            (true_points_mm[:, 0] - SYNTHETIC_K[0, 2]) / SYNTHETIC_K[0, 0],
            (true_points_mm[:, 1] - SYNTHETIC_K[1, 2]) / SYNTHETIC_K[1, 1],
            np.ones(len(true_points_mm)),
        )
    )
    raw_image_points, _ = cv2.projectPoints(
        normalized,
        np.zeros(3),
        np.zeros(3),
        SYNTHETIC_K,
        D,
    )

    report = evaluate_workspace_points(
        true_points_mm,
        raw_image_points.reshape(-1, 2),
        calibration,
        validation_workspace_calibration(),
    )

    assert report.max_error_mm < 1e-5


def test_workspace_validation_never_refits_homography(monkeypatch):
    def fail_if_called(*_args, **_kwargs):
        raise AssertionError("Validation must not compute a new homography.")

    monkeypatch.setattr(cv2, "findHomography", fail_if_called)

    report = evaluate_workspace_points(
        [[20.0, 30.0]],
        [[20.0, 30.0]],
        camera_calibration(),
        validation_workspace_calibration(),
    )

    assert report.max_error_mm == pytest.approx(0.0)


def test_workspace_validation_does_not_modify_inputs():
    true_points_mm = np.array([[20.0, 30.0], [50.0, 60.0]], dtype=np.float64)
    raw_image_points = true_points_mm.copy()
    true_before = true_points_mm.copy()
    raw_before = raw_image_points.copy()

    evaluate_workspace_points(
        true_points_mm,
        raw_image_points,
        camera_calibration(),
        validation_workspace_calibration(),
    )

    np.testing.assert_array_equal(true_points_mm, true_before)
    np.testing.assert_array_equal(raw_image_points, raw_before)


@pytest.mark.parametrize(
    ("true_points_mm", "raw_image_points", "message"),
    [
        (np.zeros((2, 2)), np.zeros((3, 2)), "same number"),
        (np.zeros((2, 3)), np.zeros((2, 2)), r"true_points_mm.*\(N, 2\)"),
        (np.empty((0, 2)), np.empty((0, 2)), "at least one"),
    ],
)
def test_workspace_validation_rejects_invalid_or_empty_point_sets(
    true_points_mm,
    raw_image_points,
    message,
):
    with pytest.raises(ValueError, match=message):
        evaluate_workspace_points(
            true_points_mm,
            raw_image_points,
            camera_calibration(),
            validation_workspace_calibration(),
        )


def test_workspace_validation_rejects_calibration_resolution_mismatch():
    workspace_calibration = validation_workspace_calibration()
    mismatched_camera = CameraCalibration(
        K=SYNTHETIC_K,
        D=ZERO_D,
        frame_width=1280,
        frame_height=720,
        source_path="synthetic",
        source_format="test",
    )

    with pytest.raises(ValueError, match="resolutions must match"):
        evaluate_workspace_points(
            [[20.0, 30.0]],
            [[20.0, 30.0]],
            mismatched_camera,
            workspace_calibration,
        )


def test_workspace_validation_report_json_preserves_raw_points_and_statistics(tmp_path):
    raw_image_points = np.array([[20.0, 30.0]], dtype=np.float64)
    workspace_calibration = validation_workspace_calibration()
    report = evaluate_workspace_points(
        raw_image_points,
        raw_image_points,
        camera_calibration(),
        workspace_calibration,
    )

    path = save_workspace_validation_json(
        report,
        raw_image_points,
        workspace_calibration,
        tmp_path / "nested",
    )
    payload = json.loads(path.read_text(encoding="utf-8"))

    assert path.parent.is_dir()
    assert payload["workspace_calibration_id"] == "validation-workspace"
    assert payload["camera_calibration_reference"] == "fixtures/camera_params.npz"
    assert payload["raw_image_points"] == [[20.0, 30.0]]
    assert payload["mean_error_mm"] == pytest.approx(0.0)


def synthetic_trajectory(samples):
    return Trajectory2D(
        metadata=TrajectoryMetadata(
            schema_version="1.0",
            trajectory_id="source-trajectory",
            frame_width=640,
            frame_height=480,
            camera_index=0,
            recording_start_timestamp_ms=1_000.0,
        ),
        raw_samples=tuple(samples),
    )


def workspace_trajectory_calibration(H_image_to_workspace=np.eye(3)):
    return WorkspaceCalibration(
        schema_version="1.0",
        calibration_id="workspace-calibration",
        frame_width=640,
        frame_height=480,
        workspace_definition=WorkspaceDefinition(width_mm=400.0, height_mm=300.0),
        image_points=IMAGE_CORNERS,
        workspace_points_mm=WORKSPACE_CORNERS_MM,
        H_image_to_workspace=H_image_to_workspace,
        camera_calibration_reference="fixtures/camera_params.npz",
    )


def valid_trajectory_sample(t_ms, x_norm, y_norm, u, v):
    return TrajectorySample(
        t_ms=t_ms,
        valid=True,
        x_norm=x_norm,
        y_norm=y_norm,
        u=u,
        v=v,
    )


def test_trajectory_to_workspace_maps_known_samples_and_preserves_timestamps():
    trajectory = synthetic_trajectory(
        [
            valid_trajectory_sample(0.0, 50 / 639, 60 / 479, 50, 60),
            valid_trajectory_sample(1_000.0, 100 / 639, 120 / 479, 100, 120),
        ]
    )

    result = trajectory_to_workspace(
        trajectory,
        camera_calibration(),
        workspace_trajectory_calibration(),
        quality_config=QualityGateConfig(max_normalized_speed=1.0),
        ema_config=EMAConfig(alpha=1.0),
    )

    assert isinstance(result, WorkspaceTrajectory2D)
    assert tuple(sample.t_ms for sample in result.samples) == (0.0, 1_000.0)
    np.testing.assert_allclose(
        [(sample.x_mm, sample.y_mm) for sample in result.samples],
        [[50.0, 60.0], [100.0, 120.0]],
    )
    assert result.metadata.source_trajectory_id == "source-trajectory"
    assert result.metadata.workspace_calibration_id == "workspace-calibration"
    assert result.metadata.coordinate_frame == WORKSPACE_COORDINATE_FRAME
    assert (result.metadata.width_mm, result.metadata.height_mm) == (400.0, 300.0)


def test_trajectory_to_workspace_preserves_invalid_gap_without_coordinates():
    trajectory = synthetic_trajectory(
        [
            valid_trajectory_sample(0.0, 0.1, 0.1, 64, 48),
            TrajectorySample(100.0, False, None, None, None, None, "no_hand"),
            valid_trajectory_sample(200.0, 0.2, 0.2, 128, 96),
        ]
    )

    result = trajectory_to_workspace(
        trajectory,
        camera_calibration(),
        workspace_trajectory_calibration(),
        quality_config=QualityGateConfig(max_normalized_speed=5.0),
        ema_config=EMAConfig(alpha=1.0),
    )

    gap = result.samples[1]
    assert tuple(sample.t_ms for sample in result.samples) == (0.0, 100.0, 200.0)
    assert (gap.valid, gap.x_mm, gap.y_mm, gap.inside_workspace, gap.invalid_reason) == (
        False,
        None,
        None,
        False,
        "no_hand",
    )


def test_trajectory_to_workspace_preserves_quality_rejection_as_invalid():
    trajectory = synthetic_trajectory(
        [
            valid_trajectory_sample(0.0, 0.0, 0.0, 0, 0),
            valid_trajectory_sample(10.0, 0.9, 0.9, 575, 431),
        ]
    )

    result = trajectory_to_workspace(
        trajectory,
        camera_calibration(),
        workspace_trajectory_calibration(),
        quality_config=QualityGateConfig(max_normalized_speed=1.0),
        ema_config=EMAConfig(alpha=1.0),
    )

    rejected = result.samples[1]
    assert rejected.t_ms == 10.0
    assert rejected.valid is False
    assert rejected.invalid_reason == "speed_gate"
    assert (rejected.x_mm, rejected.y_mm, rejected.inside_workspace) == (None, None, False)


def test_trajectory_to_workspace_maps_ema_filtered_pixels():
    trajectory = synthetic_trajectory(
        [
            valid_trajectory_sample(0.0, 0.1, 0.1, 64, 48),
            valid_trajectory_sample(1_000.0, 0.3, 0.1, 192, 48),
        ]
    )

    result = trajectory_to_workspace(
        trajectory,
        camera_calibration(),
        workspace_trajectory_calibration(),
        quality_config=QualityGateConfig(max_normalized_speed=1.0),
        ema_config=EMAConfig(alpha=0.5),
    )

    assert (result.samples[0].x_mm, result.samples[0].y_mm) == pytest.approx((64, 48))
    assert (result.samples[1].x_mm, result.samples[1].y_mm) == pytest.approx((128, 48))


def test_trajectory_to_workspace_keeps_outside_points_unclamped():
    trajectory = synthetic_trajectory(
        [
            valid_trajectory_sample(
                0.0,
                450 / 639,
                100 / 479,
                450,
                100,
            )
        ]
    )
    workspace = WorkspaceCalibration(
        schema_version="1.0",
        calibration_id="small-workspace",
        frame_width=640,
        frame_height=480,
        workspace_definition=WorkspaceDefinition(width_mm=400.0, height_mm=300.0),
        image_points=IMAGE_CORNERS,
        workspace_points_mm=WORKSPACE_CORNERS_MM,
        H_image_to_workspace=np.eye(3),
        camera_calibration_reference="fixtures/camera_params.npz",
    )

    result = trajectory_to_workspace(
        trajectory,
        camera_calibration(),
        workspace,
        quality_config=QualityGateConfig(max_normalized_speed=1.0),
        ema_config=EMAConfig(alpha=1.0),
    )

    point = result.samples[0]
    assert point.valid is True
    assert point.inside_workspace is False
    assert (point.x_mm, point.y_mm) == pytest.approx((450.0, 100.0))


def test_trajectory_to_workspace_does_not_modify_source_and_output_is_immutable():
    raw_samples = (
        valid_trajectory_sample(0.0, 0.1, 0.1, 64, 48),
        valid_trajectory_sample(1_000.0, 0.2, 0.2, 128, 96),
    )
    trajectory = synthetic_trajectory(raw_samples)
    raw_before = trajectory.raw_samples

    result = trajectory_to_workspace(
        trajectory,
        camera_calibration(),
        workspace_trajectory_calibration(),
        quality_config=QualityGateConfig(max_normalized_speed=1.0),
        ema_config=EMAConfig(alpha=1.0),
    )

    assert trajectory.raw_samples is raw_before
    assert trajectory.raw_samples == raw_samples
    assert isinstance(result.samples, tuple)
    with pytest.raises(Exception):
        result.samples += (result.samples[0],)


def test_trajectory_to_workspace_rejects_resolution_mismatch():
    trajectory = synthetic_trajectory([valid_trajectory_sample(0.0, 0.1, 0.1, 64, 48)])
    mismatched_workspace = WorkspaceCalibration(
        schema_version="1.0",
        calibration_id="mismatch",
        frame_width=1280,
        frame_height=720,
        workspace_definition=WorkspaceDefinition(width_mm=400.0, height_mm=300.0),
        image_points=IMAGE_CORNERS,
        workspace_points_mm=WORKSPACE_CORNERS_MM,
        H_image_to_workspace=np.eye(3),
        camera_calibration_reference="fixtures/camera_params.npz",
    )

    with pytest.raises(ValueError, match="resolutions must match"):
        trajectory_to_workspace(
            trajectory,
            camera_calibration(),
            mismatched_workspace,
            quality_config=QualityGateConfig(max_normalized_speed=1.0),
            ema_config=EMAConfig(alpha=1.0),
        )


def synthetic_workspace_trajectory():
    return WorkspaceTrajectory2D(
        metadata=WorkspaceTrajectoryMetadata(
            source_trajectory_id="source-trajectory",
            workspace_calibration_id="workspace-calibration",
            coordinate_frame=WORKSPACE_COORDINATE_FRAME,
            width_mm=190.0,
            height_mm=290.0,
        ),
        samples=(
            WorkspaceTrajectorySample(0.0, True, 10.0, 20.0, True),
            WorkspaceTrajectorySample(100.0, False, None, None, False, "no_hand"),
            WorkspaceTrajectorySample(250.0, True, 220.0, 300.0, False),
        ),
    )


def test_workspace_trajectory_json_round_trip_preserves_metadata_samples_and_gaps(tmp_path):
    trajectory = synthetic_workspace_trajectory()
    samples_before = trajectory.samples

    path = save_workspace_trajectory_json(trajectory, tmp_path / "nested")
    loaded = load_workspace_trajectory_json(path)

    assert path.parent.is_dir()
    assert loaded.metadata == trajectory.metadata
    assert loaded.samples == trajectory.samples
    assert tuple(sample.t_ms for sample in loaded.samples) == (0.0, 100.0, 250.0)
    assert loaded.samples[1].invalid_reason == "no_hand"
    assert loaded.samples[2].inside_workspace is False
    assert trajectory.samples is samples_before


def test_workspace_trajectory_json_rejects_missing_or_malformed_fields(tmp_path):
    path = save_workspace_trajectory_json(synthetic_workspace_trajectory(), tmp_path)
    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["samples"][1].pop("invalid_reason")
    malformed_path = tmp_path / "malformed.json"
    malformed_path.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValueError, match="missing required fields.*invalid_reason"):
        load_workspace_trajectory_json(malformed_path)


def test_workspace_playback_timeline_reuses_timing_for_mm_samples():
    trajectory = synthetic_workspace_trajectory()

    timeline = build_playback_timeline(trajectory, playback_speed=2.0)

    assert timeline.source_duration_ms == pytest.approx(250.0)
    assert timeline.playback_duration_ms == pytest.approx(125.0)
    assert [event.due_ms for event in timeline.events] == pytest.approx([0.0, 50.0, 125.0])
    due = timeline.samples_due(55.0)
    assert due == trajectory.samples[:2]
    assert due[-1].valid is False
    assert trajectory.samples == synthetic_workspace_trajectory().samples
