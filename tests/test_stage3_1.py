"""Cross-language fixture checks for the Stage 3.1A workspace JSON bridge."""

from pathlib import Path

import pytest

from stage2.workspace_trajectory_io import load_workspace_trajectory_json


FIXTURE_PATH = (
    Path(__file__).resolve().parents[1]
    / "stage3"
    / "data"
    / "workspace_trajectory_fixture.json"
)


def test_workspace_trajectory_fixture_matches_stage_2_3_contract():
    """The MATLAB fixture remains loadable by the existing Python contract."""
    trajectory = load_workspace_trajectory_json(FIXTURE_PATH)

    assert trajectory.metadata.source_trajectory_id == "stage3-1a-fixture"
    assert trajectory.metadata.workspace_calibration_id == "workspace_2d_fixture_190x290"
    assert trajectory.metadata.coordinate_frame == "workspace_2d"
    assert trajectory.metadata.width_mm == pytest.approx(190.0)
    assert trajectory.metadata.height_mm == pytest.approx(290.0)

    assert len(trajectory.samples) == 5
    assert sum(sample.valid for sample in trajectory.samples) == 4
    assert sum(sample.inside_workspace for sample in trajectory.samples) == 3

    first, center, invalid, inside, outside = trajectory.samples
    assert (first.t_ms, first.x_mm, first.y_mm) == pytest.approx((0.0, 20.0, 30.0))
    assert (center.t_ms, center.x_mm, center.y_mm) == pytest.approx((100.0, 95.0, 145.0))
    assert (inside.t_ms, inside.x_mm, inside.y_mm) == pytest.approx((300.0, 180.0, 280.0))

    assert invalid.valid is False
    assert invalid.x_mm is None
    assert invalid.y_mm is None
    assert invalid.inside_workspace is False
    assert invalid.invalid_reason == "no_hand"

    assert outside.valid is True
    assert outside.inside_workspace is False
    assert (outside.x_mm, outside.y_mm) == pytest.approx((205.0, 100.0))
    assert outside.invalid_reason is None
