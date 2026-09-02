"""Task 4 BehaviorEvent -> Task 5 JSON fixture integration tests."""

from __future__ import annotations

import json

from banter.export_task5_motion_plans import default_motion_plans, write_default_motion_plans


def test_fixture_exports_all_default_plans_without_mutating_behavior(tmp_path) -> None:
    plans = default_motion_plans()
    output = write_default_motion_plans(tmp_path / "task5.json")
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["schema_version"] == "banter_motion_fixture_v1"
    assert len(payload["plans"]) == len(plans) == 10
    assert [item["macro_name"] for item in payload["plans"]] == [plan.macro_name for plan in plans]
    assert all(item["motion_steps"][-1]["target_pose_name"] == "NEUTRAL" for item in payload["plans"])
