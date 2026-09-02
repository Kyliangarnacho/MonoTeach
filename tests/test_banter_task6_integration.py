"""Task 6 deterministic fixture integration tests."""

from __future__ import annotations

import json

from banter.export_task6_styled_plans import default_styled_motion_plans, write_default_styled_motion_plans


def test_fixture_exports_compact_default_and_fist_escalation_coverage(tmp_path) -> None:
    plans = default_styled_motion_plans()
    output = write_default_styled_motion_plans(tmp_path / "task6.json")
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["schema_version"] == "banter_styled_motion_fixture_v1"
    assert len(payload["plans"]) == len(plans) == 12
    variants = {
        (item["behavior_name"], item["variant"])
        for item in payload["plans"]
    }
    assert ("FIST_COUNTER", "DEFAULT") in variants
    assert ("FIST_COUNTER", "FIRM") in variants
    assert ("FIST_COUNTER", "FORCEFUL") in variants
    assert sum(variant != "DEFAULT" for _, variant in variants) == 2
    assert all("pose_excursion_scale" not in item["style"] for item in payload["plans"])
