"""Write deterministic Task 5 MotionPlan fixtures for the offline MATLAB gallery."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

from .motion_library import MotionPlanner
from .personality_contracts import BehaviorEvent, PersonaConfig


def default_motion_plans() -> tuple:
    """Create one synthetic base-variant plan per configured Task 4 behavior."""
    planner = MotionPlanner(clock_ms=lambda: 10_000.0)
    plans = []
    for index, rule in enumerate(planner.persona_config.response_rules, start=1):
        behavior = BehaviorEvent(
            index, "banter-task5-fixture", index, rule.behavior_name, rule.base_variant,
            (index,), (), float(index * 100), 5_000.0,
        )
        plans.append(planner.plan(behavior))
    return tuple(plans)


def write_default_motion_plans(path: Path) -> Path:
    """Write an explicit cross-language fixture without modifying canonical Banter state."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema_version": "banter_motion_fixture_v1",
        "plans": [plan.to_dict() for plan in default_motion_plans()],
    }
    target.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return target


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="Output JSON path; generated and safe to delete.")
    arguments = parser.parse_args(argv)
    output = write_default_motion_plans(arguments.output)
    print(f"Wrote {len(default_motion_plans())} Task 5 MotionPlans to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
