"""Offline Task 6 verify; it creates neither a camera nor a robot connection."""

from __future__ import annotations

from .export_task6_styled_plans import default_styled_motion_plans
from .personality_contracts import BEHAVIOR_VARIANT_VOCABULARY


def main() -> int:
    """Check compact variant coverage and FIST-only discrete timing escalation."""
    plans = default_styled_motion_plans()
    print("[1/3] Checking the global three-variant vocabulary and compact fixture coverage...")
    assert BEHAVIOR_VARIANT_VOCABULARY == frozenset({"DEFAULT", "FIRM", "FORCEFUL"})
    assert len(plans) == 12

    print("[2/3] Checking ordinary Behavior plans retain the identity DEFAULT style...")
    ordinary = [item for item in plans if item.style.variant == "DEFAULT"]
    assert len(ordinary) == 10
    assert all(item.motion_steps == item.source_motion_plan.motion_steps for item in ordinary)

    print("[3/3] Checking FIST uses discrete timing escalation without changing pose names...")
    fists = {item.style.variant: item for item in plans if item.source_motion_plan.source_behavior.behavior_name == "FIST_COUNTER"}
    assert set(fists) == BEHAVIOR_VARIANT_VOCABULARY
    assert fists["FORCEFUL"].style.move_duration_scale < fists["FIRM"].style.move_duration_scale < fists["DEFAULT"].style.move_duration_scale
    assert fists["FORCEFUL"].style.hold_duration_scale > fists["FIRM"].style.hold_duration_scale
    assert [step.target_pose_name for step in fists["DEFAULT"].motion_steps] == [step.target_pose_name for step in fists["FORCEFUL"].motion_steps]
    print("PASS: Task 6 applies compact, FIST-only discrete timing styles without altering Macro geometry.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
