"""Offline Task 5 verify; it builds no camera, TCP, MATLAB, or robot connection."""

from __future__ import annotations

from .export_task5_motion_plans import default_motion_plans
from .motion_library import MotionPlanner
from .personality_contracts import BehaviorEvent


def main() -> int:
    """Check complete macro coverage, discrete variants, and return-neutral plans."""
    print("[1/3] Checking one macro for every configured Task 4 behavior...")
    plans = default_motion_plans()
    assert len(plans) == 10
    assert len({plan.macro_name for plan in plans}) == len(plans)
    assert all(plan.motion_steps[-1].target_pose_name == "NEUTRAL" for plan in plans)

    print("[2/3] Checking discrete FIST variants retain FIST_COUNTER's macro...")
    planner = MotionPlanner(clock_ms=lambda: 10_000.0)
    playful = planner.plan(BehaviorEvent(21, "verify-task5", 1, "FIST_COUNTER", "DEFAULT", (21,), (), 100.0, 5_000.0))
    forceful = planner.plan(BehaviorEvent(22, "verify-task5", 2, "FIST_COUNTER", "FORCEFUL", (22,), (), 200.0, 5_000.0))
    assert playful.macro_name == forceful.macro_name == "COUNTER_JAB"
    assert not hasattr(forceful, "intensity")

    print("[3/3] Checking BehaviorEvent provenance and availability survive the expansion...")
    assert forceful.source_behavior.source_token_event_ids == (22,)
    assert forceful.available_t_ms >= forceful.source_behavior.available_t_ms
    print("PASS: Task 5 semantic MotionPlans have complete coverage, discrete variants, and return-neutral macros.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
