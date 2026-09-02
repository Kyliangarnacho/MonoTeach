"""Write deterministic Task 6 discrete-style fixtures for the MATLAB gallery."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

from .contracts import GestureKind
from .memory_contracts import PhraseMemoryKey
from .motion_library import MotionPlanner
from .motion_styling import MotionStyler
from .personality_contracts import BehaviorEvent, PersonaConfig


def default_styled_motion_plans() -> tuple:
    """Build every configured behavior/variant pair without camera or MATLAB state."""
    persona_config = PersonaConfig()
    planner = MotionPlanner(persona_config=persona_config, clock_ms=lambda: 10_000.0)
    styler = MotionStyler(persona_config=persona_config, clock_ms=lambda: 11_000.0)
    styled = []
    behavior_id = 1
    for rule in persona_config.response_rules:
        for variant in (rule.base_variant, *(item.variant for item in rule.variants)):
            if isinstance(rule.stimulus, GestureKind):
                token_ids, phrase_ids = (behavior_id,), ()
            elif isinstance(rule.stimulus, PhraseMemoryKey):
                token_ids, phrase_ids = (), (behavior_id,)
            else:  # Defensive even though PersonaConfig validates this union.
                raise TypeError("Task 4 BehaviorRule stimulus must be GestureKind or PhraseMemoryKey.")
            behavior = BehaviorEvent(
                behavior_id, "banter-task6-fixture", behavior_id,
                rule.behavior_name, variant, token_ids, phrase_ids,
                float(behavior_id * 100), 5_000.0,
            )
            styled.append(styler.style(planner.plan(behavior)))
            behavior_id += 1
    return tuple(styled)


def write_default_styled_motion_plans(path: Path) -> Path:
    """Write a disposable JSON fixture without modifying canonical Banter state."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema_version": "banter_styled_motion_fixture_v1",
        "plans": [plan.to_dict() for plan in default_styled_motion_plans()],
    }
    target.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return target


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="Output JSON path; generated and safe to delete.")
    arguments = parser.parse_args(argv)
    output = write_default_styled_motion_plans(arguments.output)
    print(f"Wrote {len(default_styled_motion_plans())} Task 6 StyledMotionPlans to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
