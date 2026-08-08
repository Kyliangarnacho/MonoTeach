import json
from pathlib import Path

import numpy as np

from stage0.kinematics import forward_kinematics


TEST_CASES = [
    {
        "name": "home",
        "joint_angles_deg": [0, 90, 0, 90, 0],
    },
    {
        "name": "normal",
        "joint_angles_deg": [30, 60, -30, 120, 45],
    },
    {
        "name": "large_angles",
        "joint_angles_deg": [-80, 150, 70, 30, -90],
    },
]


def main():
    results = []

    for case in TEST_CASES:
        q_deg = case["joint_angles_deg"]

        position, transform = forward_kinematics(q_deg)

        results.append(
            {
                "name": case["name"],
                "joint_angles_deg": q_deg,
                "position_m": position.tolist(),
                "transform": transform.tolist(),
            }
        )

    output_dir = Path(__file__).parent / "data"
    output_dir.mkdir(exist_ok=True)

    output_path = output_dir / "python_fk_reference.json"

    with output_path.open("w", encoding="utf-8") as f:
        json.dump(
            results,
            f,
            indent=2,
            ensure_ascii=False,
        )

    print(f"FK reference exported to:")
    print(output_path)

    for item in results:
        print()
        print(item["name"])
        print("q =", item["joint_angles_deg"])
        print(
            "position =",
            np.array(item["position_m"]),
        )


if __name__ == "__main__":
    main()