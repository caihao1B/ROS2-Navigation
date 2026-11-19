#!/usr/bin/env python3
"""Re-evaluate four on-vehicle parking windows and an independent blade-on session."""

import gzip
import json
import math
from pathlib import Path
import sys

CASE = Path(__file__).resolve().parent
sys.path.insert(0, str(CASE.parents[1] / "src" / "field_work"))
from field_work.mowing_feedback import evaluate_area, evaluate_parking


def observations(filename):
    with gzip.open(CASE / filename, "rt") as f:
        return json.load(f)


def run():
    ref = json.loads((CASE / "reference.json").read_text())
    stops = evaluate_parking(observations("parking-observations.json.gz"))
    if [s["segment"] for s in stops] != [r["segment"] for r in ref["parking"]]:
        raise ValueError("Parking sequence does not match the selected directed path")
    for s, r in zip(stops, ref["parking"]):
        if s["state"] != "PHYSICALLY_VERIFIED" or s["measured_drive_directions"] != [
            r["direction"]
        ]:
            raise ValueError(
                f"Driving direction or parking guard disagrees at {s['segment']}"
            )
        for key in ["max_xy_m", "max_yaw_rad"]:
            if not math.isclose(s[key], r[key], abs_tol=1e-8):
                raise ValueError(f"Parking measurement disagrees: {s['segment']} {key}")
    areas = evaluate_area(observations("cutting-observations.json.gz"))
    for key, value in ref["area"].items():
        if not math.isclose(areas[key], value, rel_tol=1e-9, abs_tol=1e-8):
            raise ValueError(f"Area measurement disagrees: {key}")
    return {
        "case": "strip-mowing",
        "mode": "offline replay of two independent Gazebo scenes",
        "blade_off_headland": [
            {
                "segment": r["segment"],
                "gear": r["gear"],
                "stop": r["state"],
                "max_position_error_m": r["max_xy_m"],
                "max_heading_error_rad": r["max_yaw_rad"],
            }
            for r in stops
        ],
        "blade_on_strip": areas,
    }


if __name__ == "__main__":
    print(json.dumps(run(), indent=2, ensure_ascii=False))
