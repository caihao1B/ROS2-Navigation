#!/usr/bin/env python3
"""Check a completed parked session from recorded observations and command feedback."""

import argparse
import json
from pathlib import Path


def records(path):
    with path.open() as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def evaluate(folder):
    def trace(name):
        return records(folder / name)

    def summary(name):
        return json.loads((folder / name).read_text())

    gates = list(trace("control/gate.jsonl"))
    tracks = trace("tracking/tracks.jsonl")
    measured = [
        r
        for r in trace("samples_motion/generation.jsonl")
        if r.get("sensor") == "wheel" and "shaft_v" in r
    ]
    captures = (
        summary("samples_motion/result.json")["counts"]
        | summary("samples_vision/result.json")["counts"]
    )
    final = summary("live.json")
    checks = {
        "stationary_session": summary("spec.json")["stationary"],
        "calibrated_observations": all(
            captures.get(sensor, 0) > 0
            for sensor in ("points", "left", "right", "wheel", "imu", "gnss")
        ),
        "surface_and_gnss_agree": any(
            r.get("valid") and r["reason"] == "MAP_AND_GNSS_AGREE"
            for r in trace("reference/reference.jsonl")
        ),
        "icp_increment_valid": any(
            r.get("valid") for r in trace("icp_gate/quality.jsonl")
        ),
        "observed_traversability": any(
            r.get("valid") and r["boundary_valid"] for r in trace("risk/risk.jsonl")
        ),
        "observed_path_prefix": any(
            r.get("valid") and r.get("path_valid") for r in tracks
        ),
        "decision_observed": bool((final.get("bt") or {}).get("reason_code")),
        "all_gate_requests_stay_parked": bool(gates)
        and all(
            not g["allowed"] and g["mode"] == "STOP" and g["v"] == 0 and g["w"] == 0
            for g in gates
        ),
        "stopped_feedback": any(g["stopped"] and g["nav_idle"] for g in gates),
        "stationary_axle_measurement": bool(measured)
        and max(abs(r["shaft_v"]) for r in measured) < 0.01,
        "no_navigation_dispatch": not any(
            r.get("event") == "FOLLOW_PATH_GOAL" for r in trace("mission_events.jsonl")
        ),
    }
    return {
        "scope": "parked Gazebo sensors, localization, BT and motion interlock",
        "checks": checks,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    try:
        result = evaluate(args.session)
    except (FileNotFoundError, KeyError, ValueError) as problem:
        parser.exit(2, f"Session evidence is incomplete: {problem}\n")
    print(json.dumps(result, indent=2))
    return 0 if all(result["checks"].values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
