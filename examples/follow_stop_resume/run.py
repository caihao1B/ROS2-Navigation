#!/usr/bin/env python3
"""Recompute inspection decisions and compare them with independent motion records."""

import argparse
import gzip
import json
import math
import subprocess
import sys
from pathlib import Path

CASE = Path(__file__).resolve().parent


def read_rows(filename):
    with gzip.open(CASE / filename, "rt", encoding="utf-8") as source:
        return [json.loads(line) for line in source]


def execute(binary, config, observations):
    payload = "".join(json.dumps(row, allow_nan=False) + "\n" for row in observations)
    proc = subprocess.run(
        [str(binary), str(config / "inspection_policy.json"), str(config / "inspection_tree.xml")],
        input=payload, check=True, capture_output=True, text=True,
    )
    return [json.loads(line) for line in proc.stdout.splitlines()]


def interpolate(window, sample_pair):
    first, second = sample_pair
    if not first["t"] <= window <= second["t"] or first["t"] == second["t"]:
        raise ValueError("physical pose samples do not bracket the requested time")
    weight = (window - first["t"]) / (second["t"] - first["t"])
    return [a + weight * (b - a) for a, b in zip(first["xy_m"], second["xy_m"], strict=True)]


def review(binary, config):
    inputs = read_rows("observations.jsonl.gz")
    computed = execute(binary, config, inputs)
    references = read_rows("recorded_decisions.jsonl.gz")
    if not inputs or len(inputs) != len(references) or len(inputs) != len(computed):
        raise ValueError("observation and decision streams differ in length")
    check_fields = ("t", "selected_action", "action", "reason_code", "stopped_confirmation")
    for result, recorded in zip(computed, references, strict=True):
        for key in check_fields:
            if result[key] != recorded[key]:
                raise ValueError(f"decision mismatch at {recorded['t']} s: {key}")
        if not math.isclose(result["speed_cap_mps"], recorded["speed_cap_mps"], abs_tol=1e-10):
            raise ValueError(f"speed limit mismatch at {recorded['t']} s")

    # These separate samples were recorded on the gate, odometry and evaluator
    # streams. None was provided to the C++ program above.
    feedback = json.loads((CASE / "motion_feedback.json").read_text(encoding="utf-8"))
    decisions = {row["t"]: row for row in computed}
    checkpoints = []
    for sample in feedback["checkpoints"]:
        row = decisions[sample["decision_t"]]
        stationary = sample["phase"] == "waiting"
        wanted = "WAIT" if stationary else "FOLLOW"
        gate = sample["gate"]
        if row["action"] != wanted or gate["stopped"] != stationary or gate["allowed"] == stationary:
            raise ValueError(f"vehicle and decision disagree in {sample['phase']}")
        checkpoints.append({
            "phase": sample["phase"], "sim_time_s": row["t"], "action": row["action"],
            "reason_code": row["reason_code"], "clearance_m": row["evidence"]["clearance_m"],
            "command_mps": gate["v"], "measured_mps": sample["measured"]["linear_mps"],
        })

    distances = {}
    for window in feedback["motion_windows"]:
        pair = window["bracketing_samples"]
        start = interpolate(window["t"][0], pair[:2])
        end = interpolate(window["t"][1], pair[2:])
        distances[window["name"]] = math.dist(start, end)
    if distances["leader_stopped"] >= 0.05 or distances["robot_stopped"] >= 0.05 or distances["robot_resumed"] <= 0.5:
        raise ValueError("the physical stop/resume checks failed")

    # Perturb the last observation of a real, warmed-up FOLLOW prefix. The
    # input sequence has no expected decisions or dispatch labels.
    target_t = checkpoints[0]["sim_time_s"]
    prefix = [row for row in inputs if row["t"] <= target_t]
    for change, expected in [({"leader_same_direction": False}, "WAIT"), ({"stale": True}, "STOP")]:
        altered = [*prefix[:-1], {**prefix[-1], **change}]
        result = execute(binary, config, altered)[-1]
        if result["selected_action"] != expected:
            raise ValueError(f"counterfactual {change} did not select {expected}")
        if expected == "STOP" and (result["action"] != "STOP" or result["reason_code"] != "INPUT_STALE"):
            raise ValueError("stale input did not issue a stop directive")
    return {"case": "follow-stop-resume", "passed": True, "sampled_decisions": len(computed),
            "checkpoints": checkpoints, "physical_displacement_m": distances}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true", help="write a machine-readable summary")
    parser.add_argument("--executable", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--config", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if bool(args.executable) != bool(args.config):
        parser.error("--executable and --config must be supplied together")
    try:
        if args.executable:
            binary, config = args.executable, args.config
        else:
            from ament_index_python.packages import get_package_prefix, get_package_share_directory
            prefix = Path(get_package_prefix("field_navigation"))
            binary = prefix / "lib" / "field_navigation" / "inspection_replay"
            config = Path(get_package_share_directory("field_navigation")) / "config"
        summary = review(binary, config)
    except (ImportError, LookupError, OSError, ValueError, subprocess.CalledProcessError) as error:
        reason = getattr(error, "stderr", "") or str(error)
        print(f"巡检复算失败。{reason.strip()}", file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps(summary, ensure_ascii=False, indent=2))
    else:
        titles = {"following": "跟随前方目标", "waiting": "停车等待", "resumed_following": "恢复跟随"}
        for sample in summary["checkpoints"]:
            print(f"{titles[sample['phase']]}  |  {sample['action']}  |  {sample['reason_code']}")
        print("同场对照通过。决策、实际下发指令、停车反馈与物理位移一致。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
