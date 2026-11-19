#!/usr/bin/env python3
"""Re-plan a service tour and replay a sampled precision stop, dose and task events."""

import gzip
import json
import math
from pathlib import Path
import sys

CASE = Path(__file__).resolve().parent
WORK = CASE.parents[1] / "src" / "field_work"
sys.path.insert(0, str(WORK))
from field_work.docking import StoppedWindow
from field_work.ledger import Ledger
from field_work.spray_phases import analyze
from field_work.spray_planning import plan_route


def observations(name):
    with gzip.open(CASE / name, "rt") as f:
        return json.load(f)


def confirm(actual, expected, label):
    if not math.isclose(actual, expected, rel_tol=1e-9, abs_tol=1e-6):
        raise ValueError(f"{label}: recomputed {actual}, recorded {expected}")


def replay_dock(bundle):
    window = StoppedWindow(bundle["target"], bundle["settle_s"])
    settled = False
    for row in bundle["observations"]:
        settled, _ = window.update(row["loc"]["t"], row["loc"], row["gate"])
    return settled


def replay_meter(bundle, config):
    ledger = Ledger(":memory:", config)
    goal = bundle["goal"]
    key = goal["idempotency_key"]
    try:
        reservation = ledger.reserve(goal, bundle["attempt"])
        if not reservation["ok"]:
            raise ValueError(reservation)
        for row in bundle["observations"]:
            ledger.observe(row["feedback"], row["context"], row["now"])
        finished = ledger.finish(
            key, "SUCCEEDED", "CONFIRMED_WORK", False, bundle["observations"][-1]["now"]
        )
        replay = ledger.reserve(goal, "repeat-request")
        if (
            finished["state"] != "SUCCEEDED"
            or not replay.get("replayed")
            or replay["confirmed_quantity"] != finished["confirmed_quantity"]
        ):
            raise ValueError("Meter replay or idempotent work accounting failed")
        return finished["confirmed_quantity"]
    finally:
        ledger.close()


def run():
    ref = json.loads((CASE / "reference.json").read_text())
    result = plan_route(json.loads((CASE / "planning.json").read_text()))
    for candidate in ["original", "optimized"]:
        for key, expected in ref["planning"][candidate].items():
            confirm(result[candidate][key], expected, f"{candidate}.{key}")
    service = analyze(observations("service-observations.json.gz"))
    confirmed = [r for r in service if r["status"] == "CONFIRMED"]
    if {r["point_id"] for r in confirmed} != set(ref["service_points"]):
        raise ValueError("Confirmed service and original obligations disagree")
    for key, value in ref["phase_totals"].items():
        confirm(sum(row[key] for row in confirmed), value, key)
    dock = replay_dock(observations("docking-observations.json.gz"))
    if not dock:
        raise ValueError("No stable stop before the metered spray")
    quantity = replay_meter(
        observations("meter-observations.json.gz"),
        json.loads((WORK / "config/sprayer.json").read_text()),
    )
    confirm(quantity, ref["meter_quantity"], "measured dose")
    return {
        "case": "spot-spray",
        "mode": "offline measured feedback replay and route optimization",
        "route": result,
        "stable_stop_observed": dock,
        "metered_quantity": quantity,
        "repeated_goal": "cached result with no duplicate dose",
        "work": [
            {
                "point": r["point_id"],
                "dose": r["ledger_dose"],
                "phases_sim_s": {k: r[k] for k in ref["phase_totals"]},
            }
            for r in confirmed
        ],
    }


if __name__ == "__main__":
    print(json.dumps(run(), indent=2, ensure_ascii=False))
