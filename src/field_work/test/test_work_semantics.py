"""Exercise actual measurements and deliberately remove evidence needed by each guard."""

import gzip
import json
import math
from pathlib import Path

from field_work.docking import StoppedWindow
from field_work.ledger import Ledger
from field_work.mowing_feedback import evaluate_area, evaluate_parking

ROOT = Path(__file__).resolve().parents[3]
SPRAY = ROOT / "examples/spot_spray"
MOW = ROOT / "examples/strip_mowing"


def bundle(path):
    with gzip.open(path, "rt") as stream:
        return json.load(stream)


def process_window(sample):
    window = StoppedWindow(sample["target"], sample["settle_s"])
    for row in sample["observations"]:
        settled, _ = window.update(row["loc"]["t"], row["loc"], row["gate"])
    return settled


def test_stationary_window_rejects_old_gate():
    sample = bundle(SPRAY / "docking-observations.json.gz")
    assert process_window(sample)
    sample["observations"][-1]["gate"]["t"] -= 1
    assert not process_window(sample)


def test_dose_is_idempotent_across_restart(tmp_path):
    data = bundle(SPRAY / "meter-observations.json.gz")
    cfg = json.loads((ROOT / "src/field_work/config/sprayer.json").read_text())
    original = json.loads((SPRAY / "reference.json").read_text())["meter_quantity"]
    db = tmp_path / "service.sqlite"
    job = data["goal"]
    ledger = Ledger(db, cfg)
    try:
        assert ledger.reserve(job, data["attempt"])["ok"]
        for row in data["observations"]:
            ledger.observe(row["feedback"], row["context"], row["now"])
        assert (
            ledger.observe(row["feedback"], row["context"], row["now"])["reason"]
            == "DUPLICATE"
        )
        assert math.isclose(
            ledger.finish(
                job["idempotency_key"], "SUCCEEDED", "CONFIRMED_WORK", False, row["now"]
            )["confirmed_quantity"],
            original,
        )
    finally:
        ledger.close()
    reopened = Ledger(db, cfg)
    try:
        cached = reopened.reserve(job, "duplicate-request")
        assert cached["replayed"] and math.isclose(
            cached["confirmed_quantity"], original
        )
        assert (
            reopened.reserve(
                dict(job, requested_quantity=job["requested_quantity"] + 1), "other"
            )["error"]
            == "IDEMPOTENCY_CONFLICT"
        )
    finally:
        reopened.close()


def test_park_requires_sampled_rotor_and_correct_heading():
    sample = bundle(MOW / "parking-observations.json.gz")
    stop = evaluate_parking(sample)[0]
    assert stop["state"] == "PHYSICALLY_VERIFIED"
    end = stop["sim_s"]
    target = next(
        row for row in sample["streams"][0]["rows"] if end - 0.4 <= row[0] <= end - 0.1
    )
    target[2][2] += 0.2
    assert not evaluate_parking(sample)[0]["checks"]["physical_yaw"]
    sample["streams"][1]["rows"] = []
    assert not evaluate_parking(sample)[0]["checks"]["sampled"]


def test_park_rejects_reversed_motion():
    sample = bundle(MOW / "parking-observations.json.gz")
    for gate in sample["streams"][2]["rows"]:
        if gate[1] == "DRIVE":
            gate[2] = -gate[2]
    assert all(
        not item["checks"]["strict_original_gear_and_route"]
        for item in evaluate_parking(sample)
    )


def test_area_depends_on_tool_feedback_and_union():
    sample = bundle(MOW / "cutting-observations.json.gz")
    observed = evaluate_area(sample)
    assert observed["confirmed_union_m2"] > 0 and observed["actual_cut_m2"] > 0
    sample["credits"] *= 2
    assert math.isclose(
        evaluate_area(sample)["confirmed_union_m2"], observed["confirmed_union_m2"]
    )
    sample["credits"].clear()
    for row in sample["rotor"]:
        row["active_samples"] = 0
    empty = evaluate_area(sample)
    assert empty["actual_cut_m2"] == empty["confirmed_union_m2"] == 0
    assert math.isclose(empty["target_m2"], empty["residual_m2"])
