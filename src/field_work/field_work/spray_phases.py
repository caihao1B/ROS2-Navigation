"""Attribute disjoint simulation intervals to navigation, precision docking, localization and flow."""

import math
from collections import Counter

PHASES = ("travel_s", "precision_s", "localization_wait_s", "spray_flow_s", "other_s")
COMMANDS = ("still", "rotate", "slow", "medium", "cruise")
MOTION = ("translating", "rotating", "stationary")
COLUMNS = (
    "ordinal",
    "point_id",
    "status",
    "start_sim_s",
    "end_sim_s",
    "elapsed_s",
    *PHASES,
    *(f"navigation_{key}_s" for key in COMMANDS),
    *(f"body_{key}_during_nav_s" for key in MOTION),
    "nav_attempts",
    "requested_dose",
    "ledger_dose",
)
TICKS = 1_000_000
PRECISION_EVENTS = {
    "NAV_SUCCESS",
    "GLOBAL_APPROACH_STOP_FOR_PRECISION",
    "LOCAL_DOCK_GOAL",
    "LOCAL_DOCK_PROPOSAL_REFUSED",
    "DOCKING_STABLE_OFFSET",
    "DOCKING_OBSERVATION_CONTINUED",
    "LOCAL_DOCK_NAV_SUCCESS",
    "LOCAL_DOCK_NAV_FAILED",
    "LOCAL_DOCK_STOP_FOR_GLOBAL_OBSERVATION",
}
HOLD_EVENTS = {
    "LATE_LOCALIZATION_HOLD_DISPATCHED",
    "STOPPED_RELOCALIZATION_REQUEST",
    "LOCALIZATION_PREVIEW_PREPARED",
    "STOPPED_PLAN_REVALIDATION",
    "LOCALIZATION_PLAN_COMMITTED",
}


def tick(t):
    if not math.isfinite(t) or t < 0:
        raise ValueError("Invalid sim timestamp")
    return round(t * TICKS)


def secs(t):
    return t / TICKS


def mission_markers(events):
    markers = []
    retry_start = None
    for e in events:
        t, name, reason = tick(e["t"]), e["event"], e.get("reason", "")
        stage = None
        if name == "NAV_GOAL":
            stage = "travel_s"
        elif name in PRECISION_EVENTS:
            stage = "precision_s"
        elif name == "DOCKING_SETTLED_CHECK":
            if retry_start is not None and "continuous_stopped_s" in e:
                # After RESIDUAL_RETRY the executor waits for permission and then docks.
                # Attribute only the observed continuous stationary interval to that retry.
                start = max(retry_start, t - tick(e["continuous_stopped_s"]))
                markers.append((start, "precision_s"))
            retry_start = None
            stage = "other_s" if e["stable"] else "precision_s"
        elif name in ("BT_INTERRUPTS_NAV", "BT_CANCEL_TOOL", "LOCAL_DOCK_INTERRUPTED"):
            stage = "localization_wait_s" if "LOCALIZATION" in reason else "other_s"
        elif name in HOLD_EVENTS:
            stage = "localization_wait_s"
        elif name == "RESIDUAL_RETRY":
            retry_start, stage = t, "other_s"
        elif name in (
            "NAV_TERMINAL_FAILURE",
            "NAV_REJECTED",
            "TOOL_RESULT",
            "TASK_STOP",
            "SUBSET_FINISHED",
            "LOCALIZATION_REVALIDATED_RESUME",
            "NEAR_GOAL_PROGRESS_REPLAN",
        ):
            stage = "other_s"
        if stage:
            markers.append((t, stage))
    return markers


def point_boundaries(trace):
    events = trace["events"]
    times = [tick(e["t"]) for e in events]
    if times != sorted(times) or any(t > tick(trace["cutoff_sim_s"]) for t in times):
        raise ValueError("Mission timestamps are out of sequence")
    goals, success = {}, {}
    for e in events:
        p, name, t = e.get("point"), e["event"], tick(e["t"])
        if name == "NAV_GOAL":
            goals.setdefault(p, t)
        elif name == "TOOL_RESULT" and e["state"] == "SUCCEEDED" and not e["replayed"]:
            if p in success or e["remaining"] > 1e-9 or e["uncertainty"]:
                raise ValueError("Repeated or uncertain service confirmation")
            success[p] = t
    ledger = dict(trace["ledger"])
    if (
        list(success) != trace["confirmed_ids"]
        or set(success) != set(ledger)
        or not set(success).issubset(goals)
    ):
        raise ValueError("Event, result and persistent ledger disagree")
    selected = trace["selected"]
    planned = [p for p, _ in trace["points"]]
    if (
        len(set(planned)) != len(planned)
        or len(selected) != len(set(selected))
        or not set(selected).issubset(planned)
        or list(goals) != [p for p in selected if p in goals]
    ):
        raise ValueError("Goal order differs from selected obligations")
    order = list(goals)
    first = goals[order[0]] if order else tick(trace["cutoff_sim_s"])
    stop = next(
        (
            tick(e["t"])
            for e in events
            if e["event"] in ("TASK_STOP", "SUBSET_FINISHED")
        ),
        tick(trace["cutoff_sim_s"]),
    )
    boundaries = {}
    previous = first
    for i, p in enumerate(order):
        end = success.get(p, goals[order[i + 1]] if i + 1 < len(order) else stop)
        if not previous <= goals[p] <= end <= stop:
            raise ValueError("Invalid point-cycle boundaries")
        boundaries[p] = (previous, end)
        previous = end
    return boundaries, ledger


def analyze(trace):
    if trace["schema"] != "spot-service-observations/v1":
        raise ValueError("Unknown observation schema")
    bounds, ledger = point_boundaries(trace)
    totals = {p: Counter() for p in bounds}
    markers = [(t, "phase", s) for t, s in mission_markers(trace["events"])]
    for p, (start, end) in bounds.items():
        markers.extend(((start, "point", p), (end, "point", None)))
    for channel in ("flow", "commands", "body"):
        sensor = trace[channel]
        if bounds and (
            sensor["first_us"] > min(lo for lo, _ in bounds.values())
            or sensor["last_us"] < max(hi for _, hi in bounds.values())
            or sensor["max_gap_us"] > tick(0.15)
        ):
            raise ValueError("Sensor evidence does not cover the assessed interval")
        markers.extend((t, channel, value) for t, value in sensor["changes"])
    state = {"point": None, "phase": "other_s", "flow": None}
    last = 0
    for t, channel, value in sorted(markers, key=lambda m: m[0]):
        point = state["point"]
        if t > last:
            flow = state["flow"]
            if flow is not None and flow != point:
                raise ValueError("Physical discharge outside its own point cycle")
            if point is not None:
                phase = "spray_flow_s" if flow is not None else state["phase"]
                counts = totals[point]
                counts[phase] += t - last
                if phase == "travel_s":
                    counts[f"navigation_{state['commands']}_s"] += t - last
                    counts[f"body_{state['body']}_during_nav_s"] += t - last
        last = t
        state[channel] = value
    attempts = Counter(e["point"] for e in trace["events"] if e["event"] == "NAV_GOAL")
    rows = []
    for i, (p, requested) in enumerate(trace["points"], 1):
        row = dict.fromkeys(COLUMNS)
        status = (
            "CONFIRMED"
            if p in ledger
            else (
                "ATTEMPTED_INCOMPLETE"
                if p in bounds
                else ("NOT_REACHED" if p in trace["selected"] else "NOT_SELECTED")
            )
        )
        row.update(
            ordinal=i,
            point_id=p,
            status=status,
            requested_dose=requested,
            ledger_dose=ledger.get(p),
        )
        if p in bounds:
            start, end = bounds[p]
            counts = totals[p]
            if (
                sum(counts[k] for k in PHASES) != end - start
                or sum(counts[f"navigation_{k}_s"] for k in COMMANDS)
                != counts["travel_s"]
                or sum(counts[f"body_{k}_during_nav_s"] for k in MOTION)
                != counts["travel_s"]
            ):
                raise ValueError("Per-point intervals do not reconcile")
            if p in ledger and (
                ledger[p] + 1e-9 < requested or counts["spray_flow_s"] <= 0
            ):
                raise ValueError(
                    "Completed dose without quantity and independent physical discharge"
                )
            row.update(
                start_sim_s=secs(start),
                end_sim_s=secs(end),
                elapsed_s=secs(end - start),
                nav_attempts=attempts[p],
            )
            for key in COLUMNS[
                COLUMNS.index("travel_s") : COLUMNS.index("nav_attempts")
            ]:
                row[key] = secs(counts[key])
        rows.append(row)
    return [row for row in rows if row["status"] != "NOT_SELECTED"]
