"""Check directed parking against measured chassis, rotor and gate streams."""

import math
from bisect import bisect_left, bisect_right
import numpy as np
from shapely.geometry import Point, LineString, shape
from shapely import from_wkb, union_all


def yaw_error(a, b):
    return abs(math.remainder(a - b, math.tau))


def pose_error(world, yaw, goal):
    return math.dist(world[:2], goal[:2]), yaw_error(yaw, goal[2])


def speed_and_distance(truth):
    signed = []
    rates = []
    for a, b in zip(truth, truth[1:]):
        ds = np.array(b["xyz"][:2]) - a["xyz"][:2]
        signed.append(float(ds @ [math.cos(a["rpy"][2]), math.sin(a["rpy"][2])]))
        dt = b["t"] - a["t"]
        if dt > 0:
            rates.append(yaw_error(b["rpy"][2], a["rpy"][2]) / dt)
    return (
        sum(max(0, x) for x in signed),
        sum(max(0, -x) for x in signed),
        max(rates, default=None),
    )


def sampled(records, max_gap, span):
    times = [r["t"] for r in records]
    return bool(
        len(times) >= 2
        and times[-1] - times[0] >= span - max_gap
        and all(0 < b - a <= max_gap for a, b in zip(times, times[1:]))
    )


class Timeline:
    def __init__(self, records):
        self.records = records
        self.t = [r["t"] for r in records]
        if any(b < a for a, b in zip(self.t, self.t[1:])):
            raise ValueError("Physical observation time moved backwards")

    def window(self, end, span):
        a = bisect_left(self.t, end - span - 1e-6)
        b = bisect_right(self.t, end + 1e-6)
        return self.records[a:b]


def stop_row(
    segment, events, mission, cfg, timelines, previous_direction, next_direction
):
    sid = segment["id"]
    nav = cfg["navigation_execution"]
    limits = nav["stopped_arrival_confirmation"]
    safety = cfg["vehicle"]["safety"]
    row = dict(
        segment=sid,
        kind=segment["kind"],
        gear=segment["direction"],
        gear_changed_at_entry=previous_direction is not None
        and segment["direction"] != previous_direction,
        gear_changes_after=next_direction is not None
        and segment["direction"] != next_direction,
        endpoint_map=segment["poses"][-1],
        state="NOT_STARTED",
    )
    if not events:
        return row
    row["state"] = "ATTEMPTED"
    completed = sid in mission["completed_segments"]
    selected = [e for e in events if e["event"] == "STOPPED_ORIGINAL_ARRIVAL_CONFIRMED"]
    row["worker_completed"] = completed
    if not selected:
        return row
    held = selected[-1]
    end, span = held["t"], held["span_s"]
    truth, rotor, gate = (tl.window(end, span) for tl in timelines)
    goal = segment["poses_world"][-1]
    errors = [pose_error(p["xyz"], p["rpy"][2], goal) for p in truth]
    _, _, max_w = speed_and_distance(truth)
    max_v = max((abs(r["body_v"]) for r in rotor), default=None)
    err_xy = max((p[0] for p in errors), default=None)
    err_yaw = max((p[1] for p in errors), default=None)
    max_rotor = max((abs(p["rpm_rad_s"]) for p in rotor), default=None)
    goals = [e for e in events if e["event"] == "SEGMENT_GOAL"]
    drive_directions = []
    directed_travel = None
    cross_track = None
    primary_gears = [
        e
        for e in events
        if e["event"] == "DIRECTION_ARMED_WHILE_STOPPED"
        and e.get("direction") == segment["direction"]
    ]
    if goals:
        start = goals[0]["t"]
        motion = timelines[0].window(end, end - start)
        gates = timelines[2].window(end, end - start)
        drive_directions = sorted(
            {
                int(math.copysign(1, g["v"]))
                for g in gates
                if g["mode"] == "DRIVE" and abs(g["v"]) > 0.005
            }
        )
        forward, reverse, _ = speed_and_distance(motion)
        directed_travel = forward if segment["direction"] > 0 else reverse
        if segment["id"] in cfg["connector_qualification"]["segments"]:
            curve = LineString([p[:2] for p in segment["poses_world"]])
            cross_track = max(
                (curve.distance(Point(p["xyz"][:2])) for p in motion), default=None
            )
    checks = dict(
        ordered_worker_admission=completed
        and any(e["event"] == "SEGMENT_ARRIVAL" for e in events),
        original_stop_window=held["samples"] >= limits["minimum_samples"]
        and span >= limits["span_s"],
        sampled=sampled(truth, 0.05, span)
        and sampled(rotor, 0.11, span)
        and sampled(gate, 0.09, span),
        physical_xy=err_xy is not None and err_xy <= nav["intermediate_xy_tolerance_m"],
        physical_yaw=err_yaw is not None
        and err_yaw <= nav["intermediate_yaw_tolerance_rad"],
        body_stopped=max_v is not None
        and max_v <= safety["stopped_v_mps"]
        and max_w is not None
        and max_w <= safety["stopped_w_rad_s"],
        blade_stopped=max_rotor is not None
        and max_rotor < 0.1
        and all(p["healthy"] and not p["on"] for p in rotor),
        gate_stopped=bool(gate)
        and all(g["tool_stopped"] and g["stopped"] for g in gate),
    )
    if row["gear_changed_at_entry"]:
        checks["new_gear_armed_at_physical_stop"] = bool(goals) and any(
            e["t"] <= goals[0]["t"] for e in primary_gears
        )
    if segment["id"] in cfg["connector_qualification"]["segments"]:
        checks["required_stopped_gear_armed"] = bool(goals) and any(
            e["t"] <= goals[0]["t"] for e in primary_gears
        )
        checks["strict_original_gear_and_route"] = (
            drive_directions == [segment["direction"]]
            and directed_travel is not None
            and directed_travel > 0.5 * segment["length_m"]
            and cross_track is not None
            and cross_track <= nav["endpoint_recovery"]["max_tracking_error_m"]
        )
    return dict(
        row,
        measured_drive_directions=drive_directions,
        directed_travel_m=directed_travel,
        connector_max_cross_track_m=cross_track,
        state=(
            "PHYSICALLY_VERIFIED" if all(checks.values()) else "PARKING_NOT_CONFIRMED"
        ),
        sim_s=end,
        span_s=span,
        loc_samples=held["samples"],
        truth_samples=len(truth),
        tool_samples=len(rotor),
        gate_samples=len(gate),
        max_xy_m=err_xy,
        max_yaw_rad=err_yaw,
        max_yaw_deg=math.degrees(err_yaw) if err_yaw is not None else None,
        max_linear_mps=max_v,
        max_angular_rad_s=max_w,
        max_blade_rad_s=max_rotor,
        checks=checks,
    )


def evaluate_parking(observations):
    sections = observations["sections"]
    streams = observations["streams"]
    timelines = [
        Timeline([dict(zip(stream["fields"], r)) for r in stream["rows"]])
        for stream in streams
    ]
    result = []
    for i, s in enumerate(sections):
        events = [e for e in observations["events"] if e["segment"] == s["id"]]
        result.append(
            stop_row(
                s,
                events,
                observations,
                observations["cfg"],
                timelines,
                sections[i - 1]["direction"] if i else None,
                sections[i + 1]["direction"] if i + 1 < len(sections) else None,
            )
        )
    return result


def evaluate_area(observations):
    target = shape(observations["target_map"])
    blade = union_all(
        [shape(j["area"]) for j in observations["rotor"] if j["active_samples"] > 0]
    )
    credited = union_all(
        [
            from_wkb(bytes.fromhex(j["wkb_hex"]))
            for j in observations["credits"]
            if j["wkb_hex"]
        ]
    )
    return {
        "target_m2": target.area,
        "actual_cut_m2": blade.intersection(target).area,
        "confirmed_union_m2": credited.intersection(target).area,
        "residual_m2": target.difference(credited).area,
    }
