"""A stable observed stop, not one lucky pose sample, admits point work."""

import math, statistics
from collections import deque


def stable_dock(history, target):
    if not history:
        return False, dict(reason="NO_OBSERVATION")
    times = [x[0] for x in history]
    xy = [x[1] for x in history]
    median = [statistics.median(p[j] for p in xy) for j in [0, 1]]
    dist = math.dist(median, target)
    errors = sorted(math.dist(p, median) for p in xy)
    p95 = errors[min(len(errors) - 1, math.ceil(0.95 * len(errors)) - 1)]
    span = times[-1] - times[0]
    gap = max((b - a for a, b in zip(times, times[1:])), default=float("inf"))
    latest = math.dist(xy[-1], target)
    ok = (
        len(xy) >= 20
        and span >= 1.8
        and gap <= 0.15
        and dist <= 0.035
        and latest <= 0.045
        and p95 <= 0.015
    )
    return ok, dict(
        samples=len(xy),
        span_s=span,
        max_gap_s=gap,
        median_distance_m=dist,
        latest_distance_m=latest,
        position_scatter_p95_m=p95,
    )


class StoppedWindow:
    def __init__(self, target, settle_s):
        self.target = target
        self.settle_s = settle_s
        self.history = []
        self.epoch = None
        self.stopped_since = None

    def update(self, now, loc, gate):
        stamp = loc.get("odom_t")
        qualified = (
            loc.get("valid")
            and loc.get("pose")
            and stamp is not None
            and -0.03 <= now - stamp < 0.15
            and -0.03 <= now - loc.get("t", -100) < 0.15
            and -0.03 <= now - gate.get("t", -100) < 0.15
            and gate.get("stopped")
            and gate.get("tool_stopped")
        )
        epoch = loc.get("localization_epoch")
        if not qualified or epoch != self.epoch:
            self.history.clear()
            self.stopped_since = None
            self.epoch = epoch
        if not qualified:
            return False, {}
        if self.history and (
            stamp < self.history[-1][0] or stamp - self.history[-1][0] > 0.15
        ):
            self.history.clear()
            self.stopped_since = None
        if self.stopped_since is None:
            self.stopped_since = now
        # Global messages can arrive after the gate with an older acquisition stamp.
        if stamp < self.stopped_since:
            return False, {}
        if not self.history or stamp > self.history[-1][0]:
            self.history.append((stamp, loc["pose"][:2]))
            self.history[:] = [r for r in self.history if stamp - r[0] <= 2.2]
        stable, metrics = stable_dock(self.history, self.target)
        metrics["continuous_stopped_s"] = now - self.stopped_since
        metrics["localization_epoch"] = epoch
        return bool(stable and now - self.stopped_since >= self.settle_s), metrics


class LowSpeedCapture:
    """Request a controlled stop only after multiple in-tolerance samples."""

    def __init__(self, goal_m):
        self.radius = goal_m
        self.samples = deque()
        self.epoch = None

    def update(self, now, target, loc, gate, qualified):
        stamp = loc.get("odom_t")
        epoch = loc.get("localization_epoch")
        if not (
            qualified
            and loc.get("pose")
            and stamp is not None
            and -0.03 <= now - stamp < 0.15
            and -0.03 <= now - gate.get("t", -100) < 0.15
            and math.isfinite(loc.get("v", float("nan")))
            and abs(loc["v"]) <= 0.10
        ):
            self.samples.clear()
            return None
        distance = math.dist(loc["pose"][:2], target)
        if (
            not math.isfinite(distance)
            or distance > self.radius
            or epoch != self.epoch
            or (self.samples and not 0 <= stamp - self.samples[-1][0] <= 0.15)
        ):
            self.samples.clear()
        self.epoch = epoch
        if not math.isfinite(distance) or distance > self.radius:
            return None
        if self.samples and stamp == self.samples[-1][0]:
            return None
        self.samples.append((stamp, distance))
        while self.samples and stamp - self.samples[0][0] > 0.35:
            self.samples.popleft()
        if len(self.samples) >= 4 and stamp - self.samples[0][0] >= 0.12:
            return dict(
                reason="FRESH_LOW_SPEED_INSIDE_NAV_RADIUS",
                distance_m=distance,
                samples=len(self.samples),
                span_s=stamp - self.samples[0][0],
                measured_v_mps=loc["v"],
                navigation_success=False,
                work_admitted=False,
            )
        return None
