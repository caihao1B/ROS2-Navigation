import math


def curvature(left, right, c):
    L, T = c["wheelbase"], c["wheel"]["track"]
    a, b = math.tan(left), math.tan(right)
    return 0.5 * (a / (L + T * a / 2) + b / (L - T * b / 2))


def twist(qvel, q, c):
    r, T = (
        c["wheel"]["radius"],
        c.get("calibration", {}).get("effective_track_m", c["wheel"]["track"]),
    )
    v = r * (qvel[0] + qvel[1]) / 2
    return v, (
        r * (qvel[1] - qvel[0]) / T
        if c["kind"] == "differential"
        else v * curvature(q[2], q[3], c)
    )
