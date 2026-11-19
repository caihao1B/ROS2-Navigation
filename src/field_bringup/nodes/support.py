from resources import CONFIG, read
import yaml, math, json
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation


def dump(p, d):
    p = Path(p)
    p.parent.mkdir(parents=True, exist_ok=True)
    q = p.with_name(p.name + ".tmp")
    q.write_text(json.dumps(d, ensure_ascii=False, indent=2) + "\n")
    q.replace(p)


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def set_stamp(m, t):
    m.header.stamp.sec, m.header.stamp.nanosec = divmod(round(t * 1e9), 10**9)


def quat(m):
    return np.array([m.x, m.y, m.z, m.w])


def euler(q):
    return Rotation.from_quat(q).as_euler("xyz")


def wrap(x):
    return (x + math.pi) % (2 * math.pi) - math.pi


def pose(m):
    return np.array(
        [m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]
    ), quat(m.pose.pose.orientation)


def policy():
    return yaml.safe_load((CONFIG / "odometry.yaml").read_text())
