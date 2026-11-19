import copy, numpy as np
from scipy.spatial.transform import Rotation, Slerp
from scipy.spatial import cKDTree
from support import stamp, pose


def transform(xyz, p, q):
    return Rotation.from_quat(q).apply(xyz) + p


def interpolate_odom(messages, t):
    before = [m for m in messages if stamp(m) <= t]
    after = [m for m in messages if stamp(m) >= t]
    if not before or not after:
        return None
    a, b = before[-1], after[0]
    ta, tb = stamp(a), stamp(b)
    if tb - ta > 0.10:
        return None
    out = copy.deepcopy(a)
    out.header.stamp.sec = int(t)
    out.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
    if tb - ta < 1e-9:
        return out
    ratio = (t - ta) / (tb - ta)
    pa, qa = pose(a)
    pb, qb = pose(b)
    xyz = pa * (1 - ratio) + pb * ratio
    q = Slerp([ta, tb], Rotation.from_quat([qa, qb]))([t]).as_quat()[0]
    out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = (
        xyz.tolist()
    )
    (
        out.pose.pose.orientation.x,
        out.pose.pose.orientation.y,
        out.pose.pose.orientation.z,
        out.pose.pose.orientation.w,
    ) = q.tolist()
    out.pose.covariance = (
        np.array(a.pose.covariance) * (1 - ratio) + np.array(b.pose.covariance) * ratio
    ).tolist()
    return out


def temporal_filter(xyz, p, q, history, t):
    world = transform(xyz, p, q)
    # Preserve broad ground support. Two previous independent observations must
    # support above-ground surfaces within 0.25 m; this is not semantic removal.
    support = np.zeros(len(xyz), np.int8)
    for old_t, old in history:
        if 0.15 < t - old_t < 3.0 and len(old):
            support += cKDTree(old).query(world, k=1)[0] < 0.25
    keep = (xyz[:, 2] < 0.18) | (support >= 2)
    history.append((t, world))
    return xyz[keep], world, keep
