import sys
from motion_consistency_patrol import assess
import sys

"""Pair actual RTAB-Map registration quality with pure-lidar odometry."""
import argparse, copy, signal
from collections import OrderedDict
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from rtabmap_msgs.msg import OdomInfo
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from support import *


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    f = (a.out / "quality.jsonl").open("w")
    rclpy.init()
    n = rclpy.create_node("local_motion_icp_gate")
    cfg = policy()["icp"]
    pub = n.create_publisher(Odometry, "/local_motion/inputs/icp", 10)
    qp = n.create_publisher(String, "/local_motion/icp/quality", 10)
    pending = OrderedDict()
    infos = OrderedDict()
    geometry = {}
    last = [-1.0, None]
    history = []
    gnss_fixes = []
    wheel_samples = []
    increments = n.create_publisher(
        TwistWithCovarianceStamped, "/local_motion/inputs/icp_increment", 10
    )
    now = [0.0]
    motion = dict(
        wheel_t=-100.0, wheel_v=None, imu_t=-100.0, gyro=None, quiet_since=None
    )

    def wheel_observed(m):
        motion.update(wheel_t=stamp(m), wheel_v=m.twist.twist.linear.x)
        wheel_samples.append((stamp(m), m.twist.twist.linear.x))
        wheel_samples[:] = [x for x in wheel_samples if x[0] > stamp(m) - 3.0]

    def imu_observed(m):
        motion.update(
            imu_t=stamp(m),
            gyro=float(
                np.linalg.norm(
                    [m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]
                )
            ),
        )

    n.create_subscription(
        TwistWithCovarianceStamped,
        "/sensors/localization/wheel",
        wheel_observed,
        qos_profile_sensor_data,
    )
    n.create_subscription(
        Imu, "/local_motion/inputs/imu", imu_observed, qos_profile_sensor_data
    )

    def fix_observed(m):
        g = json.loads(m.data)
        gnss_fixes.append(g)
        gnss_fixes[:] = [x for x in gnss_fixes if x["t"] > g["t"] - 3.0]

    n.create_subscription(String, "/global_pose/gnss", fix_observed, 20)

    def stationary(t):
        quiet = (
            abs(t - motion["wheel_t"]) < 0.20
            and abs(t - motion["imu_t"]) < 0.20
            and motion["wheel_v"] is not None
            and abs(motion["wheel_v"]) < 0.01
            and motion["gyro"] is not None
            and motion["gyro"] < 0.02
        )
        if not quiet:
            motion["quiet_since"] = None
            return False
        if motion["quiet_since"] is None:
            motion["quiet_since"] = t
        return t - motion["quiet_since"] >= 1.0

    def clock(m):
        t = m.clock.sec + m.clock.nanosec * 1e-9
        if t < now[0]:
            pending.clear()
            infos.clear()
            geometry.clear()
            history.clear()
            last[:] = [-1.0, None]
        now[0] = t

    n.create_subscription(Clock, "/clock", clock, qos_profile_sensor_data)

    def geom(m):
        d = json.loads(m.data)
        if d["t"] <= now[0] + 0.25:
            geometry.update(d)

    def emit(t, o, i):
        if t > now[0] + 0.25:
            return
        p, q = pose(o)
        cov = np.array(o.pose.covariance).reshape(6, 6)
        dt = t - last[0]
        step = 0.0
        angle = 0.0
        quaternion_valid = np.isfinite(q).all() and np.linalg.norm(q) > 0.5
        if last[1] is not None and quaternion_valid:
            prev, pq = last[1]
            step = float(np.linalg.norm(p - prev))
            angle = float(
                (Rotation.from_quat(pq).inv() * Rotation.from_quat(q)).magnitude()
            )
        valid = (
            quaternion_valid
            and not i.lost
            and i.icp_inliers_ratio >= cfg["min_inlier_ratio"]
            and np.isfinite(cov).all()
            and 0 <= np.max(np.diag(cov)) < 100
            and geometry.get("valid", False)
            and abs(t - geometry.get("t", -100)) < 0.4
        )
        if last[0] >= 0 and (
            dt <= 0
            or step > cfg["max_translation_step_m"] * max(1, dt / 0.1)
            or angle > cfg["max_rotation_step_rad"] * max(1, dt / 0.1)
        ):
            valid = False
        out = copy.deepcopy(o)
        for k in range(6):
            out.pose.covariance[k * 7] = max(
                cfg["pose_variance_floor"] if k < 3 else cfg["angular_variance_floor"],
                float(cov[k, k]),
            )
        # The pose below is diagnostic; only independently derived window increments enter the filter.
        consistency_v = None
        increment_valid = False
        window_start = None
        window_end = None
        increment_variance = None
        stationary_conflict = False
        quiet = stationary(t)
        independent_motion = dict(available=False, icp_valid=False, wheel_valid=False)
        if valid:
            if history and t - history[-1][0] > 0.6:
                history.clear()
            history.append((t, p.copy(), q.copy(), np.diag(cov).copy()))
            while len(history) > 2 and history[1][0] < t - 0.8:
                history.pop(0)
            if t - history[0][0] >= 0.5:
                pt, pp, pq, pc = history[0]
                span = t - pt
                middle = Rotation.from_quat(pq) * Rotation.from_rotvec(
                    (Rotation.from_quat(pq).inv() * Rotation.from_quat(q)).as_rotvec()
                    * 0.5
                )
                velocity = middle.inv().apply(p - pp) / span
                angular = (
                    Rotation.from_quat(pq).inv() * Rotation.from_quat(q)
                ).as_rotvec() / span
                variance = np.maximum(
                    (pc + np.diag(cov)) / span**2, [0.0025] * 3 + [0.0004] * 3
                )
                inc = TwistWithCovarianceStamped()
                inc.header = copy.deepcopy(o.header)
                inc.header.frame_id = "base_link"
                mid = (pt + t) / 2
                inc.header.stamp.sec = int(mid)
                inc.header.stamp.nanosec = round((mid - int(mid)) * 1e9)
                (
                    inc.twist.twist.linear.x,
                    inc.twist.twist.linear.y,
                    inc.twist.twist.linear.z,
                ) = velocity.tolist()
                (
                    inc.twist.twist.angular.x,
                    inc.twist.twist.angular.y,
                    inc.twist.twist.angular.z,
                ) = angular.tolist()
                for k in range(6):
                    inc.twist.covariance[7 * k] = float(variance[k])
                # Conservative bounded-window increment, not absolute pose feedback.
                stationary_conflict = quiet and (
                    np.linalg.norm(velocity) > 0.02 or np.linalg.norm(angular) > 0.03
                )
                independent_motion = assess(history, gnss_fixes, wheel_samples, t)
                if (
                    independent_motion["icp_valid"]
                    and np.linalg.norm(velocity) < 1.2
                    and np.linalg.norm(angular) < 0.8
                    and not stationary_conflict
                ):
                    increments.publish(inc)
                    consistency_v = float(velocity[0])
                    increment_valid = True
                    window_start = pt
                    window_end = t
                    increment_variance = float(variance[0])
        row = dict(
            independent_motion=independent_motion,
            window_start=window_start,
            window_end=window_end,
            increment_variance=increment_variance,
            stationary_window=quiet,
            stationary_conflict=bool(stationary_conflict),
            t=t,
            valid=bool(valid and increment_valid),
            pose_valid=bool(valid),
            increment_valid=increment_valid,
            lost=i.lost,
            inlier_ratio=i.icp_inliers_ratio,
            complexity=i.icp_structural_complexity,
            distribution=i.icp_structural_distribution,
            correspondences=i.icp_correspondences,
            processing_s=i.time_estimation,
            position=p.tolist(),
            quaternion=q.tolist(),
            pose_cov=list(out.pose.covariance),
            v=consistency_v,
            w=o.twist.twist.angular.z,
            raw_twist_v=o.twist.twist.linear.x,
            step_m=step,
            step_rad=angle,
            geometry=geometry.copy(),
        )
        msg = String()
        msg.data = json.dumps(row)
        qp.publish(msg)
        f.write(json.dumps(row) + "\n")
        if valid:
            pub.publish(out)
            last[:] = [t, (p, q)]

    def drain():
        for key in list(pending):
            if key in infos:
                emit(key, pending.pop(key), infos.pop(key))
        while len(pending) > 8:
            pending.popitem(last=False)
        while len(infos) > 8:
            infos.popitem(last=False)

    def odom(m):
        pending[round(stamp(m), 6)] = m
        drain()

    def info(m):
        infos[round(stamp(m), 6)] = m
        drain()

    def epoch(m):
        pending.clear()
        infos.clear()
        geometry.clear()
        history.clear()
        last[:] = [-1.0, None]

    n.create_subscription(String, "/local_motion/cloud_quality", geom, 10)
    n.create_subscription(
        Odometry, "/local_motion/icp/raw", odom, qos_profile_sensor_data
    )
    n.create_subscription(
        OdomInfo, "/local_motion/icp/info", info, qos_profile_sensor_data
    )
    n.create_subscription(String, "/local_motion/epoch", epoch, 10)
    stop = [False]

    def end(s, f):
        stop[0] = True

    signal.signal(signal.SIGINT, end)
    signal.signal(signal.SIGTERM, end)
    (a.out / "ready").write_text("pure lidar registration gate")
    try:
        while not stop[0]:
            rclpy.spin_once(n, timeout_sec=0.02)
    finally:
        f.close()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
