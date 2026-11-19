#!/usr/bin/env python3
"""Read-only checks for a live, parked inspection launch. The timeout uses wall time."""

import argparse
import json
import math
import os
from pathlib import Path
import struct
import sys
import time


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def finite(*values):
    return all(math.isfinite(v) for v in values)


def live_checks(timeout):
    from ament_index_python.packages import get_package_share_directory
    import rclpy
    from geometry_msgs.msg import TwistWithCovarianceStamped
    from lifecycle_msgs.msg import State
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import Odometry, Path as RosPath
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image, Imu, NavSatFix, PointCloud2, PointField
    from std_msgs.msg import String

    profile = (
        Path(get_package_share_directory("field_bringup"))
        / "config/cloud_transport.xml"
    )
    # Fast DDS reads its transport settings when creating the participant.
    # Use the same large-sample SHM profile as the cloud publisher.
    for key in ("FASTDDS_DEFAULT_PROFILES_FILE", "FASTRTPS_DEFAULT_PROFILES_FILE"):
        os.environ[key] = str(profile)

    started = time.monotonic()
    deadline = started + timeout
    sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    tracked = {}
    failures = {}
    sampled = set()
    subscriptions = {}
    last_stamps = {}
    clock_start = None
    clock_latest = None
    last_advance = None
    last_gate = None
    last_request = None
    first_stops = {}
    counts = {}
    futures = {}
    clients = {}

    def expect(key, source):
        tracked[key] = {
            "ok": False,
            "source": source,
            "detail": "等待有效消息或服务响应",
        }

    def update(key, ok, detail):
        tracked[key].update(ok=bool(ok), detail=detail)

    def reject(key, detail):
        failures[key] = detail
        update(key, False, detail)

    def observe(key, typ, topic, validate, *, one_sample=False, sensor=False):
        expect(key, topic)

        def callback(message):
            if key in failures:
                return
            counts[key] = counts.get(key, 0) + 1
            try:
                ok, detail = validate(message)
            except (
                TypeError,
                ValueError,
                KeyError,
                IndexError,
                AttributeError,
            ) as error:
                ok, detail = False, f"消息内容无效：{error}"
            if key not in failures:
                update(key, ok, detail)
            if ok and one_sample:
                sampled.add(key)

        subscriptions[key] = node.create_subscription(
            typ, topic, callback, sensor_qos if sensor else 20
        )

    def clock_progress(msg):
        nonlocal clock_start, clock_latest, last_advance
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if clock_latest is not None and t < clock_latest:
            reject("clock", "监视期间仿真时钟倒退，请检查是否混入其他会话")
        if clock_start is None:
            clock_start = t
        if clock_latest is not None and t > clock_latest:
            last_advance = time.monotonic()
        clock_latest = t
        return t > clock_start, f"接收仿真时钟，自 {clock_start:.3f} s 推进到 {t:.3f} s"

    def sensor_payload(key, test):
        def validate(msg):
            t = stamp(msg.header)
            if t <= 0 or not msg.header.frame_id:
                return False, "数据缺少有效仿真时间或坐标系"
            if not test(msg):
                return False, f"{msg.header.frame_id} 的样本不满足数据格式或有限值检查"
            last_stamps[key] = t
            return True, f"已收到 {msg.header.frame_id} 的实测样本，仿真时间 {t:.3f} s"

        return validate

    def cloud(msg):
        fields = {f.name: f for f in msg.fields}
        coordinates = [fields.get(axis) for axis in ("x", "y", "z")]
        if (
            not all(
                f
                and f.datatype == PointField.FLOAT32
                and f.count == 1
                and 0 <= f.offset <= msg.point_step - 4
                for f in coordinates
            )
            or msg.width == 0
            or msg.height == 0
            or msg.row_step < msg.width * msg.point_step
            or len(msg.data) < msg.row_step * msg.height
        ):
            return False
        # Sample the organized raster in place: do not materialize another dense XYZ cloud.
        value = struct.Struct(">f" if msg.is_bigendian else "<f")
        for row in range(0, msg.height, max(1, msg.height // 12)):
            for col in range(0, msg.width, max(1, msg.width // 24)):
                start = row * msg.row_step + col * msg.point_step
                xyz = [
                    value.unpack_from(msg.data, start + f.offset)[0]
                    for f in coordinates
                ]
                if finite(*xyz) and any(x != 0.0 for x in xyz):
                    return True
        return False

    def image(msg):
        return (
            msg.width > 0
            and msg.height > 0
            and msg.step >= msg.width
            and bool(msg.encoding)
            and len(msg.data) >= msg.step * msg.height
        )

    def local_motion(msg):
        p = msg.pose.pose
        q = p.orientation
        return (
            msg.header.frame_id == "odom"
            and msg.child_frame_id == "base_link"
            and finite(
                p.position.x,
                p.position.y,
                q.x,
                q.y,
                q.z,
                q.w,
                msg.twist.twist.linear.x,
                msg.twist.twist.angular.z,
            )
            and q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 0.5
        )

    def global_pose(msg):
        value = json.loads(msg.data)
        pose = value.get("pose", [])
        good = (
            value.get("valid") is True
            and value.get("truth_used") is False
            and len(pose) == 3
            and finite(*pose)
            and bool(value.get("map_revision"))
            and bool(value.get("rig_revision"))
        )
        return (
            good,
            f"导航定位有效={value.get('valid')}，监督原因={value.get('reason')}",
        )

    def decision(msg):
        d = json.loads(msg.data)
        return (
            bool(d.get("action") and d.get("selected_action") and d.get("reason_code"))
            and d.get("session") == "connector-inspection",
            f"策略 {d.get('selected_action')}，派发 {d.get('action')}，原因 {d.get('reason_code')}",
        )

    def safe_stop(key, value, t):
        if (
            value.get("mode") != "STOP"
            or not finite(value["v"], value["w"])
            or value["v"] != 0
            or value["w"] != 0
        ):
            reject(
                key,
                f"曾收到非停车控制：mode={value.get('mode')} v={value.get('v')} w={value.get('w')}",
            )
            return False
        first_stops.setdefault(key, t)
        return counts[key] >= 2 and t - first_stops[key] >= 0.5

    def gate(msg):
        nonlocal last_gate
        d = json.loads(msg.data)
        valid_window = safe_stop("gate", d, d["t"])
        if d.get("allowed") is not False or "PARKED_INSPECTION" not in d.get(
            "reasons", []
        ):
            reject("gate", "未锁定 drive:=false 的停车门控")
        stopped = d.get("stopped") is True and d.get("tool_stopped") is True
        last_gate = time.monotonic()
        return (
            valid_window and stopped,
            f"零速停车；底盘停稳={stopped}，模式={d.get('mode')}",
        )

    def actuator(msg):
        nonlocal last_request
        d = json.loads(msg.data)
        if d.get("session") != "connector-inspection":
            reject("actuator", "收到来自其他会话的执行器请求")
        last_request = time.monotonic()
        valid = safe_stop("actuator", d, d["stamp"])
        return (
            valid,
            f"已接收执行器请求，模式={d.get('mode')}，序号={d.get('sequence')}",
        )

    def path(msg):
        good = (
            msg.header.frame_id == "map"
            and len(msg.poses) >= 2
            and all(finite(p.pose.position.x, p.pose.position.y) for p in msg.poses)
        )
        return good, f"参考系 {msg.header.frame_id}，路径包含 {len(msg.poses)} 个位姿"

    rclpy.init(args=[])
    node = rclpy.create_node("parked_inspection_observer")
    try:
        observe("clock", Clock, "/clock", clock_progress, sensor=True)
        channels = [
            ("lidar", PointCloud2, "/sensors/lidar/points", cloud),
            ("left_image", Image, "/sensors/stereo/left/image_rect", image),
            ("right_image", Image, "/sensors/stereo/right/image_rect", image),
            (
                "imu",
                Imu,
                "/sensors/imu/data",
                lambda m: finite(
                    m.angular_velocity.x,
                    m.angular_velocity.y,
                    m.angular_velocity.z,
                    m.linear_acceleration.x,
                    m.linear_acceleration.y,
                    m.linear_acceleration.z,
                ),
            ),
            (
                "wheel",
                TwistWithCovarianceStamped,
                "/sensors/wheel/twist",
                lambda m: finite(m.twist.twist.linear.x, m.twist.twist.angular.z),
            ),
            (
                "gnss",
                NavSatFix,
                "/sensors/gnss/fix",
                lambda m: m.status.status >= 0
                and finite(m.latitude, m.longitude, m.altitude)
                and abs(m.latitude) <= 90
                and abs(m.longitude) <= 180,
            ),
            ("local_odometry", Odometry, "/localization/odometry", local_motion),
        ]
        for key, typ, topic, check in channels:
            observe(
                key,
                typ,
                topic,
                sensor_payload(key, check),
                one_sample=True,
                sensor=True,
            )
        observe("global_reference", String, "/localization/status", global_pose)
        observe("decision", String, "/mission/decision", decision)
        observe("gate", String, "/motion/gate", gate)
        observe("actuator", String, "/motion/actuator_request", actuator)
        observe("path", RosPath, "/navigation/active_path", path, one_sample=True)
        expect("actuator_owner", "/motion/actuator_request")
        for name in (
            "map_server",
            "planner_server",
            "controller_server",
            "bt_navigator",
        ):
            service = f"/{name}/get_state"
            expect(name, service)
            clients[name] = node.create_client(GetState, service)

        next_graph = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(
                node, timeout_sec=min(0.08, max(0, deadline - time.monotonic()))
            )
            # Destroy high-bandwidth subscriptions as soon as a real payload passed.
            for key in sampled:
                sub = subscriptions.pop(key, None)
                if sub is not None:
                    node.destroy_subscription(sub)
            now = time.monotonic()
            if now >= next_graph:
                next_graph = now + 0.5
                publishers = node.get_publishers_info_by_topic(
                    "/motion/actuator_request"
                )
                owners = [
                    p.node_namespace.rstrip("/") + "/" + p.node_name for p in publishers
                ]
                if len(owners) > 1 or (owners and owners != ["/patrol_actuator_gate"]):
                    reject("actuator_owner", f"检测到非预期执行器发布端：{owners}")
                elif "actuator_owner" not in failures:
                    update("actuator_owner", len(owners) == 1, f"实际发布端：{owners}")
                for name, client in clients.items():
                    pending = futures.get(name)
                    if pending is not None and pending.done():
                        response = pending.result()
                        if response is not None:
                            state = response.current_state
                            update(
                                name,
                                state.id == State.PRIMARY_STATE_ACTIVE,
                                state.label,
                            )
                        else:
                            update(name, False, "生命周期服务未返回状态")
                        del futures[name]
                    if not client.service_is_ready():
                        update(name, False, "未发现生命周期服务")
                    elif name not in futures:
                        futures[name] = client.call_async(GetState.Request())
            if failures:
                break
            safe_channels_live = (
                last_gate is not None
                and last_request is not None
                and last_advance is not None
                and now - min(last_gate, last_request, last_advance) <= 1.0
            )
            if safe_channels_live and all(v["ok"] for v in tracked.values()):
                break

        if (
            last_gate is not None
            and time.monotonic() - last_gate > 1
            and "gate" not in failures
        ):
            update("gate", False, "结束检查时停车门控已停止更新")
        if (
            last_request is not None
            and time.monotonic() - last_request > 1
            and "actuator" not in failures
        ):
            update("actuator", False, "结束检查时执行器请求已停止更新")
        if (
            last_advance is not None
            and time.monotonic() - last_advance > 1
            and "clock" not in failures
        ):
            update("clock", False, "结束检查时仿真时钟已停止推进")
        return {
            "passed": all(v["ok"] for v in tracked.values()),
            "wall_seconds": round(time.monotonic() - started, 2),
            "checks": tracked,
            "sample_stamps_sim_s": last_stamps,
        }
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=120, help="wall-clock seconds")
    parser.add_argument(
        "--json", action="store_true", help="print a machine-readable report"
    )
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be a finite positive duration")
    try:
        result = live_checks(args.timeout)
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        result = {"passed": False, "error": str(error), "checks": {}}
    if args.json:
        print(json.dumps(result, indent=2, ensure_ascii=False))
    else:
        for key, check in result["checks"].items():
            mark = "✓" if check["ok"] else "×"
            print(f"{mark} {key}  {check['source']}  {check['detail']}")
        if "error" in result:
            print(result["error"], file=sys.stderr)
        print("停车会话检查通过。" if result["passed"] else "停车会话检查未通过。")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
