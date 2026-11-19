"""Run a field inspection in Gazebo; the default session is physically parked."""

import json
import os
from pathlib import Path
import shutil
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    EmitEvent,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def component_exited(event, context):
    if context.is_shutdown:
        return []
    if event.returncode != 0:
        raise RuntimeError(
            f"{event.process_name} exited with status {event.returncode}"
        )
    return [EmitEvent(event=Shutdown(reason=f"{event.process_name} finished"))]


def assemble(context):
    share = Path(get_package_share_directory("field_bringup"))
    nodes = share / "nodes"
    sys.path.insert(0, str(nodes))
    from resources import verify_resources

    verify_resources()
    config = share / "config"
    scenarios = share / "scenarios"

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    simulated_driving = arg("drive") == "true"
    requested_dir = arg("output_dir")
    if requested_dir:
        work = Path(requested_dir).expanduser().resolve()
        work.mkdir(parents=True, exist_ok=True)
        if any(work.iterdir()):
            raise ValueError("Choose an empty output_dir for this inspection session.")
    else:
        work = Path(tempfile.mkdtemp(prefix="field-inspection-"))
    mission = json.loads((config / "inspection.json").read_text())
    shutil.copyfile(config / "inspection.json", work / "mission.json")
    session = "connector-inspection"
    (work / "spec.json").write_text(
        json.dumps(
            {
                "operator_hint": [21.5, 27.5, -0.41364050993828666],
                "distance_limit_m": 20.0,
                "sim_seconds": 220,
                "wall_budget": 840,
                "stationary": not simulated_driving,
            }
        )
    )
    raw_domain = int(arg("raw_domain"))
    online_domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    if raw_domain == -1:
        raw_domain = (online_domain + 1) % 100
    if raw_domain == online_domain or not 0 <= raw_domain <= 101:
        raise ValueError("Set raw_domain to a distinct ROS domain between 0 and 101.")
    plugin_path = Path(get_package_prefix("field_bringup")) / "lib"
    env = {
        "FIELD_RAW_DOMAIN": str(raw_domain),
        "GZ_PARTITION": f"field-{os.getpid()}",
        "GZ_SIM_RESOURCE_PATH": os.pathsep.join(
            [str(scenarios / "farm"), os.environ.get("GZ_SIM_RESOURCE_PATH", "")]
        ),
        "GZ_SIM_SYSTEM_PLUGIN_PATH": os.pathsep.join(
            [str(plugin_path), os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")]
        ),
        "OPENBLAS_NUM_THREADS": "1",
        "OMP_NUM_THREADS": "1",
        "PYTHONDONTWRITEBYTECODE": "1",
    }
    cloud_env = {
        **env,
        "FASTRTPS_DEFAULT_PROFILES_FILE": str(config / "cloud_transport.xml"),
        "FASTDDS_DEFAULT_PROFILES_FILE": str(config / "cloud_transport.xml"),
    }
    raw_env = {**cloud_env, "ROS_DOMAIN_ID": str(raw_domain)}
    python = os.environ.get("FIELD_PYTHON") or shutil.which("python3")
    all_actions = [LogInfo(msg=f"Inspection evidence directory: {work}")]

    def critical(action):
        all_actions.extend(
            [
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=action,
                        on_exit=component_exited,
                    )
                ),
                action,
            ]
        )

    def script(name, folder, *args, large=False):
        target = work / folder
        target.mkdir(exist_ok=True)
        critical(
            ExecuteProcess(
                name=name,
                cmd=[
                    python,
                    str(nodes / (name + ".py")),
                    "--out",
                    str(target),
                    *map(str, args),
                ],
                additional_env=cloud_env if large else env,
                output="log",
                sigterm_timeout="4",
                sigkill_timeout="6",
            )
        )

    def native(
        package,
        executable,
        *,
        parameters=None,
        remappings=None,
        name=None,
        namespace=None,
        extra_env=None,
        arguments=None,
    ):
        critical(
            Node(
                package=package,
                executable=executable,
                name=name,
                namespace=namespace,
                parameters=parameters or [],
                remappings=remappings or [],
                arguments=arguments or [],
                additional_env=extra_env or env,
                output="log",
                sigterm_timeout="4",
                sigkill_timeout="6",
            )
        )

    gz_arguments = ["gz", "sim", "--force-version", "8", "-r", "--seed", "41"]
    gz_arguments.append("-s")
    critical(
        ExecuteProcess(
            name="farm_world",
            cmd=[
                python,
                str(nodes / "farm_world.py"),
                *gz_arguments,
                str(scenarios / "farm/inspection.sdf"),
            ],
            additional_env=env,
            output="log",
            sigterm_timeout="18",
            sigkill_timeout="8",
        )
    )
    if arg("gui") == "true":
        critical(
            ExecuteProcess(
                name="farm_view",
                cmd=["gz", "sim", "--force-version", "8", "-g"],
                additional_env=env,
                output="log",
            )
        )
    clock = "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
    bridges = [
        clock,
        "/motion/encoders@std_msgs/msg/String[gz.msgs.StringMsg",
        "/motion/actuator_request@std_msgs/msg/String]gz.msgs.StringMsg",
    ]
    bridges += [
        f"/inspection/near_{d}/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
        for d in ("front", "rear", "left", "right")
    ]
    native(
        "ros_gz_bridge",
        "parameter_bridge",
        name="inspection_bridge",
        arguments=bridges,
        extra_env=cloud_env,
    )
    raw = [
        clock,
        "/sensors/raw/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        "/sensors/raw/left@sensor_msgs/msg/Image[gz.msgs.Image",
        "/sensors/raw/right@sensor_msgs/msg/Image[gz.msgs.Image",
        "/sensors/raw/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        "/sensors/raw/gnss@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
        "/sensors/raw/joints@std_msgs/msg/String[gz.msgs.StringMsg",
    ]
    native(
        "ros_gz_bridge",
        "parameter_bridge",
        name="sensor_bridge",
        arguments=raw,
        extra_env=raw_env,
    )
    for stream in ("motion", "vision"):
        script(
            "sensor_observations",
            "samples_" + stream,
            "--vehicle",
            scenarios / "vehicles/inspection.json",
            "--seed",
            "41",
            "--profile",
            "nominal",
            "--streams",
            stream,
            large=stream == "vision",
        )
    script("near_field", "near", large=True)
    script("cloud_filter", "filtered")
    script("odometry_inputs", "inputs")
    script("global_odometry", "odometry")
    for estimator in ("fused", "wheel_imu"):
        native(
            "robot_localization",
            "ekf_node",
            name="local_motion_" + estimator,
            namespace="/local_motion/" + estimator,
            parameters=[str(config / (estimator + ".yaml"))],
            remappings=[
                ("odometry/filtered", "/local_motion/raw/" + estimator),
                ("/tf", "/local_motion/internal/" + estimator + "_tf"),
            ],
        )
    script("guess_inspection", "motion_guess")
    native(
        "rtabmap_odom",
        "icp_odometry",
        name="local_motion_icp",
        namespace="/local_motion/icp",
        parameters=[str(config / "icp.yaml")],
        remappings=[
            ("scan_cloud", "/local_motion/icp/cloud"),
            ("scan", "/local_motion/unused_scan"),
            ("odom", "/local_motion/icp/raw"),
            ("odom_info", "/local_motion/icp/info"),
            ("/tf", "/local_motion/internal/icp_tf"),
        ],
    )
    script("icp_gate", "icp_gate")
    script("map_inputs", "map_inputs", large=True)
    script("regional_localization", "map_match")
    task_file = work / "mission.json"
    script("global_observations", "map_sources", "--config", task_file)
    native(
        "field_bringup",
        "global_pose_supervisor",
        parameters=[
            {"use_sim_time": True, "policy_file": str(config / "manager.json")}
        ],
    )
    script("reference_inspection", "reference", "--config", task_file)
    native(
        "field_bringup",
        "terrain_surface",
        parameters=[
            {"use_sim_time": True, "config_file": str(config / "terrain.json")}
        ],
    )
    script("risk_inspection", "risk", "--config", task_file)
    script("track_front_target", "tracking", "--config", task_file)
    script(
        "motion_gate",
        "control",
        "--config",
        task_file,
        "--session",
        session,
        *(["--drive"] if simulated_driving else []),
    )
    native(
        "field_navigation",
        "inspection_supervisor",
        parameters=[
            {
                "use_sim_time": True,
                "session": session,
                "task_version": mission["version"],
                "vehicle_hash": mission["vehicle"]["config_hash"],
                "rig_revision": mission["rig"]["revision"],
            }
        ],
    )
    navigation_params = str(config / "navigation.yaml")
    for package, name, extra in [
        (
            "nav2_map_server",
            "map_server",
            {"yaml_filename": str(scenarios / "maps/prior.yaml")},
        ),
        ("nav2_controller", "controller_server", {}),
        ("nav2_planner", "planner_server", {}),
        (
            "nav2_bt_navigator",
            "bt_navigator",
            {"default_nav_to_pose_bt_xml": str(config / "navigate.xml")},
        ),
        ("nav2_lifecycle_manager", "lifecycle_manager", {"autostart": True}),
    ]:
        native(
            package,
            name,
            name=name,
            parameters=[navigation_params, extra],
            remappings=[("cmd_vel", "/navigation/candidate_velocity")],
        )
    script("patrol_task", ".")
    return all_actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "drive",
                default_value="false",
                choices=["false", "true"],
                description="Park and report live observations by default. Enable physical path execution with true.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="false",
                choices=["false", "true"],
                description="Open the Gazebo GUI together with the server.",
            ),
            DeclareLaunchArgument(
                "raw_domain",
                default_value="-1",
                description="ROS domain for raw sensor generation; auto-select by default.",
            ),
            DeclareLaunchArgument(
                "output_dir",
                default_value="",
                description="Empty directory for timestamped node evidence.",
            ),
            OpaqueFunction(function=assemble),
        ]
    )
