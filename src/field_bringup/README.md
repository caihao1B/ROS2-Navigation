# 田间巡检的仿真与控制

在同一个 Gazebo 农田里，移动小车沿连接路接近工作区，周边有行人、设备、坡面与低矮障碍。机器人用本地里程计和区域点云确定位置，再从传感器观测判断前方可通行范围和目标运动。行为树据此选择动作，Nav2 产生候选速度。最终门控结合定位、障碍、停车回执和任务阶段，决定底盘实际收到的命令。

## 启动并检查一辆停驻的车

从仓库根目录完成主 README 的构建，加载 ROS 与本次构建的安装环境。仿真机需要可渲染 Gazebo OGRE 相机及三维雷达的图形环境。默认只启动服务端，传感器仍需要图形驱动。运行节点的 Python 解释器需要同时能导入 `rclpy`、NumPy、SciPy、Shapely 和 PyYAML。可用 `FIELD_PYTHON` 指定带 ROS 系统包的虚拟环境解释器。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch field_bringup inspection.launch.py --show-args
run_dir=$(mktemp -d)
printf '%s\n' "$run_dir"
ros2 launch field_bringup inspection.launch.py output_dir:="$run_dir"
```

另开已加载同一 ROS 环境与域的终端。下面的入口分别查看传感器、活动路线、行为树选择和最终门控。大点云只打印消息头，便于判断采样坐标系及时间。初始化时行为树会等待定位，配准后可从决策的 `evidence` 查看 `localized`、`stopped`、`blocked` 等条件。停车模式下，门控始终保留 `PARKED_INSPECTION` 原因。

```bash
ros2 topic echo --once --qos-reliability best_effort /sensors/lidar/points sensor_msgs/msg/PointCloud2 --field header
ros2 topic echo --once /navigation/active_path nav_msgs/msg/Path --field header
ros2 topic echo --once /mission/decision std_msgs/msg/String --field data
ros2 topic echo --once /motion/gate std_msgs/msg/String --field data
ros2 topic info --verbose /motion/actuator_request
ros2 lifecycle get /controller_server
```

等定位状态 `valid` 与通行风险均已产生，在启动终端按 `Ctrl+C`。传感器节点会写入采样摘要。Gazebo 服务终止后，使用同一终端内的 `run_dir` 读取证据。

```bash
python3 src/field_bringup/test/check_parked.py "$run_dir"
```

这项检查覆盖点云、双目、轮速、IMU、GNSS、激光里程计、地图与 GNSS 一致性、当前可通行地形、跟踪模块确认的路径前缀、行为树输出、停车回执及轮轴反馈。同时核验所有门控记录和路径派发事件。输入来自同场的节点日志。输出目录只接受空目录，前后两次启动各自保留独立记录。

## 源码入口

| 现场问题 | 数据怎样流动 | 代码 |
| --- | --- | --- |
| 看到了什么 | Gazebo 桥先在独立 ROS 域接收原始读数。噪声模型按安装标定生成带时间戳的观测。主雷达用于局部定位，近地测距和当前点云共同进入通行分析。双目带标定与相机光学坐标。 | [观测生成](nodes/sensor_observations.py)、[近地与远距点云](nodes/near_field.py)、[传感器标定](config/inspection_rig.json) |
| 车在哪里 | RTAB-Map 的 ICP 里程计从当前激光求增量，独立轮速和 IMU 估计提供初值。通过质量门限的增量进入融合。当前静态表面与冻结的区域点云配准，独立的 GNSS 创新检验监督 `map → odom`。 | [ICP 配置](config/icp.yaml)、[增量质量](nodes/icp_gate.py)、[点云配准](nodes/regional_localization.py)、[全局定位监督](src/global_manager.cpp) |
| 路能不能走 | 使用与点云采样时刻对齐的里程计计算地面、台阶、坡度与几何聚类。未知区域保留未知标记。当前路径和跟踪目标用于判断前方净距、同向运动、横穿风险及可观察的车身足迹。 | [地形计算](src/terrain.cpp)、[动态目标](nodes/track_front_target.py)、[可通行前缀](nodes/prefix_patrol.py) |
| 为什么等或绕行 | 任务调用 Nav2 `FollowPath`。当前观察下无法通行时，行为树可等待或触发受限的重新规划。替代路径交给当前观测验证，满足条件后才执行，接回时重新确认位置。 | [巡检任务](nodes/patrol_task.py)、[巡检决策](../field_navigation)、[全局规划插件](src/exact_goal.cpp) |
| 车如何停车 | 唯一底盘门控接收候选 `Twist`、行为树决策、定位、传感器与工具状态。驱动器独立检查租约、时标、轮轴反馈和看门狗，再向车轮关节发送受限转速。 | [速度门控](nodes/motion_gate.py)、[物理底盘](src/wheel_drive.cpp) |

`scenarios/farm` 包含农田、动态前车和碰撞几何。车体参数、标定、导航先验与观测地图分别存放，加载时按 `config/resources.json` 核验。行人的脚本和前车的未来轨迹属于世界侧，定位、跟踪和决策节点只订阅观测。[仿真场景](../../scenarios)

## 任务模式与退出

默认 `drive:=false`。这种模式通过活动路线和传感器生成实时决策，任务节点持续报告空闲，速度门控独立保持停车。`drive:=true` 会启用短程连接路任务并允许派发真实 Nav2 目标。新启动包的停车检查与原场次的行驶回放各有独立证据。

`gui:=true` 可打开单独的 Gazebo 图形客户端。也可以从订阅端打开 RViz，按 `map` 或 `odom` 观察路径与传感器。多个本地 ROS 工程并行时，各用独立 `ROS_DOMAIN_ID`，将 `raw_domain` 设置为尚未占用且与在线域不同的值。

Gazebo 按会话使用独立的传输分区。退出时首先调用 Gazebo 的服务器停止服务。若安装的 Gazebo 与 Boost 的二进制散列匹配已知版本，还会只在 Gazebo 服务进程内启用原项目的并发停止修复。其他 ROS 节点与启动终端使用原来的动态库环境。[服务进程管理](nodes/farm_world.py)、[停止修复实现](src/serialized_stop.cpp)
