# 构建与运行

工程面向 Ubuntu 24.04、ROS 2 Jazzy 与 Gazebo Harmonic。命令从仓库根目录运行。建议在新的终端只加载系统 Jazzy 和当前工程，避免继承其他机器人工程的 overlay。Gazebo 主雷达与双目相机需要可用的图形渲染设备。案例复算可以在没有渲染设备的 Linux 主机上运行。

## 建立工作区

Ubuntu 第一次安装 ROS 时，先按官方步骤配置 ROS 软件源并安装 Jazzy、开发工具。已有 Jazzy 的主机可从下面命令开始。[ROS 官方 Ubuntu 安装指南](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

```bash
git clone https://github.com/caihao1B/ROS2-Navigation.git
cd ROS2-Navigation
source /opt/ros/jazzy/setup.bash
# 若本机从未初始化过 rosdep，先执行一次 sudo rosdep init
rosdep update --rosdistro jazzy
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

rosdep 从各包 `package.xml` 安装依赖。colcon 构建本工程自己的 C++ 节点、Gazebo 和 Nav2 插件及 ROS 动作类型。`install/setup.bash` 让 ROS 能发现它们。限制并行度可以减少在普通笔记本上编译点云库时的内存压力。

与系统 ROS 配合时推荐使用 Ubuntu 的 `python3-shapely`。若主机已有 ROS 依赖，但希望在不调整系统 Python 的情况下提供几何计算库，可在 **仓库外**建立兼容环境：

```bash
python3 -m venv --system-site-packages "$HOME/.venvs/field-navigation"
source "$HOME/.venvs/field-navigation/bin/activate"
python -m pip install 'shapely>=2,<3'
export FIELD_PYTHON="$VIRTUAL_ENV/bin/python"
export PYTHONPATH="$("$FIELD_PYTHON" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')${PYTHONPATH:+:$PYTHONPATH}"
```

`--system-site-packages` 让虚拟环境能使用系统版本的 NumPy、SciPy 和 ROS Python 绑定。导出的 `PYTHONPATH` 也供使用系统解释器的 ROS 入口和 colcon 测试读取同一份 Shapely。若使用 apt 安装的依赖，可跳过这段。

## 无需启动仿真的案例

```bash
python3 tools/review_demo.py --list
python3 tools/review_demo.py --case follow-stop-resume
python3 tools/review_demo.py --case spot-spray
python3 tools/review_demo.py --case strip-mowing

mapping_share="$(ros2 pkg prefix --share field_mapping)"
ros2 run field_mapping verify_loop \
  "$mapping_share/example/observations.json" \
  "$mapping_share/example/reference.json"

colcon test --executor sequential
colcon test-result --verbose
```

| 入口 | 看什么 |
| --- | --- |
| 巡检 | 观测上下文进入原生 C++ 策略。对照树选择、实际派发与停车确认。这里重算的是决策层，同场跟踪和运动记录提供上下文。 |
| 点喷 | 从边界、安全连边和目标坐标生成访问顺序。用独立实录复算精停判据、作业时间与剂量。 |
| 割草 | 对冻结的有向段检查车体停车和关刀换向。另一场数据用于计算工具工作时扫掠与确认的面积。 |
| 回环几何 | 调用原生 RTAB-Map，检验给定重访候选的点云约束。该例与在线建图启动可分别运行。 |

## 在 Gazebo 查看巡检系统

在已加载当前 `install/setup.bash` 的终端启动。默认停车模式会运行仿真时钟和传感器、定位、导航及决策，同时让底盘门控保持停车。`output_dir` 要选空目录，便于关联同一场次的节点输出。

```bash
ros2 launch field_bringup inspection.launch.py --show-args
run_dir="$(mktemp -d)"
ros2 launch field_bringup inspection.launch.py drive:=false output_dir:="$run_dir"
```

在另一个已加载相同工作区的终端运行以下命令。观察数据前给传感器、定位与 Nav2 生命周期节点留出启动时间。

```bash
python3 tools/check_parked.py --timeout 150
ros2 topic list -t
ros2 topic echo /clock --once
ros2 topic echo /localization/odometry --once --field child_frame_id
ros2 topic echo /mission/decision --once --field data
ros2 topic echo /motion/gate --once --field data
ros2 lifecycle get /controller_server
ros2 action list -t
```

停车检查器在同一 ROS 域订阅带时间戳的真实观测，并实际询问 Nav2 生命周期。它检查存在且持续为零的底盘指令。超时会列出尚未收到的反馈并以非零码退出。加 `--json` 可导出逐项结果。

先看 `/clock` 是否推进，再看里程计、决策原因和门控反馈。行为树选中一个分支与底盘获得运动许可是两个独立事件。结束时在 launch 终端按 `Ctrl+C`，等待子节点退出后再启动下一场。想看农田和底盘时可在带桌面的 Linux 主机上添加 `gui:=true`。`drive` 控制实际下发导航的权限。本文的启动检查命令保持停车。历史整车案例的使用范围在各案例中单独注明。

无需 Gazebo 的复现还可直接用随仓观测建立新地图。[双终端回放与数据库检查](../src/field_mapping/example/README.md)

## 复现一次短程跟停驾驶

先结束停车检查的 launch，再在同一工作区开启一个新的巡检会话。场景中的前车先沿路前行，再停留并重新起步。巡检车通过传感器观测处理这次相遇。

```bash
drive_dir="$(mktemp -d)"
ros2 launch field_bringup inspection.launch.py drive:=true output_dir:="$drive_dir"
```

让任务自然结束。完成后可用下面的只读命令查看 Nav2 何时中断或接回路线、底盘是否在终点停车。作业路线取连接路的短程段。停车模式的检查器用于 `drive:=false` 会话。

```bash
python3 -m json.tool "$drive_dir/mission_result.json"
python3 - "$drive_dir/mission_events.jsonl" <<'PYTHON'
import json, sys
with open(sys.argv[1]) as stream:
    for line in stream:
        event = json.loads(line)
        if event["event"] in {"BT_INTERRUPTS_PATH", "FOLLOW_PATH_CANCEL_ACK",
                              "RESUME_REMAINING_ROUTE", "FOLLOW_PATH_SUCCESS"}:
            print(event["t"], event["event"], event.get("reason", ""))
PYTHON
```

## 查看地图

独立建图时按[建图与回环说明](../src/field_mapping/README.md)启动 `field_mapping`，用 RViz 看观测与优化轨迹。点喷和割草在本发行范围内提供专项复算、工具节点与控制插件。[作业组件及接口](../src/field_work/README.md)
