# 用田间观测建立一张新地图

`sequence.zip` 取自同一次田间仿真的一段连续关键帧。包内有原时戳的左右相机图像、内参、相机安装位姿、车体坐标系下的激光扫描和当时的里程计估计。扫描已经过原建图前端的过滤。样例保留这些观测，用于重跑双目与激光同步、局部稳定表面筛选和 RTAB-Map 的关键帧插入。回放程序只发布传感器、里程计、TF 和时钟，地图由建图节点写入新文件。

## 启动与查看

在编译好的工作区根目录打开两个终端。两个终端选用同一个**空闲的独立 ROS 域**，与车辆或其他仿真使用不同的域。

**终端一**启动建图。临时目录用于保存这次生成的数据库。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=73 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
field_map_dir=$(mktemp -d)
ros2 launch field_mapping mapping.launch.py database:="$field_map_dir/farm.db"
```

**终端二**回放观测。终端一会出现 RTAB-Map 处理关键帧的消息。等待回放结束，待建图端输出稳定后在**终端一**按一次 Ctrl+C，将图和传感器记录保存到数据库。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=73 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 run field_mapping replay_mapping_inputs.py
```

**在终端一**确认写入的关键帧与观测。有多个关键帧写入，且库存有双目、点云和标定时，命令会列出对应的时间戳。积累新回环需要回到先前观测区域，本例展示局部行驶段的增量建图。

```bash
python3 - "$field_map_dir/farm.db" <<'PY'
import sqlite3
import sys
from pathlib import Path

source = Path(sys.argv[1]).resolve().as_uri() + "?mode=ro"
with sqlite3.connect(source, uri=True) as database:
    print("数据库检查", database.execute("PRAGMA integrity_check").fetchone()[0])
    rows = database.execute("""
        SELECT Node.id, Node.stamp FROM Node JOIN Data ON Node.id = Data.id
        WHERE length(Data.image) > 0 AND length(Data.depth) > 0
          AND length(Data.scan) > 0 AND length(Data.calibration) > 0
        ORDER BY Node.id
    """).fetchall()
    print("含完整观测的关键帧", rows)
    assert len(rows) >= 2
PY
```

## 随仓数据

解压包内的 `manifest.json` 可查看传感器标定、相机与车体位姿和每帧原始仿真秒。左右图使用 JPEG，点云使用 NumPy XYZ 米制数组。这段观测可直接从包安装目录读取，不需要旧地图数据库。目录里的 `observations.json` 与 `reference.json` 另用于[回访点云几何核对](../src/verify_loop.cpp)。
