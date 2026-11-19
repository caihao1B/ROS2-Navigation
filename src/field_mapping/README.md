# RTAB-Map 建图与回环

农田里的长直路与大面积平地外观相似。机器人把轮速、惯导与激光里程计作为相邻时刻的运动约束，用同步的双目图像和激光表面描述地点。回到相似地点时，RTAB-Map 通过图像及空间邻近关系寻找候选，再用几何配准检验约束。优化后的位姿图把局部观测放回同一张地图。

随仓的短段观测可直接驱动建图节点生成一张新局部地图，适合先了解这条技术链。[从同场双目、点云和里程计开始](example/README.md)

## 观察建图输入

[mapping_inputs.py](scripts/mapping_inputs.py) 从 `/sensors/stereo/{left,right}/image_rect`、对应 `camera_info`、`/sensors/lidar/points` 和 `/localization/odometry` 取带时戳的观测。左右图像严格配对，点云和里程计必须落入同步窗口。激光点依照 TF 外参变换到 `base_link`，剔除自身回波并体素降采样。高出地面的表面需得到此前独立观测的支持，减少移动物体在地图上留下拖影。

建图节点按[配置](config/mapping.yaml)接收三个话题。`/mapping/input/stereo` 使用 `rtabmap_msgs/RGBDImage` 打包双目，右目图像放在 RTAB-Map 的右图槽位，由左右相机标定解释。`/mapping/input/cloud` 提供过滤后的三维表面，`/mapping/input/odom` 提供与其匹配的里程计。场地结构、运动物体脚本和仿真真值均不参与建图输入。

## 复算一次重访

在已构建并载入本仓库的终端运行以下命令。这个示例不需要 Gazebo，也不需要下载整幅地图数据库。

```bash
mapping_share="$(ros2 pkg prefix --share field_mapping)"
ros2 run field_mapping verify_loop \
  "$mapping_share/example/observations.json" \
  "$mapping_share/example/reference.json"
```

两帧点云来自一次已存档建图任务的 RTAB-Map 关键帧扫描，点已转到对应观测的车身坐标，并保留原导出器的隔点抽样。算法接收一个“回到同一位置附近”的候选，初值平移为零，初始朝向来自两帧原始里程计。C++ 调用原生 `RegistrationIcp` 从点云估计配准变换，随后才读取独立的参考约束。输出包含接受或拒绝、对应点数与变换差异。此入口验证的是已给定候选的几何检验。在线候选搜索、图优化和数据库写入由另一个 ROS 节点承担。[C++ 配准逻辑](src/verify_loop.cpp)

## 启动在线建图

打开传感器与里程计输入，再在另一个已载入工作区的终端启动。每次默认写独立的新数据库。显式指定既有 `database` 路径会接续其中的地图。

```bash
ros2 launch field_mapping mapping.launch.py
```

```bash
ros2 node info /mapping/rtabmap/mapper
ros2 topic hz /mapping/input/cloud
ros2 topic list | grep /mapping/rtabmap
rviz2 -d "$(ros2 pkg prefix --share field_mapping)/config/mapping.rviz"
```

若建图进程为唯一 `map → odom` 发布者，可在启动时设置 `publish_map_tf:=true`。巡检定位拥有该变换时沿用默认值。停车时可能只产生起始关键帧。观察图增长或回环需要有时间对齐的运动观测。完成采集后用 `Ctrl+C` 关闭，让 RTAB-Map 将数据库写完。

巡检运行另由 RTAB-Map 的局部激光 ICP、轮速与 IMU 提供连续运动估计，区域激光点云匹配与 GNSS 提供全局位置约束。两个接口分别用于建图和行驶中定位。
