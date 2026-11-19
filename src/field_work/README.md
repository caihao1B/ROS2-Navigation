# 作业规划、底盘控制与工具反馈

## 从导航到作业

点喷规划在足迹能够通过的连边上访问离散的目标点。定位持续稳定、底盘停止、工具健康时才能执行喷洒。条带割草沿有向路径行驶。地头换挡保持刀具关闭。作业端用 SQLite 事务把带时间戳的传感器反馈持久化，在重复请求、反馈中断和进程重启后区分已经确认的作业量与需要检查的残余。

[点喷复算](../../examples/spot_spray/README.md)和[割草复算](../../examples/strip_mowing/README.md)都可以从仓库根目录运行。二者读取真实仿真记录。点序会从地图与目标重新求解，地头换向采用已冻结的路径。线上工具动作和插件可供 ROS 系统集成使用。

## 核心组件

| 组件 | 接口与控制逻辑 |
| --- | --- |
| `field_work::DirectedPathCurvature` | Nav2 `Controller` 插件。RPP 选取候选速度。本插件将后轴位置投影到最近有向路径段，根据路径曲率、横向和车头方向偏差修正角速度。修正后再次核对挡位、曲率上限和预测车身碰撞。使用时须按车型提供正值 `vehicle_max_curvature`（单位 m⁻¹）。 |
| `/work/tool_task` | 版本化的 [ToolTask](../field_interfaces/action/ToolTask.action) ROS 动作，支持到点服务、启动作业、停止作业。客户端提交幂等键、点或面、工作量和到期仿真时间，订阅执行阶段、健康与计量反馈。 |
| `work_ledger` | 订阅 `/work/context`、`/work/tool/feedback` 和 `/clock`，把最新门控、账本量和作业状态发布至 `/work/ledger/state`。当定位、反馈新鲜度、停车或刀具条件不满足时停止计量。 |
| `field_work::ToolActuator` | Gazebo 系统插件，读取真实仿真关节速度及工具状态。需要模型提供 `base_link`、`tool_link`、`tool_motor_joint` 和配置。传感反馈经桥接送到动作节点，独立效果量用于事后核对。 |

在已构建且 source 过安装环境的终端查看接口和已注册插件：

```bash
ros2 interface show field_interfaces/action/ToolTask
ros2 pkg executables field_work
ros2 plugin list | grep DirectedPathCurvature
```

若所用 ROS 命令行未安装插件列表扩展，可直接查看 `ros2 pkg prefix --share field_work` 下的 `plugins.xml`。插件测试另外使用 Nav2 的 pluginlib 加载器实际实例化控制器。集成工具动作时，以同一配置连接动作服务、反馈账本与相应作业模型。当前独立作业案例直接运行上述离线复算。

接口、工具执行器、账本和 Nav2 控制插件保留来源包的许可声明。来自任务编排与实验规划的部分 Python 模块沿用项目原有的权利声明，见仓库素材与许可说明。
