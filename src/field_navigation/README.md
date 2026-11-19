# 巡检决策与导航插件

[巡检树](config/inspection_tree.xml)自上而下检查紧急危险、配置绑定、观测新鲜度、定位、停车时间预算和已锁定的人工介入。路线受阻时，节点依次检查是否已耗尽搜索预算、是否可跟随同向目标、是否应继续等待、是否有可通行的局部替代路径，以及是否需要全局重规划。策略返回所选动作、理由、限速和候选方案的可用条件。

[决策监督器](include/field_navigation/supervision.hpp)还需处理 Nav2 取消目标的异步反馈。选择等待时，导航尚在执行就先发 `CANCEL`。导航已空闲且底盘与工具分别确认停止后才发 `WAIT`。跟随与正常行驶之间只需切换限速。超过取消时限会停车并锁定人工介入。

## 在线接口

`inspection_supervisor` 从本包 share 目录加载树和阈值。下列话题都使用 `std_msgs/msg/String` 承载 JSON，配置参数和输入生产端约定四个相同的绑定值。

| 接口 | 用途 |
| --- | --- |
| `/mission/context` | `stamp` 为仿真秒，`sequence` 为递增正整数。输入携带 `task_version`、`vehicle_hash`、`rig_revision`、`session`、定位与传感器健康、路径有效性、跟踪、时间预算及停车反馈。 |
| `/mission/decision` | `selected_action` 表示树选择的动作，`action` 表示监督器实际派发的指令。`reason_code` 与 `alternatives` 解释分支和备选方案。`speed_cap_mps` 仅在运动指令下可用，门控收到停车或取消时应制动。 |
| `/mission/acknowledge_manual_hold` | `std_srvs/srv/Trigger`。仅在停车、工具停止、导航空闲、定位和数据有效时允许显式复位。 |

里程计与控制门控各自提供执行反馈，仿真物理轨迹进入独立评价。用[跟停样例](../../examples/follow_stop_resume/README.md)可在相同观测下重算选项、派发和停车确认。
