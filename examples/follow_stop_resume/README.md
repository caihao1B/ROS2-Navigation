# 巡检车怎样跟随、等待和恢复

跟随同向移动目标时，行为树根据目标方向、净距和预计碰撞时间选择跟随速度。目标停住后，树会重新判断是否等待，监督器请求取消正在运行的导航动作，并检查底盘、工具和导航回执。前方条件恢复后，巡检车继续执行路径。

## 在构建好的工作区复算

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 examples/follow_stop_resume/run.py
python3 examples/follow_stop_resume/run.py --json
```

入口将观测送入 [C++ 行为树](../../src/field_navigation/include/field_navigation/policy.hpp) 和[取消与停车监督器](../../src/field_navigation/include/field_navigation/supervision.hpp)。ROS 图和 Gazebo 均可关闭。终端依次显示跟随、等待和恢复的动作及理由。机器可读输出同时列出实际控制门控速度、里程计速度和独立评估的位移。

| 随仓输入或对照 | 用途 |
| --- | --- |
| `observations.jsonl.gz` | 来自一次巡检仿真的行为树输入，在决策层记录。包括原始仿真时刻、前方目标运动、净距、路径与定位有效性、时效和停车反馈。包含从本场启动开始的前导时序，保留计时器和取消请求的历史。 |
| `recorded_decisions.jsonl.gz` | 独立对照，包括策略动作、实际指令、原因、速度上限和停车确认。仅在 C++ 决策运行结束后用来比较。 |
| `motion_feedback.json` | 另存的门控记录、估计速度和评价端采集的车体与目标位置。物理位移按各端点邻近样本线性插值，用于区分发出停车命令和车辆实际停住。 |

时间以仿真秒计，位置以米计，线速度以米每秒计。位姿评估只参与对照。压缩样例取自上层观测记录，不含原始激光扫描。按原记录保存的时间戳检查每一个采样决策。两个采样之间的实时调度由在线监督器负责。修改前车同向判据会使同一跟随时刻转为等待，标记观测过期会发出停车指令。程序在每次复算中均执行这两个输入反例。

可用 `gzip -cd examples/follow_stop_resume/observations.jsonl.gz | head -n 1` 读取首个样本。继续阅读[行为树优先级](../../src/field_navigation/config/inspection_tree.xml)及[策略阈值](../../src/field_navigation/config/inspection_policy.json)。
