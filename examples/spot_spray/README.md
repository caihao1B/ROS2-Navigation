# 点喷：从访问顺序到确认出液

差速车依次到达作业点，定位和底盘反馈均确认停稳后才允许工具工作。这个例子分别复算整组目标的离线路线，以及选定服务点的仿真观测记录。地图坐标以米计，朝向用弧度，时钟用仿真秒。

从仓库根目录运行：

```bash
python3 examples/spot_spray/run.py
```

算法使用 NumPy、SciPy 的 HiGHS 混合整数求解器及 Shapely。输出包含访问顺序、停车判据、已确认的剂量和逐点阶段。点序求解保留起终点和总路长上限，每条候选边都要容纳车体外接圆，再选择累计转角较小的连通路线。等价最优解可以具有不同的点序。

| 文件 | 来自哪里，用来复算什么 |
| --- | --- |
| `planning.json` | 从作业计划提取地图坐标、先验多边形、车体与余量。仅用这些输入重新建立可达边与求解模型。 |
| `docking-observations.json.gz` | 单个点从导航请求到停稳确认期间的实际定位和底盘门控记录。取得最近一次不晚于定位帧的门控样本，按采集时间重新评估连续停车窗。 |
| `meter-observations.json.gz` | 同一作业点的 SQLite 观测事件及账本接收时刻，保留工具流量、转速、健康标志、配对位姿和车速。带上公开的地图和工具版本重新绑定，通过账本重新计量，再重复发出目标以检验幂等。 |
| `service-observations.json.gz` | 同场连续点服务的任务事件、底盘状态变化、独立出液观测与账本记录，分别解释行驶、精停、定位等待、喷洒和其他交接时间。 |
| `reference.json` | 原记录中的独立对照。计算完再比较。 |

相关源码在 [访问顺序求解](../../src/field_work/field_work/spray_planning.py)、[点位精停](../../src/field_work/field_work/docking.py)、[工时划分](../../src/field_work/field_work/spray_phases.py) 和 [反馈账本](../../src/field_work/field_work/ledger.py)。
