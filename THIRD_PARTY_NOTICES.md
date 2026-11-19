# 源码、资源与第三方组件

本仓库的任务与车辆模型来自作者的农田机器人仿真工程。世界、基本几何车体、规划先验、区域观测地图、工程采样与作业反馈随对应功能提供。车型名称在示例中表示功能与运动学类型。未收录厂商原始模型、商标贴图、第三方行人网格或检测网络权重。

各包的 `package.xml` 保留来源许可。建图、工具动作、工具模拟和账本的来源包声明 Apache-2.0，随附[许可证文本](licenses/Apache-2.0.txt)。独立任务策略、监督器和部分控制/装配来源的许可证标记是 `LicenseRef-Project-Unspecified`。该标记未授予这些部分新的许可。混合来源的包保留对应声明。仿真装配中的轮轴反馈源自传感器适配包，场景运动源自农田世界包，全局位置监督源自全局位姿管理包，这些上游包声明 Apache-2.0。[轮轴反馈](src/field_bringup/src/shaft_feedback.cpp) · [场景运动](src/field_bringup/src/moving_obstacles.cpp) · [全局位置监督](src/field_bringup/src/global_manager.cpp)

ROS 2、Nav2、robot_localization、Gazebo、ros_gz、RTAB-Map、BehaviorTree.CPP、Eigen、PCL、OpenCV、NumPy、SciPy、Shapely 和 nlohmann/json 通过各自发行包安装，适用其上游许可证。RTAB-Map 软件包声明 BSD，BehaviorTree.CPP 声明 MIT，Nav2 与 Gazebo 桥相关包声明 Apache-2.0。发行版中安装的包说明可在其 `share/<package>/package.xml` 查看，Debian/Ubuntu 许可证位于相应包的 `/usr/share/doc/<package>/copyright`。

公开案例来自不同的仿真或离线规划任务。巡检观测、监督器记录和物理评价各自独立。点喷的路线对照与工具反馈、割草的地头停车与开刀面积分别提供样本。在发布样本中，时间使用仿真时钟，长度或面积使用米制单位。

工程图解由作者根据场地先验、激光扫描、定位与作业记录绘制。整幅 SLAM 成果图使用项目存档的地图点云与优化位姿图。重访配准、巡检跟停、点喷与割草图解使用随仓案例。车辆符号用于说明底盘方向与动作。数据文件、图例与复算入口见[工程图解](docs/demo.md)。
