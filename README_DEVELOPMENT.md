# 真机开发接口

平台为 Jetson Orin Nano、Ubuntu 22.04、ROS 2 Humble。统一硬件只包含麦克纳姆底盘、IMU 和 N10Plus 串口雷达。

## 模块

| 包 | 职责 |
| --- | --- |
| `duojin01_base_driver` | 底盘串口、速度命令、里程计 / IMU / 电池反馈 |
| `lslidar_driver` | N10Plus 串口采集、540 格扫描和点云发布 |
| `duojin01_description` | 底盘、雷达、IMU 的 URDF 和 TF |
| `duojin01_bringup` | 启动、EKF、速度仲裁、建图、定位与 Nav2 |
| `duojin01_slam_tools` | 地图快照、位姿图保存、录包离线建图和 TF 过滤 |
| `duojin01_teleop` | 键盘遥控；手柄使用 bringup 的 Joy 配置 |
| `duojin01_navigation` | 精确终点 Lattice 插件、填充足迹与制动扫掠保护 |

相机、机械臂、AprilTag、任务包装和相关自定义消息包已移除。导航直接使用标准 Nav2 Action。外部 ROS 依赖由 rosdep 安装；SLAM Toolbox 从固定源码构建项目补丁覆盖层。

## 控制与反馈

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 导航 / 键盘速度输入 |
| `/cmd_vel_normal`、`/cmd_vel_slow` | `geometry_msgs/msg/Twist` | 手柄两档速度输入 |
| `/cmd_vel_requested` | `geometry_msgs/msg/Twist` | twist_mux 仲裁后的待验证速度 |
| `/cmd_vel_guarded` | `geometry_msgs/msg/Twist` | 制动扫掠验证后的速度 |
| `/cmd_vel_base` | `geometry_msgs/msg/Twist` | Collision Monitor 输出到真实底盘 |
| `/odom` | `nav_msgs/msg/Odometry` | 原始底盘里程计 |
| `/imu` | `sensor_msgs/msg/Imu` | IMU 反馈 |
| `/battery_voltage` | `std_msgs/msg/Float32` | 电池电压 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | EKF 里程计 |
| `/scan` | `sensor_msgs/msg/LaserScan` | laser 坐标系的 540 格扫描 |
| `/lslidar_point_cloud` | `sensor_msgs/msg/PointCloud2` | 同帧雷达点云 |
| `/motor_control` | `std_msgs/msg/Int8` | 雷达停转 `0` / 旋转 `1` |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | map 坐标系导航目标及结果 |

Nav2 启动后先通过 RViz 的 2D Pose Estimate 或 `/initialpose` 指定真实起始位置。真机入口使用系统时钟；只有独立录包回放使用 `/clock`。

未知环境可启动 `ros2 launch duojin01_bringup online_navigation.launch.py`，由 SLAM Toolbox 在线输出地图和定位，并使用同一导航与速度保护。它接收指定目标，不包含自主探索。固定地图仍使用 `navigation.launch.py map:=...`。算法选择、测试指标与限制见 [导航升级报告](docs/navigation-upgrade-review.md)。底盘速度流超过 0.25 s 未更新会发送实际串口零速度帧；这不能替代下位机独立通信超时保护。

## 配置边界

底盘几何与雷达安装按最新 CAD 维护，基准为四轮轴中心的地面投影，详见 [底盘测算报告](docs/chassis/README.md)。安装尺寸集中在 `duojin01_description/urdf/chassis_geometry.xacro`，生成依据在 `docs/chassis/measurements.json`。四轮滚子、壳体和雷达使用真实 CAD 缓存网格；URDF 用于硬件显示与 TF，不包含未经测量的质量或惯量。

雷达零度标记已确认朝车头；光学扫描高度暂用明确标记的壳体竖直中面代理，IMU 仍沿用原来的零外参，二者需要实物标定。当前导航足迹按底盘（含四轮）投影维护。

`base_serial_port`、`base_serial_baudrate`、`lidar_serial_port` 保留部署配置。里程计比例、IMU 偏置和协方差由 `base_driver.yaml` 配置；底盘几何由描述包维护，导航投影由 `nav2.yaml` 配置；SLAM、EKF、Nav2 的其他参数分别由对应 YAML 配置。底层控制器负责轮速换算，主机驱动没有轮径、轴距或轮距参数。配置由项目维护，不再给错误参数增加多层修复或降级。

雷达固定使用当前 N10Plus 的 460800 波特率、108 字节双回波和 540 格输出，角度转换固定对应本机顺时针协议。仅保留串口路径、话题、坐标系和距离过滤参数。删除其他型号、UDP / PCAP、矩阵变换、多回波输出、高精度格数和设备时间模式。数据按角度格重排，`time_increment=0`，不能据此生成逐束时间戳。

雷达原始包只在 C++ 内部使用固定长度数组，不生成或发布自定义 ROS 包消息。对外使用标准 ROS 接口。雷达子 launch 独立作用域，避免其 `params_file` 覆盖 Nav2 配置。

## 构建与数据

镜像安装路径为 `/opt/duojin01/install`。开发构建使用 `/ws/install/slam_backend` 与 `/ws/install/hardware`，通过 `source.sh` 加载；跨架构不复用构建产物。`DUOJIN01_WORKSPACE_ROOT` 控制导航选图和保存地图的共同根目录，Docker 默认 `/ws`。

从精简前的工作区升级时，应删除生成的 `build/hardware`、`install/hardware`、`log/hardware` 后运行 `scripts/build_hardware.sh`；已有安装目录会留下已删除的包。Docker 需要重新构建镜像。

地图保存、续建和离线回放见 [SLAM 工作流](docs/slam-upgrade-review.md)。测试使用容器内伪串口和构造录包，入口为 `scripts/validate_hardware.sh`、`scripts/test_lidar.sh`、`scripts/test_slam.sh`；均不连接物理设备。
