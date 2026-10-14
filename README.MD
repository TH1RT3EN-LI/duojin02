# Duojin02 真机工作区

面向 **Jetson Orin Nano / Ubuntu 22.04 + ROS 2 Humble**，包含麦克纳姆底盘、IMU、N10Plus 二维雷达、建图、定位、Nav2 导航和遥控。6 个自研 ROS 包及其代码、资源、环境变量和镜像统一使用 `duojin02` 名称；第三方雷达包保持 `lslidar_driver`。

从旧命名版本升级时，先按[名称迁移指南](docs/package-renaming.md)更新构建产物、设备规则和外部调用。当前入口只接受新名称。

导航通过一个 `use_gpu` 参数选择几何加速，默认 `true`；`use_gpu:=false` 使用完整 CPU 路径。两种模式沿用同一套 Nav2 MPPI 优化器与导航参数；CUDA 能力、构建和验证见 [GPU / CPU 统一入口](docs/gpu-cpu.md)。

本页是构建与启动入口；模块、指令链路见 [开发接口](README_DEVELOPMENT.md)，操作说明与历史试验的阅读顺序见 [文档索引](docs/README.md)。工作树、main、设备部署是不同状态，历史报告的通过结果不能证明当前版本已合并或部署。

## Ubuntu 22.04 原生构建（Jetson 首选）

先安装 [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)，在仓库根目录执行：

```bash
./setup_ubuntu22.sh
./scripts/build_hardware.sh
source ./source.sh
export ROS_DOMAIN_ID=20
```

项目脚本先构建带项目补丁的 SLAM Toolbox 2.6.10 到 `install/slam_backend`，再将 **7 个真机包**构建到 `build/hardware`、`install/hardware`、`log/hardware`。普通 `colcon build` 或 apt 原版 SLAM 库不能替代这个覆盖层。构建默认 2 作业、1 包并行，来源与补丁边界见 [SLAM 工作流](docs/slam-workflow.md)。跨机器可传源码、录包、地图；跨架构不复用 `build/` 和 `install/`。

串口用户需要 `dialout` 组权限，新增组成员资格后重新登录。先按设备身份确认底盘和雷达，再用 `scripts/set_dev.sh` 建立设备名。脚本要求设备提供厂商、产品及序列号，缺字段会退出；已有同名规则会跳过，不会替换绑定。下面的路径是占位符，必须替换；`ttyACM0/1` 的枚举顺序不能用于判断身份。

```bash
sudo ./scripts/set_dev.sh /dev/ttyACM_CONTROLLER duojin02_controller Controller
sudo ./scripts/set_dev.sh /dev/ttyACM_LIDAR lslidar Lslidar
```

## 真机启动

每个终端先在工作区加载 `source ./source.sh` 并设置控制系统的 ROS 域。下列入口按任务选择；建图、固定图导航、在线建图导航分别拥有其底盘和雷达节点，不要与独立底盘/雷达入口叠加启动。

| 任务 | 入口 |
| --- | --- |
| 底盘节点联调，不启动雷达 | `ros2 launch duojin02_bringup base.launch.py use_lidar:=false` |
| 独立雷达观察 | `ros2 launch lslidar_driver lslidar_x10_launch.py serial_port:=/dev/lslidar` |
| 建图 | `ros2 launch duojin02_bringup mapping.launch.py` |
| 固定地图定位与导航 | `ros2 launch duojin02_bringup navigation.launch.py map:=maps/0.yaml` |
| 在线 SLAM 与导航 | `ros2 launch duojin02_bringup online_navigation.launch.py` |

无雷达联调不代表具备移动条件，缺扫描时速度保护可能停车。在线导航接收指定目标，不包含自主探索。固定图入口不传 `map` 时优先选择工作区 `maps/` 的最大数字编号；无数字地图时选修改时间最新的 YAML，不按具名地图的字母顺序。使用指定地图时显式传路径。地图需采集或提供，不随镜像附带。

固定图启动后，在 RViz 用 **2D Pose Estimate** 指定真实初始位置；待定位健康确认且 Nav2 生命周期节点 ACTIVE 后，再发送目标。失配与恢复步骤见 [定位健康与恢复](docs/localization-health.md)。所有真机入口使用系统时钟；录包时钟只用于独立离线消费者。

以下操作附着于已启动的系统，不再启动一套底盘或雷达：

```bash
# 已有建图节点，先停稳，再保存地图与位姿图
ros2 launch duojin02_bringup save_map.launch.py
# 键盘遥控；需要接管导航中的机器人时先取消目标
ros2 run duojin02_teleop keyboard_teleop_node
```

常用参数的外层显式声明范围如下；子 launch 的声明、转发与作用域另需核对：

| 参数 | `base` | `mapping` | `navigation` | `online_navigation` |
| --- | --- | --- | --- | --- |
| `base_serial_port`、`lidar_serial_port` | 声明 | 声明 | 声明 | 声明 |
| `use_teleop`、`use_foxglove` | 声明 | 声明 | 声明 | 声明 |
| `base_params_file`、`base_serial_baudrate`、`odom0`、`imu0` | 声明 | 声明 | 声明 | 声明 |
| `use_lidar`、`publish_robot_model`、`publish_joint_states` | 声明 | — | — | — |
| `use_rviz` | — | 声明 | 声明 | — |
| `rviz_config` | — | 声明 | — | — |
| `slam_params_file`、`pose_graph`、`map_start_pose`、`start_at_dock` | — | 声明 | — | 声明 |
| `map`、`autostart` | — | — | 声明 | — |
| `params_file`（Nav2） | — | — | 声明 | 声明 |

表中名称省略 `.launch.py`，`—` 只表示外层未声明，不证明嵌套入口无法接收同名参数。`use_teleop` 控制手柄节点，键盘遥控是上方独立入口。建图 / 固定图无显示器时传 `use_rviz:=false`；底盘及在线导航不自动启动 RViz。已构建环境可用 `ros2 launch duojin02_bringup mapping.launch.py --show-args` 查看声明，检查其他入口时替换文件名；参数被命令行接受仍需核对实际节点有效值。

`DUOJIN02_WORKSPACE_ROOT` 指定数据工作区的绝对路径，导航选图和保存地图共用它；录包、离线输出和续建前缀的路径规则见 [SLAM 工作流](docs/slam-workflow.md)。雷达来源、固定协议和配置集中在 [UPSTREAM](src/lslidar_driver/UPSTREAM.md)；换硬件需验证对应驱动，不能猜协议。

持久调参只在[参数维护表](README_DEVELOPMENT.md#配置与模型边界)列出的入口进行。在线导航的地图分辨率和控制集由实际 `slam_params_file` 派生；`nav_log_level` 控制在线 Nav2 日志。停稳时间 `settle_time` 是启动参数，修改配置后重新配置 / 启动节点。

## 本机与远程 RViz

观看机运行独立 RViz，加载本机架构的同版本描述包和网格；不再启动机器人入口。域 / 网络、只看雷达的固定坐标系、完整导航模板的已知差异及 QoS 排查统一见 [RViz 观察](docs/remote-rviz.md)。该页也说明直接订阅与实验分域显示的性能边界。

## 可选 Docker 环境

原生运行无需 Docker。需要隔离开发环境时，在仓库根目录执行：

```bash
./scripts/build_image.sh
DEV_LIST=/dev/duojin02_controller,/dev/lslidar ./scripts/dev.sh
./scripts/enter.sh
```

默认镜像 `duojin02:hardware-humble`、容器 `duojin02_hardware`；工作区挂载到 `/ws`，预编译安装位于 `/opt/duojin02/install`，开发构建覆盖层通过 `source.sh` 加载。容器只打开开发环境，不自动启动机器人。`DEV_LIST` 支持逗号分隔设备；`DEV`、`DEV_GLOB`、`IMAGE`、`NAME` 可覆盖相应设置。已有容器不更新映射；改设备或 GUI 设置前先正常退出其中的 ROS，再用 `./scripts/stop.sh` 删除旧容器，以新设置运行 `dev.sh`。该脚本会强制删除容器，`/ws` 外未另挂载的数据须先保存。

Orin 本地构建得到 ARM64 镜像；具备 ARM64 构建器或架构模拟器的 x86 机器可用 `PLATFORM=linux/arm64 ./scripts/build_image.sh`，默认另命名为 `duojin02:hardware-humble-arm64`。用 `docker save/load` 转移对应架构镜像，在板端按需要重命名；不可运行错误架构的产物。

本机容器 RViz 可设置 `GUI=1 GPU=1`；Jetson 上的 GPU 映射依赖宿主已安装的 NVIDIA Container Runtime。二维算法主要使用 CPU，容器不安装或更换 JetPack。构建脚本传递终端代理，`DOCKER_BUILD_NETWORK` 可设置构建网络；基础镜像拉取的代理由 Docker 服务单独配置。

## 验证边界

已构建环境可运行 `./scripts/validate_hardware.sh` 和 `./scripts/test_lidar.sh`。伪串口、合成扫描与构造录包证明软件路径，不等于真实雷达稳定性、底盘制动距离、毫米级精度或 Orin 长期实时性。Docker 集成检查使用 `testing` 目标，包含与当前补丁对应的冻结 SLAM 源码；普通运行镜像不携带该测试源码。先完成镜像构建，再在断网、无设备映射、源码只读的容器中执行：

```bash
IMAGE=duojin02:hardware-humble-testing ./scripts/build_image.sh --target testing
validation_output=$(mktemp -d /tmp/duojin-hardware-check.XXXXXX)
docker run --rm --init --network none --cap-drop ALL --user "$(id -u):$(id -g)" \
  -v "$PWD":/ws:ro -v "$validation_output":/check -w /ws \
  -e PYTHONDONTWRITEBYTECODE=1 -e PYTEST_ADDOPTS='-p no:cacheprovider' \
  -e SLAM_TEST_ROOT=/check/slam -e NAVIGATION_TEST_ROOT=/check/navigation \
  -e ROS_LOG_DIR=/check/ros-logs \
  duojin02:hardware-humble-testing bash -c \
  './scripts/validate_hardware.sh && ./scripts/test_slam.sh && ./scripts/test_navigation.sh'
```

导航算法的测试构建与分项计时见 [导航验证](docs/navigation-validation.md)。默认硬件构建关闭 `BUILD_TESTING`；`validate_hardware.sh` 的 Python / launch 检查不能替代导航 CTest。结果按冻结版本阅读，不从旧报告推断当前性能。CAD 几何、未标定外参见 [底盘报告](docs/chassis/README.md)。自研 `duojin02_*` 包使用 GPL-3.0-only；第三方遵循各自目录许可证。

2026-10-13 的结构清理、算法保留范围和参数入口见[维护记录](docs/maintenance-cleanup.md)。
