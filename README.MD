# Duojin02 真机工作区

面向 **Jetson Orin Nano / Ubuntu 22.04 + ROS 2 Humble**，包含麦克纳姆底盘、IMU、N10Plus 二维雷达、建图、定位、Nav2 导航和遥控。仓库名为 `duojin02`；ROS 包、脚本、环境变量和镜像保留 `duojin01_*` 名称，命令中不要按仓库名替换它们。

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
sudo ./scripts/set_dev.sh /dev/ttyACM_CONTROLLER duojin01_controller Controller
sudo ./scripts/set_dev.sh /dev/ttyACM_LIDAR lslidar Lslidar
```

## 真机启动

每个终端先在工作区加载 `source ./source.sh` 并设置控制系统的 ROS 域。下列入口按任务选择；建图、固定图导航、在线建图导航分别拥有其底盘和雷达节点，不要与独立底盘/雷达入口叠加启动。

| 任务 | 入口 |
| --- | --- |
| 底盘节点联调，不启动雷达 | `ros2 launch duojin01_bringup base.launch.py use_lidar:=false` |
| 独立雷达观察 | `ros2 launch lslidar_driver lslidar_x10_launch.py serial_port:=/dev/lslidar` |
| 建图 | `ros2 launch duojin01_bringup mapping.launch.py` |
| 固定地图定位与导航 | `ros2 launch duojin01_bringup navigation.launch.py map:=maps/0.yaml` |
| 在线 SLAM 与导航 | `ros2 launch duojin01_bringup online_navigation.launch.py` |

无雷达联调不代表具备移动条件，缺扫描时速度保护可能停车。在线导航接收指定目标，不包含自主探索。固定图入口不传 `map` 时优先选择工作区 `maps/` 的最大数字编号；无数字地图时选修改时间最新的 YAML，不按具名地图的字母顺序。使用指定地图时显式传路径。地图需采集或提供，不随镜像附带。

固定图启动后，在 RViz 用 **2D Pose Estimate** 指定真实初始位置；待定位健康确认且 Nav2 生命周期节点 ACTIVE 后，再发送目标。失配与恢复步骤见 [定位健康与恢复](docs/localization-health.md)。所有真机入口使用系统时钟；录包时钟只用于独立离线消费者。

以下操作附着于已启动的系统，不再启动一套底盘或雷达：

```bash
# 已有建图节点，先停稳，再保存地图与位姿图
ros2 launch duojin01_bringup save_map.launch.py
# 键盘遥控；需要接管导航中的机器人时先取消目标
ros2 launch duojin01_bringup keyboard_teleop.launch.py
```

当前所有底盘参数只维护在 [chassis.yaml](src/duojin01_bringup/config/chassis.yaml)。前进、后退、横移三个速度仍可分别设置；串口、波特率、EKF、雷达裁剪、guard、SLAM 和导航组件均从总配置派生。修改后执行 `python3 scripts/chassis_config.py check` 和 `python3 scripts/chassis_config.py adopt`，具体字段及 Jetson 入口见 [唯一配置说明](docs/chassis/configuration.md)。

各 bringup 入口接收 `chassis_profile` 指向该总配置；运行时地图、回放/续建位置、显示开关和输出目录继续按单次任务指定。旧串口与局部 YAML 参数覆盖入口已移除。Nav2 内部的 `params_file` 只传递生成后的参数。`use_teleop` 控制手柄，键盘使用上方独立入口。

`DUOJIN01_WORKSPACE_ROOT` 指定数据工作区的绝对路径，导航选图和保存地图共用它；录包、离线输出和续建前缀的路径规则见 [SLAM 工作流](docs/slam-workflow.md)。雷达来源、固定协议和配置集中在 [UPSTREAM](src/lslidar_driver/UPSTREAM.md)；换硬件需验证对应驱动，不能猜协议。

## 本机与远程 RViz

观看机运行独立 RViz，加载本机架构的同版本描述包和网格；不再启动机器人入口。域 / 网络、只看雷达的固定坐标系、完整导航模板的已知差异及 QoS 排查统一见 [RViz 观察](docs/remote-rviz.md)。该页也说明直接订阅与实验分域显示的性能边界。

## 可选 Docker 环境

原生运行无需 Docker。需要隔离开发环境时，在仓库根目录执行：

```bash
./scripts/build_image.sh
DEV_LIST=/dev/duojin01_controller,/dev/lslidar ./scripts/dev.sh
./scripts/enter.sh
```

默认镜像 `duojin01:hardware-humble`、容器 `duojin01_hardware`；工作区挂载到 `/ws`，预编译安装位于 `/opt/duojin01/install`，开发构建覆盖层通过 `source.sh` 加载。容器只打开开发环境，不自动启动机器人。`DEV_LIST` 支持逗号分隔设备；`DEV`、`DEV_GLOB`、`IMAGE`、`NAME` 可覆盖相应设置。已有容器不更新映射；改设备或 GUI 设置前先正常退出其中的 ROS，再用 `./scripts/stop.sh` 删除旧容器，以新设置运行 `dev.sh`。该脚本会强制删除容器，`/ws` 外未另挂载的数据须先保存。

Orin 本地构建得到 ARM64 镜像；具备 ARM64 构建器或架构模拟器的 x86 机器可用 `PLATFORM=linux/arm64 ./scripts/build_image.sh`，默认另命名为 `duojin01:hardware-humble-arm64`。用 `docker save/load` 转移对应架构镜像，在板端按需要重命名；不可运行错误架构的产物。

本机容器 RViz 可设置 `GUI=1 GPU=1`；Jetson 上的 GPU 映射依赖宿主已安装的 NVIDIA Container Runtime。二维算法主要使用 CPU，容器不安装或更换 JetPack。构建脚本传递终端代理，`DOCKER_BUILD_NETWORK` 可设置构建网络；基础镜像拉取的代理由 Docker 服务单独配置。

## 验证边界

已构建环境可运行 `./scripts/validate_hardware.sh` 和 `./scripts/test_lidar.sh`。伪串口、合成扫描与构造录包证明软件路径，不等于真实雷达稳定性、底盘制动距离、毫米级精度或 Orin 长期实时性。集成回归在隔离容器执行，不映射物理设备：

```bash
docker run --rm --init --network none -v "$PWD":/ws -w /ws \
  duojin01:hardware-humble bash -c './scripts/test_slam.sh'
```

导航算法的测试构建与分项计时见 [导航验证](docs/navigation-validation.md)。默认硬件构建关闭 `BUILD_TESTING`；`validate_hardware.sh` 的 Python / launch 检查不能替代导航 CTest。结果按冻结版本阅读，不从旧报告推断当前性能。CAD 几何、未标定外参见 [底盘报告](docs/chassis/README.md)。自研 `duojin01_*` 包使用 GPL-3.0-only；第三方遵循各自目录许可证。
