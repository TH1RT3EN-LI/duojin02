# Duojin02 仿真开发分支

`sim` 从 `main` 的 `bf733e7` 创建，已同步 `main` 的 `fd3c05a` 底盘修订。本阶段布置双臂移动机器人模型及 description 包的资产安装规则，供后续 Gazebo 测试使用。仿真启动、ROS 桥接和控制接入留待下一阶段。

| 分支 | 用途 |
| --- | --- |
| `main` | 最终使用的 hardware-only 真机工作区 |
| `sim` | 模型、仿真及测试开发；当前完成模型资产布置 |

模型使用提高精度后的 **516,650 面**版本，包含 45 个带颜色的 DAE 网格、SDF、URDF 和预览文件。目录、质量与使用说明见 [双臂移动机器人模型](src/duojin01_description/models/dual_arm_mobile/README.md)。包名沿用 `duojin01_description`，以保持现有资产 URI 和真机入口兼容。

日常真机开发使用 `duojin02` 工作目录，模型与仿真开发使用独立的 `duojin02-sim` 工作目录。共用代码修复优先进入 `main`，再在 `sim` 中合并 `main`；仿真变更保留在 `sim`。

## 仿真开发约定

仿真任务只在 `sim` 中增加 Gazebo 模型、桥接、适配节点、独立启动和配置。以 `main` 的真机接口及业务逻辑为兼容目标，保持继承的真机驱动、算法、默认启动和配置行为；由仿真层适配话题、消息、坐标系、单位和时钟。真机代码的问题在独立的真机任务中处理，再同步到 `sim`。

具体开发边界见 [AGENTS.md](AGENTS.md)，接口基准见 [README_DEVELOPMENT.md](README_DEVELOPMENT.md)。

## 真机工作区

默认启动配置沿用 `main`，面向 **Jetson Orin Nano / Ubuntu 22.04（Jammy）+ ROS 2 Humble**。包含麦克纳姆底盘串口驱动、N10Plus 雷达、建图、Nav2 导航和遥控。真机硬件配置统一，使用底盘、雷达和 IMU。

雷达基于官方 **V5.1.3_250822** 的 N10Plus 串口实现，固定上游提交 `ecd9a836`，仅保留当前 108 字节双回波协议；来源及项目补丁见 [UPSTREAM.md](src/lslidar_driver/UPSTREAM.md)。

## 最新底盘模型

真机描述已按 2026-09-21 的 CAD 更新：四个独立麦克纳姆轮使用原始滚子网格，轮毂拟合轴距约 **312.8 mm**、轮距约 **355.1 mm**、轮宽 **50.0 mm**；底盘投影约 **426.0 × 405.4 mm**。`base_footprint` 原点为四轮轴中心的地面投影，雷达转轴位于车体中心前方约 **92.5 mm**，零度标记朝车头。

Nav2 足迹采用底盘（含四轮）的保守投影。详细尺寸、预览、坐标系、生成方法与尚待实测的雷达光学高度 / IMU 外参见 [底盘测算报告](docs/chassis/README.md)。

独立 Gazebo 整机资产的底盘坐标基准、四轮轴心及碰撞尺寸、雷达 `laser` 坐标系也已同步。显示网格保持已确认的 516,650 面版本，复现方法及验证范围见 [模型同步说明](src/duojin01_description/models/dual_arm_mobile/README.md#与-main-的底盘同步)。

## Docker：构建和运行

在仓库根目录执行：

```bash
./scripts/build_image.sh
```

生成 `duojin01:hardware-humble`。镜像从 `ros:humble-ros-base-jammy` 构建，包含已编译的全部 6 个真机 ROS 包；进入容器后即可使用 ROS 命令。镜像安装所需的 ROS 和硬件依赖，不添加 OSRF 软件源。

Docker 将包依赖清单与源码编译分层缓存。固定源码的 SLAM Toolbox 2.6.10 覆盖层包含旋转筛选、Ceres 线程数与离线队列检查补丁，详见 [SLAM 升级与验证](docs/slam-upgrade-review.md)。编译默认 2 作业、1 包并行，适合从 Orin 开始联调。

在 Orin 上直接构建会生成 ARM64 镜像。在具备 ARM64 构建器或架构模拟器的 x86 开发机上可执行：

```bash
PLATFORM=linux/arm64 ./scripts/build_image.sh
# x86 交叉构建默认使用 duojin01:hardware-humble-arm64，避免覆盖本机镜像

# 从开发机导出 ARM64 镜像
docker save -o duojin01-hardware-humble-arm64.tar duojin01:hardware-humble-arm64
# 在 Orin 上导入，并设置开发脚本的默认镜像名
docker load -i duojin01-hardware-humble-arm64.tar
docker tag duojin01:hardware-humble-arm64 duojin01:hardware-humble
```

镜像的架构必须匹配运行设备。跨机器只传源码、录包、地图及对应架构镜像；不复用另一种架构的 `build/`、`install/`。Jetson 无需 GPU 运行此二维 SLAM；`GPU=1` 时使用已安装的 NVIDIA Container Runtime。系统和驱动由宿主机的 JetPack 提供，容器不安装或更换 JetPack。

构建脚本会传递当前终端的代理变量；Linux 上的本地回环代理会自动使用构建主机网络，也可用 `DOCKER_BUILD_NETWORK` 指定网络。基础镜像拉取由 Docker 服务负责，如这一阶段超时，应配置 Docker 服务的网络代理，或先通过 `docker load` 导入官方基础镜像。代理配置不会写入生成镜像的运行环境。

底盘和雷达使用稳定设备名：

```bash
DEV_LIST=/dev/duojin01_controller,/dev/lslidar \
  ./scripts/dev.sh
./scripts/enter.sh
```

容器名默认 `duojin01_hardware`，工作区挂载到 `/ws`，地图保存到宿主机仓库的 `maps/`。容器启用 `--init` 回收退出的子进程。启动容器只打开开发环境，不会自动启动机器人节点或发送运动命令。

需要本机 RViz 窗口时增加 `GUI=1 GPU=1`。`DEV_LIST` 支持逗号分隔的设备列表，`DEV` 支持单设备，`DEV_GLOB` 支持设备通配符。明确指定的设备不存在时会报错，需按实际连接情况调整列表。

```bash
# 示例：启用本机图形界面
GUI=1 GPU=1 \
  DEV_LIST=/dev/duojin01_controller,/dev/lslidar \
  ./scripts/dev.sh

# 停止并删除本分支的开发容器
./scripts/stop.sh

# 转移镜像到另一台真机
# docker save -o duojin01-hardware-humble.tar duojin01:hardware-humble
# docker load -i duojin01-hardware-humble.tar
```

修改设备映射后应先停止旧容器，再重新创建。`IMAGE` 和 `NAME` 可以覆盖默认镜像和容器名。默认 `ROS_DOMAIN_ID=20`，参与同一真机系统的所有终端应使用相同值；可在创建容器前设置其他值。

镜像中的工作区位于 `/opt/duojin01/install`。如需修改源码并在开发容器中重编译：

```bash
./scripts/build_hardware.sh
source ./source.sh
```

开发构建先生成 `install/slam_backend`，再将 6 个真机包构建到独立的 `build/hardware`、`install/hardware`、`log/hardware`。再次进入容器会优先加载这两个覆盖层；架构标记不匹配时会提示重新构建。

## Ubuntu 22.04 原生构建

先安装 ROS 2 Humble，然后运行：

```bash
./setup_ubuntu22.sh
./scripts/build_hardware.sh
source ./source.sh
```

ROS 安装说明：[官方 Ubuntu Debian 安装文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)。

串口设备需要当前用户具备 `dialout` 组权限。稳定设备名可通过 `scripts/set_dev.sh` 为实际设备建立，例如：

```bash
sudo ./scripts/set_dev.sh /dev/ttyACM0 duojin01_controller Controller
sudo ./scripts/set_dev.sh /dev/ttyACM1 lslidar Lslidar
```

## 真机启动入口

```bash
# 底盘联调：包含导航与手柄速度仲裁
ros2 launch duojin01_bringup base.launch.py use_lidar:=false

# 独立雷达，不自动启动 RViz
ros2 launch lslidar_driver lslidar_x10_launch.py serial_port:=/dev/lslidar

# 建图
ros2 launch duojin01_bringup mapping.launch.py

# 保存栅格图、位姿图和校验清单到工作区 maps/；建图节点需已启动，先停稳机器人
ros2 launch duojin01_bringup save_map.launch.py

# 导航，默认从 maps/ 选择最大的数字编号地图
ros2 launch duojin01_bringup navigation.launch.py
# 或显式指定地图；相对路径从工作区根目录解析
ros2 launch duojin01_bringup navigation.launch.py map:=maps/0.yaml

# 键盘控制
ros2 run duojin01_teleop keyboard_teleop_node
```

所有真机启动入口固定使用系统时钟。独立离线回放入口使用录包时钟，操作见 [SLAM 升级与验证](docs/slam-upgrade-review.md)。导航启动后需在 RViz 中用 **2D Pose Estimate** 指定真实初始位置，然后发送目标点。地图不是随镜像附带的预设场景，需要从真实环境采集或指定已有地图。

常用启动参数：

| 参数 | 含义 |
| --- | --- |
| `base_serial_port` | 底盘串口，默认 `/dev/duojin01_controller` |
| `base_serial_baudrate` | 底盘波特率，默认 `115200` |
| `base_params_file` | 底盘尺度、IMU 偏置及速度方差配置 |
| `slam_params_file` | 在线 / 离线 SLAM 参数配置 |
| `pose_graph` / `map_start_pose` | 续建的位姿图及真实起始位置 |
| `lidar_serial_port` | 雷达串口，默认 `/dev/lslidar`；底盘、建图和导航入口均支持 |
| `use_teleop` | 手柄链路开关，与雷达开关独立 |
| `use_foxglove` | Foxglove Bridge 开关 |
| `use_rviz` | 建图/导航的 RViz 开关 |
| `map` / `params_file` | 导航地图及 Nav2 参数文件 |

无显示器时可设置 `use_rviz:=false`；不使用 Foxglove 时设置 `use_foxglove:=false`。可通过 `DUOJIN01_WORKSPACE_ROOT` 指定数据工作区的绝对路径，导航选图和保存地图使用相同根目录。

雷达配置为 [`duojin01_n10plus.yaml`](src/lslidar_driver/lslidar_driver/config/duojin01_n10plus.yaml)，保留串口、话题、坐标系和距离过滤配置。协议、460800 波特率、顺时针角度转换及 540 格输出固定对应当前 10 Hz N10Plus；每格取最近有效回波。换用其他硬件时应修改对应驱动，不通过兼容分支猜测协议。

## 验证

在硬件 Docker 镜像或已构建的原生环境执行：

```bash
./scripts/validate_hardware.sh

# 编译并运行雷达协议与数据发布测试，使用独立测试构建目录
./scripts/test_lidar.sh
```

验证包依赖边界、真实时钟配置、URDF/网格完整性、速度仲裁连接、地图路径以及全部启动入口的参数解析；不会打开物理设备。当前精简及回归结果见 [本轮减法记录](docs/hardware-simplification.md)。伪串口与构造录包证明软件路径可执行，真实雷达采集、地图精度和 Orin 实时性能仍需要板端验证。

模块和标准接口见 [README_DEVELOPMENT.md](README_DEVELOPMENT.md)。雷达来源和裁剪范围见 [UPSTREAM.md](src/lslidar_driver/UPSTREAM.md)。自研 `duojin01_*` 包使用 GPL-3.0-only；第三方驱动遵循各自目录的许可证。

录包、独立离线建图与位姿图续建的具体命令、验证记录和算法升级顺序见 [SLAM 升级与验证](docs/slam-upgrade-review.md)。集成回归测试使用容器内伪串口和构造录包：

```bash
# 在隔离网络的容器内执行，不映射实际设备
docker run --rm --init --network none -v "$PWD":/ws -w /ws \
  duojin01:hardware-humble bash -c './scripts/test_slam.sh'
# QEMU 跨架构验证时可设置 SLAM_PROBE_RATE=0.1、SLAM_PROBE_DRAIN_TIMEOUT=120
```
