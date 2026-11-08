# Duojin02 本机 CUDA 版本

当前代码基线、实验状态和证据位置见 [工作区状态](docs/current-workspace-status.md)。此目录包含尚未合并的实验代码；不要把版本号本身当作整机验收结论。

[文档索引](docs/README.md) · [目录与 Git 协作](CONTRIBUTING.md) · [测试说明](tests/README.md)

版本 `0.3.0-local-cuda.1`。本目录在上一版本基础上继续实现 **MPPI 优化器与 AMCL 激光观测更新的 CUDA 路径**，目标为本机 RTX 5070 / x86_64 / ROS 2 Humble。源码固定到 Nav2 `1.1.20`，两个独立包避免与系统安装的 Nav2 库混用。

- MPPI：GPU 高斯采样、速度/位姿展开、控制代价、softmax、加权控制更新，以及五类 critic；地图碰撞和路径对齐评分仍用原生 CPU 实现。
- AMCL：GPU 粒子×光束评分，支持当前 `likelihood_field_prob`、普通 `likelihood_field` 和 beam skipping；归一化、运动模型、KLD 重采样保留 CPU 实现。
- 原有几何 CUDA、制动复验和下游速度保护继续生效。详见 [实现、验证与边界](docs/cuda-local-v3.md)，上一版本证据见 [v2 记录](docs/cuda-local-v2.md)。

2026-10-16 漂移修正已接入扫描时间/逐束几何、地图保存、图诊断与 AMCL 数值路径；配置、构建和验收边界见 [GPU 漂移修正实现](docs/drift-remediation-gpu.md)。

本机已有 `duojin02:hardware-humble` 依赖镜像时，在本目录执行：

```bash
python3 scripts/validate_cuda_algorithms.py --architecture 120 --jobs 2 --frames 60
```

该入口构建当前源码、运行原生测试、检查真实插件和定位节点，将新记录保存到 `output/cuda-local-v3/` 的独立目录。安装前缀为 `install/cuda-local-v3/`。在同一 ROS/CUDA 容器中可单独构建及选择此版本：

```bash
./scripts/build_slam_backend.sh
./scripts/build_cuda_local.sh
source ./scripts/source_cuda_local.sh
```

CUDA 入口会校验本地六个组件包的源码清单与编译安装记录，并核对本地 SLAM 后端的版本及补丁清单；不匹配时会拒绝加载并提示重建。此校验不涵盖系统依赖、二进制篡改或直接绕过此入口的其他启动方式。修改后先构建 SLAM 后端，再构建 CUDA 组件。普通 `source ./source.sh` 是原硬件环境入口，不会选择或核验 CUDA 本地覆盖层。

导航中的被动任务采集器默认在 `ROS_LOG_DIR`（未设时为 `~/.ros/log`）创建唯一的 `task-goal-*.jsonl`，启动日志会显示文件路径，包含目标、地图身份、命令、里程计和诊断。长时间运行会持续占用磁盘；按需要归档或清理。独立运行时可用 `--output 新文件路径` 指定位置，或用 `--no-record` 只发布在线目标元数据。文件创建失败会明确报错，不会伪称已保存；直接 Action 请求的原始目标并不会自动记录。

算法待确认事项及历史证据见 [当前算法问题清单](docs/algorithm-issues-20261106.md)。

`use_gpu:=false` 将几何、MPPI 和 AMCL 全部切为 CPU。开启时分别使用 YAML 的 `optimizer_backend` 与 `observation_backend`：`auto` 允许无设备回退，`cuda` 强制设备可用，`cpu` 固定 CPU。默认确定性采样种子为 42；`optimizer_seed: -1` 可保留 Nav2 的 CPU 噪声生成器，此时仍可加速展开和更新。

以下硬件操作说明沿用原项目。本次验证范围是本机组件与合成输入回放，Jetson、实物底盘和完整真实环境导航需要另行验证。

## 原项目硬件操作说明

适用于 Jetson Orin Nano、Ubuntu 22.04、ROS 2 Humble，包含麦克纳姆底盘、IMU、N10Plus 雷达、建图、定位、Nav2 导航和遥控。

## 安装与构建

Jetson 直接原生安装和运行，无需 Docker。先安装 [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)，在仓库根目录执行：

```bash
./setup_ubuntu22.sh
./scripts/build_hardware.sh
source ./source.sh
```

安装和构建共用一套依赖声明、系统 Python 和固定版本 CMake；Jetson 沿用 JetPack 的 CUDA。构建脚本会准备项目补丁版 SLAM Toolbox 和全部 10 个 ROS 包。跨架构只传源码、地图和录包，重新生成 `build/`、`install/`。旧版本升级时，先移走旧构建产物，再构建并打开新终端。

串口用户需有 `dialout` 组权限，新增组成员后重新登录。按设备身份确认底盘和雷达，将下面的占位路径替换为实际设备；不要按串口枚举顺序判断：

```bash
sudo ./scripts/set_dev.sh /dev/ttyACM_CONTROLLER duojin02_controller Controller
sudo ./scripts/set_dev.sh /dev/ttyACM_LIDAR lslidar Lslidar
```

设备须提供厂商、产品和序列号。已有同名规则不会被脚本覆盖，升级时需核对原规则绑定。

## 启动与遥控

每个终端先执行 `source ./source.sh`。默认 ROS 域为 20；已有 `ROS_DOMAIN_ID` 设置会保留，多机使用时保持一致。

| 任务 | 命令 |
| --- | --- |
| 底盘联调，不启动雷达 | `ros2 launch duojin02_bringup base.launch.py use_lidar:=false` |
| 独立观察雷达 | `ros2 launch lslidar_driver lslidar_x10_launch.py serial_port:=/dev/lslidar` |
| 建图 | `ros2 launch duojin02_bringup mapping.launch.py` |
| 固定地图导航 | `ros2 launch duojin02_bringup navigation.launch.py map:=maps/site_a.yaml` |
| 在线建图与导航 | `ros2 launch duojin02_bringup online_navigation.launch.py` |

建图、固定图导航、在线导航三选一，它们已启动底盘和雷达。地图需自行采集或提供；在线导航接收目标，不包含自主探索。固定图启动后，在 RViz 用 **2D Pose Estimate** 设置机器人所在的大致位置和朝向，检查扫描与地图对齐，等待扫描、TF 和导航节点就绪后发送目标。

固定地图导航默认会围绕人工给定的粗略起点进行局部搜索，再由 AMCL 跟踪；地图本身保持不动。默认搜索地图 X、Y 各 ±2 m，方向覆盖一周，搜索范围并不代表精度或纠错成功保证。初始化时保持机器人停止，导航运行中需先取消目标并停车后再重新设置起点。细节、等待状态与当前验证边界见 [人工粗略起点与自动定位修正](docs/local-initialization.md)。

建图和固定图导航在无显示器时加 `use_rviz:=false`；在线导航不自动启动 RViz。导航默认尝试配置的 GPU 几何、MPPI 和 AMCL 加速，无 CUDA 能力时自动策略回退 CPU；加 `use_gpu:=false` 可指定 CPU，两者保留相同的轨迹规模、传感器和运动参数。其他启动参数可用 `ros2 launch duojin02_bringup mapping.launch.py --show-args` 查看，替换文件名即可检查其他入口。

底盘、建图、固定图导航和在线导航入口均支持 `lidar_yaw_deg`：雷达相对底盘的水平角，单位为度，从上方看逆时针为正；当前默认 `3.7` 是暂定拟合值。例如建图时指定 `ros2 launch duojin02_bringup mapping.launch.py lidar_yaw_deg:=3.7`；传入 `0` 可临时恢复原始零角度。该参数影响机器人模型发布的雷达 TF，修改后需重启入口；标定范围见 [雷达水平角记录](docs/lidar-yaw-calibration-20261107.md)。

在已运行的系统上另开终端启动键盘遥控：

```bash
ros2 run duojin02_teleop keyboard_teleop_node
```

按住键盘运动键或手柄使能键接管，松开后交回导航。仅仍在运行的目标会继续；已失败的目标需重新发送。键盘 `X` 只归零键盘输入，不是全局急停。

键盘和手柄遥控直接进入速度仲裁，不经过雷达避障或定位检查；操作时由操作者确认周围空间。导航指令仍经过速度保护，传感器、TF 或指令断流及制动路径碰撞会使导航减速或停车；诊断话题为 `/navigation/guard_status`。遥控输入停止后保留指令超时停车；如导航目标仍在运行，释放遥控后仍会交回导航。键盘 `X` 仅停止键盘输入，不是全局急停。软件停车输出不代表已验证实物制动距离，仍需下位机通信超时与实物验收。

## 地图与录包

建图完成后先停稳，再保存；同名快照会拒绝覆盖。保存成功后停止建图入口，再启动固定图导航：

```bash
ros2 launch duojin02_bringup save_map.launch.py map_name:=site_a
```

默认输出到工作区 `maps/`：`.yaml` 与图像用于导航，`.posegraph` 与 `.data` 用于同版本后端续建。需要续建时，停止原入口，将示例坐标替换为机器人在旧地图中的真实起点：

```bash
ros2 launch duojin02_bringup mapping.launch.py \
  pose_graph:="$PWD/maps/site_a" map_start_pose:='[1.2, 0.5, 0.0]'
```

底盘和雷达运行后可录包，按 `Ctrl+C` 正常结束；录包路径必须是新目录。离线建图使用独立 ROS 域，不启动真机节点：

```bash
./scripts/record_slam.sh bags/run01

ros2 run duojin02_slam_tools offline_mapping "$PWD/bags/run01" \
  --odom-source recorded --output-dir "$PWD/maps" --map-name run01
```

需要重新运行 EKF 时将 `recorded` 改为 `ekf`，其他选项见 `ros2 run duojin02_slam_tools offline_mapping --help`。真机使用系统时钟，录包时钟只用于离线回放。

`DUOJIN02_WORKSPACE_ROOT` 可指定数据根目录，导航相对地图路径和在线保存目录以它为基准；录包、显式离线输出和续建路径以当前目录为基准。改数据根目录后，建议统一传绝对路径。

## 远程 RViz

观看机使用 ROS 2 Humble，并按本机架构构建同版本工作区。两端 ROS 域一致，机器人进程也需允许远程发现。观看机只启动 RViz：

```bash
source ./source.sh
export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0
ros2 run rviz2 rviz2 -d "$PWD/src/duojin02_bringup/config/rviz/navigation.rviz" \
  --ros-args -p use_sim_time:=false
```

建图可改用同目录的 `mapping.rviz`。只看独立雷达时，Fixed Frame 设为 `laser`，添加 `/scan` 的 LaserScan，Reliability 选 Best Effort；完整导航模板使用 `map`。

## 本地测试与开发

原生环境与本地容器共用一个测试入口：

```bash
./scripts/test.sh
```

默认运行全部检查，可追加 `quick`、`navigation`、`slam` 或 `lidar` 只运行一组。导航组包含 C++ / GPU 测试及命令链集成检查。结果统一写入 `build/tests/`，可用 `TEST_ROOT` 指定输出位置，`BUILD_JOBS` 和 `BUILD_WORKERS` 控制构建并行度。

本机未安装 Humble 时，入口自动构建并使用本地 Docker 环境；已有 Humble 时也可加 `--docker`。本地开发与测试使用同一镜像，包含所需编译工具和补丁 SLAM 测试源码。测试容器断网运行，不映射底盘或雷达设备，源码只读。原生 SLAM 测试使用已准备好的补丁源码，通常由首次构建生成。合成数据与伪串口检查不能替代真机验收。

需要交互式容器开发时再执行：

```bash
./scripts/build_image.sh
DEV_LIST=/dev/duojin02_controller,/dev/lslidar ./scripts/dev.sh
./scripts/enter.sh
```

默认镜像 `duojin02:hardware-humble`、容器 `duojin02_hardware`，工作区挂载到 `/ws`。容器只打开开发环境，进入后按上文启动。改设备或 GUI 设置前先退出 ROS，保存 `/ws` 外的数据，再用 `./scripts/stop.sh` 删除旧容器并重建。

本机容器 RViz 使用 `GUI=1 GPU=1 ./scripts/dev.sh`，GPU 依赖宿主 NVIDIA Container Runtime。Docker 用于本地开发与测试，Jetson 使用上方原生流程。

## 配置与检查

常用配置集中在 [bringup/config](src/duojin02_bringup/config)：

| 内容 | 文件 |
| --- | --- |
| 里程计、IMU 与底盘反馈 | [base_driver.yaml](src/duojin02_bringup/config/base_driver.yaml) |
| 导航、足迹和 GPU 开关 | [nav2.yaml](src/duojin02_bringup/config/nav2.yaml) |
| 制动与指令有效期 | [safety.yaml](src/duojin02_bringup/config/safety.yaml) |
| 速度来源与优先级 | [twist_mux.yaml](src/duojin02_bringup/config/twist_mux.yaml) |
| 建图 | [slam_toolbox.yaml](src/duojin02_bringup/config/slam_toolbox.yaml) |
| 定位质量报警 | [localization_monitor.yaml](src/duojin02_bringup/config/localization_monitor.yaml) |

修改源码或 `src/` 下的配置后，重新执行 `./scripts/build_hardware.sh` 并重启对应节点。底盘模型见 [chassis_geometry.xacro](src/duojin02_description/urdf/chassis_geometry.xacro)，来源与测量保存在 [measurements.json](docs/chassis/measurements.json)；雷达扫描高度和 IMU 外参仍需实测标定。

历史试验数据保留在 `docs/evidence/`，旧报告可从 Git 历史查阅。

自研 `duojin02_*` 包使用 GPL-3.0-only；第三方遵循各自许可证，雷达来源见 [UPSTREAM.md](src/lslidar_driver/UPSTREAM.md)。项目维护：李天顺（litianshun.cn@gmail.com）。

实时目标实施与独立验证入口见 [任意地图目标实施合同](docs/realtime-goal-implementation.md)。物理 2 cm/1° 与 95% 联合到达率尚未验收。
