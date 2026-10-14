# 项目名称统一与升级（2026-10-14）

当前版本把 6 个自研 ROS 包及其内容从 `duojin01` 统一为 `duojin02`。包目录、清单、CMake、Python 模块、安装资源、启动引用、C++ 命名空间、Nav2 插件标识、URDF / 网格 URI、配置引用和测试入口一并迁移。算法、参数数值、话题及 TF 坐标系保持原有语义；此次改名不新增兼容包、旧名称别名或环境变量回退。

## 包与运行入口

| 旧包名 | 当前包名 |
| --- | --- |
| `duojin01_base_driver` | `duojin02_base_driver` |
| `duojin01_description` | `duojin02_description` |
| `duojin01_bringup` | `duojin02_bringup` |
| `duojin01_slam_tools` | `duojin02_slam_tools` |
| `duojin01_teleop` | `duojin02_teleop` |
| `duojin01_navigation` | `duojin02_navigation` |

第三方雷达包 `lslidar_driver` 保持原名，工作区总计仍为 7 包。启动文件和标准 Nav2 Action 名称保留，包参数换成上表新名称；例如：

```bash
ros2 launch duojin02_bringup navigation.launch.py map:=maps/site_a.yaml
ros2 run duojin02_teleop keyboard_teleop_node
ros2 run duojin02_slam_tools offline_mapping "$PWD/bags/run01" --output-dir "$PWD/maps"
```

外部 launch、Python 导入、C++ include / 命名空间、pluginlib 类名和 `package://` URI 均需更新。导航插件当前标识是 `duojin02_navigation/PreciseLattice`、`duojin02_navigation::PreciseMPPI` 和 `duojin02_navigation::StableStoppedGoalChecker`。终端、服务启动脚本和观看机必须使用同版本安装产物。

## 原生构建与环境

先正常停止旧 ROS 进程，再在未加载旧工作区的新终端进入仓库根目录。旧构建缓存和 ament 索引不能直接沿用。下例归档生成目录后重新构建；源码、地图、录包和冻结证据保留：

```bash
mkdir -p .cache
renaming_backup=$(mktemp -d "$PWD/.cache/package-renaming.XXXXXX")
for generated_dir in build install log; do
  if [[ -e "$generated_dir" ]]; then
    mv -- "$generated_dir" "$renaming_backup/$generated_dir"
  fi
done
./scripts/build_hardware.sh
source ./source.sh
```

项目脚本重新生成 SLAM 后端和硬件覆盖层。不要把旧安装复制回新 `install/`，也不要同时加载新旧工作区。若 shell 配置、systemd 服务或外部脚本曾自动 source 旧工作区，更新该配置后重新启动对应终端或服务。跨架构仅传源码与数据，观看机按自身架构重新构建描述包。

工作区数据变量由 `DUOJIN01_WORKSPACE_ROOT` 改为 `DUOJIN02_WORKSPACE_ROOT`；旧变量不再读取。需要自定义数据目录时，在 source 前设置新变量的绝对路径，例如：

```bash
export DUOJIN02_WORKSPACE_ROOT=/absolute/path/to/data
source ./source.sh
```

现有地图和录包文件无需改名或重写。地图保存的旧参数快照、后端来源及文件哈希描述保存时的版本，继续作为历史元数据；不要为了名称统一改写原始快照。

## 设备规则与 Docker

默认底盘串口由 `/dev/duojin01_controller` 改为 `/dev/duojin02_controller`；雷达仍为 `/dev/lslidar`。按真实设备身份执行 [主 README](../README.MD#ubuntu-2204-原生构建jetson-首选) 中的 `scripts/set_dev.sh`，新规则文件为 `/etc/udev/rules.d/99-duojin02-devlinks.rules`。旧规则不会自动迁移。清理确认为本项目创建的旧底盘链接规则；若旧规则文件还含雷达绑定，保留或迁移该绑定后再退役旧文件。自定义串口路径仍可通过启动参数显式提供。

| 内容 | 旧默认值 | 当前默认值 |
| --- | --- | --- |
| 镜像 | `duojin01:hardware-humble` | `duojin02:hardware-humble` |
| ARM64 交叉构建镜像 | `duojin01:hardware-humble-arm64` | `duojin02:hardware-humble-arm64` |
| 容器 | `duojin01_hardware` | `duojin02_hardware` |
| 镜像内安装根目录 | `/opt/duojin01` | `/opt/duojin02` |

旧镜像仅改标签仍含旧安装，必须从新源码重建。旧容器也不会因默认名称变化而被新 `stop.sh` 找到：先正常停止其中 ROS 进程、保存未挂载数据，再用旧实际名称处理容器；该步骤由操作者明确执行。新环境按 [Docker 操作](../README.MD#可选-docker-环境) 重建并传入新设备链接。测试镜像同样重新构建 `testing` 目标。

## 核对与历史证据

加载新环境后，可先检查索引及 launch 声明，再执行 [软件验证](../README.MD#验证边界)：

```bash
ros2 pkg list | grep -E '^(duojin02_(base_driver|description|bringup|slam_tools|teleop|navigation)|lslidar_driver)$'
ros2 launch duojin02_bringup mapping.launch.py --show-args
./scripts/validate_hardware.sh
```

包索引应包含 6 个新自研包和 `lslidar_driver`。若还能查到旧自研包，检查加载路径与旧安装残留。索引和声明检查不等于插件已实际加载或硬件已部署；导航插件与指令链、雷达和 SLAM 的回归分别由导航、雷达和 SLAM 测试入口验证。当前源代码更新不代表机器人或观看机已升级。

`docs/evidence/` 是冻结试验原始资料，字节保持不变，其中旧包名、参数路径、哈希和归档成员按原版本解释。[硬件精简历史](hardware-simplification.md) 中当时实测的旧镜像和安装路径也保留原名；真实外部历史文件链接不改成不存在的新路径。当前源码链接与操作命令使用新名称。

本次改名的[验证记录](evidence/package-renaming/validation.json)来自干净构建的 AMD64 测试镜像：7 包构建、124 项 Python 测试、12 个 launch 入口、14 项导航 CTest，以及导航指令链、雷达和 SLAM 集成均通过；3 个 Nav2 插件经 pluginlib 动态加载并实例化。安装索引没有旧自研包。全部运行在无外部网络、无设备映射的容器中，未部署到实物机器人。
