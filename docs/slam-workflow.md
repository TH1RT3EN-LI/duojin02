# SLAM 数据工作流

本页集中维护录包、离线回放、保存与续建。在线使用项目补丁版 SLAM Toolbox，固定图使用 AMCL；构建和各模式启动见 [主 README](../README.MD)，固定图就绪与恢复见 [定位健康](localization-health.md)。来源、工具链修复与冻结验证保留在 [SLAM 升级记录](slam-upgrade-review.md)，不作为本页默认参数。

## 使用前

在工作区根目录加载 `source ./source.sh`。使用项目构建脚本生成 `install/slam_backend` 和 `install/hardware`；apt 原版后端不能替代带补丁的覆盖层。源码锁和补丁校验由 [source.json](../patches/slam_toolbox/source.json) 及构建记录维护。

在线入口拥有底盘与雷达，选择建图、固定图导航或在线建图导航中的一个。真机使用系统时钟；离线回放只启动消费者，使用独立域和录包时钟。当前阈值以有效 YAML 为准，旧报告数值不能覆盖它们。

示例假定数据工作区就是当前仓库。若设置 `DUOJIN02_WORKSPACE_ROOT` 指向其他绝对目录，按下面的实际路径规则统一保存和读取，不混用两份 `maps/`：

| 输入 | 相对路径的基准 |
| --- | --- |
| 固定导航 `map:=...`、在线保存 `output_dir:=...` | `DUOJIN02_WORKSPACE_ROOT`；默认来自 `source.sh` 的仓库根目录 |
| 录包目录、离线 `--output-dir`、续建 `pose_graph:=...` | 调用时的当前目录；示例显式用 `$PWD` 的位置不要当作自定义数据根目录 |

离线 `--output-dir` 不传时默认用数据根目录下的 `maps`；表格说明的是显式相对路径。数据根目录改变时，离线输出及续建前缀优先传同一数据目录的绝对路径。快照名不是目录；位姿图前缀与栅格 YAML 也不能互换。

## 录包

先启动实际底盘和雷达，确保 `base_footprint -> laser`、`base_footprint -> imu_link` 的标定和 TF 正确。随后执行：

```bash
./scripts/record_slam.sh bags/run01
# Ctrl+C 正常结束录包，让 rosbag2 写完 metadata.yaml
```

录制 `/scan`、`/odom`、`/imu`、`/odometry/filtered`、`/tf`、`/tf_static`、`/diagnostics`，使用 SQLite。静态 TF 使用 transient-local QoS，晚于静态变换发布启动的记录器仍可接收其历史样本。节点参数放入相邻的 `run01.configuration/`。录包目录和元信息目录都必须是新目录；元信息目录以原子创建预留，任一目录已存在就拒绝启动，保留原数据不写入。录包先启动，节点参数随后在同一等待窗口内并行采集，不让缺失参数服务推迟传感器记录。参数不可用时会保留错误日志并提示，应检查该目录与节点状态；录包仍继续。用于配置对照前应补齐缺失的有效参数信息。记录器自身失败时，脚本返回其失败退出码。

不要通过强制杀死记录器结束采集。容器默认只持久化 `/ws`，录包与地图应写入此工作区。

## 独立离线回放

以下命令在仓库根目录执行，原生环境与 Docker 均使用实际工作区路径，`/ws` 只属于容器：

```bash
# 保留录包中的 odom -> base_footprint：评估原有里程计链路
ros2 run duojin02_slam_tools offline_mapping "$PWD/bags/run01" \
  --odom-source recorded --output-dir "$PWD/maps" --map-name run01_recorded

# 从原始底盘速度 / IMU 重新运行 EKF：评估偏置与方差修改
ros2 run duojin02_slam_tools offline_mapping "$PWD/bags/run01" \
  --odom-source ekf --output-dir "$PWD/maps" --map-name run01_ekf
```

回放默认 ROS 域 97，必须区别于调用终端的在线域；冲突会拒绝运行，可用 `--domain-id` 另选 0–101 内的独立域。入口将离线进程设为 `ROS_LOCALHOST_ONLY=1`，不是把回放发布给真机。默认速率 0.5，可用 `--rate` 调整。离线入口只启动数据过滤节点、同步 SLAM，以及按需启动的 EKF。录包中的 `map -> odom` 被删除；重跑 EKF 时也删除旧的 `odom -> base_footprint`。只播放白名单中的传感器话题，不播放速度命令。

播放器以暂停状态启动，先输出 `/clock`；控制器确认 SLAM 的 ROS 时钟已激活，且时间与暂停的播放器相同，才通过服务恢复播放。这样首次时间切换发生在 TF 输入之前。暂停、时钟输出和恢复服务使用 [rosbag2 Humble 的接口](https://github.com/ros2/rosbag2/blob/humble/rosbag2_transport/src/rosbag2_transport/player.cpp)，初始化超时会结束流程。

重跑 EKF 默认以最初 0.5 秒扫描作为预热区间，期间仍播放原始反馈。报告明确列出原始扫描数量、跳过数量和实际待处理数量；数据已含充分预滚时可设置 `--ekf-warmup 0`。记录里程计路线不跳过预热扫描。

这里的 `use_sim_time=true` 用于 rosbag2 的 `/clock`。它局限于离线消费者，不会启动仿真器，也不会改变真机入口的系统时钟。

工具先检查录包消息类型、扫描时间递增和 TF 链；播放结束后核对扫描输入数量、非法输入计数、队列归零与处理完成计数。移动 / 时间阈值未达到的扫描会按 SLAM 的既定筛选规则跳过，`accepted` 和 `completed` 指外层筛选及处理调用，不等于 Karto 最终图节点数。队列超过 `max_pending_scans`（默认 1000）时显式报错，需降低回放速率。缺 TF、丢消息、处理超时或保存失败均返回非零，且不发布新地图 YAML。

QEMU 跨架构验证中，早期 1 倍速回放曾触发状态服务响应超时；降速后仍复现了首次时钟切换造成的 TF 丢失，因此进一步修正了上述启动顺序。集成探针支持 `SLAM_PROBE_RATE` 和 `SLAM_PROBE_DRAIN_TIMEOUT` 调整测试速率与排空期限，扫描计数、非法输入和队列完成校验保持相同。跨架构功能验证使用的速率不能用于推断 Orin 原生性能；板端应按实际负载选择 `--rate`。

每次运行产生 `offline-session-*.json` 与日志。复盘应使用同一份录包、相同输入路线和校准参数，只改变一个待比较的算法或参数。

## 保存、导航和续建

已有在线建图节点时，先停稳，再保存：

```bash
ros2 launch duojin02_bringup save_map.launch.py map_name:=site_a
```

在线保存只有一个 `save_timeout` 等待预算，默认从保存工具的 `DEFAULT_SAVE_TIMEOUT` 读取；离线导出对应 `--save-timeout`。快照锁等待、地图刷新 / 保存、位姿图序列化及运行时参数采集共用同一单调时钟截止，不再分别设置服务发现和响应超时。后端保存回调已删除固定一秒等待。

地图像素、阈值和文件完整性检查必须通过才发布 YAML。运行时 SLAM 参数元信息取不到时记录警告并写入快照来源信息，保留有效完整的地图与位姿图；它不会把文件或地图无效当成可忽略的元信息缺失。预算限制客户端等待，不能抢占已经进入后端的处理或文件写入。手柄保存快捷键在保存仍进行时忽略重复触发，不会杀掉正在保存的进程。

保存成功时 `ros2 launch` 返回 0；保存客户端失败时启动入口返回非零，脚本和手柄反馈可据此判断结果。

保存完成后停止建图入口，再用栅格图启动固定图导航：

```bash
ros2 launch duojin02_bringup navigation.launch.py map:=maps/site_a.yaml
```

显式指定 `map` 避免自动选中其他实验地图；同名快照会拒绝保存，不覆盖原文件。需要新一轮记录时换 `map_name`，不要删除旧地图来绕过重名检查。

需要续建时，先停止原建图 / 导航入口；下面的示例初值必须替换为机器人在旧地图中的真实起点：

```bash
ros2 launch duojin02_bringup mapping.launch.py \
  pose_graph:="$PWD/maps/site_a" map_start_pose:='[1.2, 0.5, 0.0]'
# 若确实位于首次扫描的位置，才使用 start_at_dock:=true
```

建图、导航和续建是不同启动模式，不叠加运行。固定图导航需指定真实初值并等新鲜扫描及 TF 就绪，见 [恢复操作](localization-health.md)。导航验证图像存在、分辨率和原点有效；有清单时核对实际选择的 YAML 和其图像的角色、路径与 SHA256，不依赖未使用的位姿图文件。声明快照却缺少必需角色，或角色指向另一份地图时拒绝使用。没有清单的旧有效 YAML + 图像地图仍可用于导航，包括合法的 PGM 和 PNG。续建需要匹配的 `.posegraph` / `.data`，有清单时还核对这两个角色；栅格图不能代替位姿图。`map_start_pose` 必须是三个有限数值，布尔值不是坐标；`start_at_dock` 只接受 true 或 false。不要跨未验证的 SLAM 版本加载二进制位姿图。

就绪等待、元信息缺失与保存预算的统一边界见 [运行机制](runtime-mechanisms.md)。快照发布是文件层面的完整性保证。在线导出不暂停建图或机器人运动，各次服务调用之间可能继续收帧；需要同一时刻的严格离线基准时，用队列已清空的离线流程导出。

## 输出与复核

| 文件 | 用途 |
| --- | --- |
| `.yaml` + `.pgm` | 栅格导航地图，含阈值、分辨率和原点 |
| `.posegraph` + `.data` | 同后端版本的位姿图续建 |
| SHA256 清单、参数与后端来源 | 检查快照完整性和构建版本 |
| `offline-session-*.json` 与日志 | 回放输入计数、排空、失败与保存结果 |

保存客户端以 SaveMap 实际输出为准，检查 PGM/trinary 像素与 `free_thresh: 0.196`、`occupied_thresh: 0.65`；不拿较早的异步地图响应覆盖保存原点。详细修复与旧地图限制见 [地图契约](navigation-repairs.md)。文件完整性、处理计数和可重载性分别验证，不能据此推断地图精度或板端性能。
