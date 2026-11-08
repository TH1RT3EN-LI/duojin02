# GPU 链路的时间契约与逐束几何

这次将数学报告的 P1/P2 接入 GPU 工作区的实际 SLAM 和 AMCL 消费者。新增共享包 `duojin02_scan_geometry`，并通过 source-pinned `patches/slam_toolbox/deskew.patch` 修改 Karto 建图、图保存与恢复；AMCL 使用同一准备层。没有修改 CPU 工作区。

这是已集成的可选处理链，不是历史数据采集时钟已经校准的声明。默认仍为 `scan_motion_compensation: false`、`capture_time_offset_sec: 0.0`、`scan_reference_fraction: 0.0`。`header-first-ray-assumed-v1` 明确表示按消息规范假定 header 为首束，不能作为历史驱动确实符合该规范的证据。报告中的 −110 ms 未设为生产默认值。

## 生产接口

`include/duojin02_scan_geometry/geometry.hpp` 提供无 ROS 依赖的几何与历史支持；`ros_scan_geometry.hpp` 提供接收 `/tf`、`/tf_static` 的消费者准备器。

| 接口 | 契约 |
|---|---|
| `declareConfig(node)` | 声明只读配置，非零 offset 必须有非默认 mapping ID 与非空证据来源 |
| `referenceStampNs(scan, config)` | 整数纳秒物理参考时刻；不改源消息 |
| `RosScanGeometry::prepareReference(scan, config, odom, base)` | P1：严格支持的参考位姿、raw/reference stamp、来源 ID；不改束几何 |
| `RosScanGeometry::prepare(scan, config, odom, base)` | P2：同一参考信息及每束 endpoint/origin，坐标为 `base` 在物理参考时刻的坐标 |
| `TimedScan::odom_from_base_reference` | 与 deskew 完全相同 SE(2) 插值得到的参考 base 坐标到 odom 的位姿，运动更新和 map→odom 也使用它 |
| `Beam` | 保留 `original_index`、逐束整数采集时刻、raw range、有效性、命中状态、端点和移动射线原点 |
| `Error::reason` | 输入错误、缺少历史/未来支持、内部采样缺口、缺少帧路径分别可见 |

准备器按实际 TF edge 样本建立历史。它拒绝历史外推、非单调或冲突的 TF 样本，以及超过 `scan_max_tf_gap_sec` 的内部插值间隔；精确已有采样节点仍合法。SE(2) 分段恒定 body twist 插值用于 base 运动，静态激光外参的平移、yaw 和水平倒装都只应用一次。轮式机器人转动时，非零杆臂产生的逐束激光原点平移也保留。

不能用 tf2 的“查询成功”代替内部采样缺口检查。tf2 平移线性插值和共享层的 SE(2) 插值也不能混用：转弯区间中，AMCL 运动更新和 Karto odom pose 使用 `odom_from_base_reference`，map→odom 由同一位姿直接组合。

`scan_motion_compensation: false` 且时间映射为历史恒等时保持原有入口，无新增 TF 支持等待；P1 非恒等映射和 P2 都使用按源顺序的有界等待队列。扫描先于最后束 TF 到达时会等待；容量、等待超时、支持不足和输入错误会输出明确拒绝原因，不能悄悄换成最新 TF。

## 配置和地图一致性

| 参数 | 默认 |
|---|---|
| `scan_motion_compensation` | `false` |
| `capture_time_offset_sec` | `0.0` |
| `capture_time_mapping_id` | `header-first-ray-assumed-v1` |
| `capture_time_mapping_source` | 空字符串 |
| `scan_reference_fraction` | `0.0`，首束；`0.5` 为首末束中点 |
| `scan_max_tf_gap_sec` | `0.1` |
| `scan_tf_history_sec` | `120.0` |
| AMCL `scan_pending_queue_size` / `scan_pending_timeout_sec` | `200` / `2.0` |
| SLAM `scan_geometry_pending_limit` / `scan_geometry_wait_timeout_sec` | `200` / `2.0` |

SLAM 的 `config/slam_toolbox.yaml` 与 AMCL 的 `config/nav2.yaml` 都有上述恒等默认值。要选择 D1，应在各自参数文件同时选择相同时间映射和 `scan_motion_compensation: true`，从原始扫描重新建图，保存该地图，再用相同配置做固定地图定位。

每次图保存写出 `PREFIX.scan_geometry.json`，记录几何开关、offset、mapping ID/来源、reference fraction 与最大插值缺口。快照保存时将它作为 `scan_geometry` 角色复制并计算哈希；仅保存栅格时也从 SLAM 的只读运行参数生成侧车。`localization.launch.py` 在启动节点前对照地图侧车和 AMCL 参数；图恢复也对照同一契约。缺少侧车的旧地图只允许历史恒等 D0。不能对旧地图只开启 query deskew。

Karto 的 `LocalizedRangeScan` 使用版本化 v2 序列化保存逐束本地端点、射线原点、原始索引、逐束时间、raw/reference 时间来源、显式 `Invalid/Hit/NoReturn` 状态及驱动真实量程范围。驱动有效性与 mapper 的 min/threshold 选束门限分开；有限短距无效束既不匹配也不清空空间，明确无返回束只按算法上限清空空间。raw 的地图边界保留历史 sensor pose 加匹配命中点政策；prepared 对同一 `IsBeamMatchable` 命中集合加入补偿后的端点及其真实逐束原点。无返回及算法量程外射线不扩大地图边界，实际清空仍从真实原点裁到算法上限并由原生边界逻辑处理。优化或图恢复重建 dirty cache 时只消费持久化 prepared 几何；explicit-loop fine pass 使用保测量身份 clone。prepared 几何、来源与原始审计量程不可改写，无法保持子类或 custom data 语义的复制显式拒绝。旧 v0/v1 刚体 archive 仍可加载；旧 v1 prepared archive 缺少状态合同，原生读入明确拒绝并要求重建地图。旧程序不能保证读取新 v2 archive。

建图和定位均不把补偿点重新分桶成 `LaserScan`。源 header 保持可审计；估计器运动更新、观测和发布使用显式物理参考时刻。

## 构建与复核

在已有 Humble/CUDA 容器或等效环境内构建：

```bash
./scripts/build_slam_backend.sh
./scripts/build_cuda_local.sh
source ./scripts/source_cuda_local.sh
```

纯几何检查可以在普通 C++17 host 执行：

```bash
g++ -std=c++17 -Wall -Wextra -Werror -pthread \
  -I src/duojin02_scan_geometry/include \
  src/duojin02_scan_geometry/test/geometry_checks.cpp -o /tmp/scan_geometry_checks
/tmp/scan_geometry_checks
python3 -m pytest -q tests/test_scan_geometry_contract.py
```

实际 ROS SLAM 队列合同复核明确使用安装后的 `sync_slam_toolbox_node`（脚本依赖其 `processing_status` 服务，不能传入 live 默认的 async 节点）：

```bash
python3 tests/integration/slam_deskew_probe.py \
  --executable "$PWD/install/slam_backend/lib/slam_toolbox/sync_slam_toolbox_node" \
  --output "$PWD/output/slam-deskew-validation"
```

该脚本默认读取本仓库 SLAM 配置，也可通过 `--parameters` 指定；输出目录须为新目录。已执行证据为 `output/drift-remediation-timing/slam-d1-r3/validation.json`：含非零杆臂和连续平移转弯，8 帧扫描先于最后束 TF 到达，8/8 完成、0 invalid、0 drop，发布时刻与整数纳秒参考一致，实际 serialize/deserialize 服务成功。`time_increment` 是 float32 消息字段，合成探针也按其序列化数值计算预期时刻，避免把 1 ns 量化差误判为时间错误。

`tests/integration/karto_deskew_probe.cpp` 验证端点、算法 range 门限、dirty cache、移动原点、正确原点裁剪、独立 sensor point API 和 v2 binary 保存读取。`karto_measurement_contract_probe.cpp` 将完整生产 Karto.cpp/Mapper.cpp 以 undefined/float-cast-overflow sanitizer 编译，执行实际录制 811 束扫描的短距束、原生 lookup、地图插入、coarse/fine 保身份复制、archive 往返和首/中/末参考的等价性；另检验 NaN/Inf/大有限坐标、整数乘加边界及 100m no-return 不扩大历史命中点地图边界，raw/prepared 在静止几何下边界相同。`karto_prepared_v1_writer.cpp` 用冻结的旧安装库生成真实 v1 prepared fixture，验证新读取器明确拒绝；`karto_legacy_scan_probe.cpp` 的 writer 使用冻结 v0 库，reader 使用新库，验证历史刚体读取。共享几何检查另覆盖纯转杆臂、恒定 twist、转弯参考插值、倒装外参、坏束原索引和无外推/内部缺口边界。

## 当前边界

当前保存合同使用 `duojin02.scan_geometry.v2`，测量版本为 `typed-rays.v2`。
Invalid/Hit/NoReturn 显式区分；Invalid 无可访问的端点或射线原点。
`FinitePoint` 构造检查有限值，算法量程门限使用独立的选择原因及原始束索引。
传感器无返回政策明确为 `max-or-positive-infinity`，不是把所有无效束改作清空空间。
合同固定记录 `capture_time_evidence_status=uncalibrated` 和
`beam_order_evidence_status=original-index-increasing-time-assumed`；自定义映射名称或来源
文字不构成已校准证据。旧 v1 补偿地图必须重建，不能仅更换 sidecar 宣称升级。

当前束时间模型是原始束索引乘 `time_increment` 的顺序均匀采集模型。D1 对多束扫描要求正的、有限的 `time_increment`；无法确认历史束序或采集时刻时，应保留默认 D0，并先取得设备/驱动时钟证据。内部动态运动必须为平面 SE(2)，邻接旋转使用主值分支；不支持倾斜激光扫描、动态 roll/pitch 或可从间隔内任意额外整圈转动恢复真实轨迹的推断。

运动补偿按静态世界点假设工作，不补偿动态行人或物体的自身运动。原始 TF/里程计精度、外参和采集标定来源仍需独立核验和冻结。D1 要求 `enable_interactive_mode: false`，避免交互模块以原始扫描重播覆盖补偿语义。开启 D1 时 AMCL 的 endpoint likelihood 使用补偿端点；beam raycast 模型不能假定共用固定射线原点，当前明确拒绝该组合。

这些检查确认了生产链的契约、异常路径和保存恢复语义；它们不是新的 LILocBench 精度成绩。报告要求的 T0D0/T1D0/T0D1/T1D1 应各自从原始扫描重建地图，并在独立确定时间映射后按冻结支持集合评测。

相关匹配保留 Karto 的原始 source-beam roster 归一化：`score = sum(selected finite hit response) / (Nsource * 100)`，即有效覆盖率乘以有效命中的平均响应。无效、无返回及算法未选束不进入相关查表分子，仍在固定原始束数中表达零贡献覆盖；这是置信尺度，不是把无效坐标当作观测。Nsource 或有效命中为零时安全返回零。日志记录原始束数、预角度有限选中命中数、分母及固定 `source-beam-roster.coverage.v1` 政策；预角度命中数不等同于每个候选实际落图贡献数。
