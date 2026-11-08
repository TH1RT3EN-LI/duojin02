# 任意地图目标：实施与运行合同

本轮修改基于开工时的完整工作树（包含原有未提交内容），不是只检出 HEAD。旧输出与原安装树保留。本轮证据目录为 `output/lilocbench-realtime-goal-implementation-20261019-212448`；报告分别列出成功、失败、默认状态和未验证项。

## 默认行为

- 已知地图导航继续冻结地图；PF 是唯一后验所有者，AMCL 是唯一 map→odom 提交者。
- 保留原观测模型、束数、温度、KLD 计数、运动噪声和控制阈值。`pre_resample_pose_output=false`、`fine_mode_diagnostics=false`；不自动启用连续观测、共享噪声权重或影子子地图图。
- Karto 新编源码默认返回实际返回位姿的同支撑复评分；峰值另存。选某个 attained 峰仍是显式实验选项。返回分数忠实不等于模式正确或物理定位更准。
- 导航链默认增加被动 `task_goal_capture`，保存原 RViz `/goal_pose` 的 binary64 数值、frame、地图内容身份和转换来源。它只发布元数据，不发送 Action/速度。直接 Action 客户端必须提供原请求 ledger；相同 planner endpoint 不能证明 Action 身份。
- 现有 converge、XY 复查、实测速度检查、连续停稳及 smoother/mux/guard 链保留。没有无依据的 vτ、硬门、执行器参数或新速度发布者。

AMCL 合同严格区分原始/参考时刻、最后有效观测、消息过滤后回调收妥时刻、PF 完成时刻和 TF stamp。PF 完成时刻先于控制接收，不冒充端到端可用时刻。地图身份是明确的内容 fingerprint；物理标定状态仍为 unknown/unvalidated。

控制消费合同记录实际精确/最新 TF fallback 查询、消费与命令可用时刻、实测速度和请求命令。旁路 AMCL 诊断和 TF 没有原子版本绑定时明确 unknown；当前接口不能伪装为已经具备完整物理联合任务 covariance。

## 构建和隔离验证

在已有 ROS Humble/CUDA 环境中，普通项目构建流程仍使用当前源码及有序 patch 清单：

```bash
./scripts/build_slam_backend.sh
./scripts/build_cuda_local.sh
```

本轮没有覆盖原 install。已验证入口在 Humble 容器内使用：

```bash
source scripts/realtime_goal/source_reviewed.sh
```

该入口优先加载私有 `candidate-A03/native/install` 与 `backend-native/A02/install`，同时保留必要的 ROS 与硬件包父依赖，不启动节点。A03 的 15 个完整 ELF 与 A02 严格逐字节相同，仅重建修复 SIGINT 收尾的 Python bringup 包；复制、源码差异和安装 smoke 均有独立证明。原 `source_cuda_local.sh` 入口及旧安装不改，仍可显式回退。

可用独立前缀重建，输出目录须全新：

```bash
bash scripts/realtime_goal/build_isolated.sh /work /work/output/NEW/native ON
```

此脚本构建 scan geometry、MPPI、AMCL、navigation、bringup、slam tools，开启真实组件测试并固定本机 CUDA 架构 120。它是本机验证入口；不宣称 Orin 原生兼容。可将第三参数设为 OFF 验证无 CUDA 构建。测试必须加载对应私有 install 的 lib；单独 source 环境可能仍加载旧库，需保存 `/proc/PID/maps` 及 SHA。

`run_recorded.sh` 在隔离容器中复用冻结真实 CDR、历史 C000 map/prior，保持 D0/D1 与 T0=0、原 200ms TF 运输提前量。运行前复制 runtime 脚本并冻结 SHA，运行中不修改脚本。通过 `--max-scans` 的查询是前缀开发回归，不是全序列或保留测试。发布到位姿接收计时不含完整采集/执行链；回放并不证明持续输入实时性。

```bash
bash scripts/realtime_goal/run_recorded.sh PRIVATE_INSTALL NEW_TRIAL T0D0 PARAMETERS.yaml 512
```

地图、prior、数据和 prefix 必须按本轮报告的 bind 关系可见。完整回放最后参数为 0。脚本拒绝旧结果目录，要求实际 CUDA 更新，单列数值边界 CPU fallback，结束后执行独立 motion/source 审计。

## 实验路径和验收

[AMCL](realtime-goal-amcl.md)、[匹配/连续几何/刚性子地图](realtime-goal-mapping.md)、[共享误差](realtime-goal-statistics.md)、[目标/控制/任务评价](realtime-goal-navigation.md)记录具体数学与实现。

影子路径不会回写 PF、图或控制。连续端点线段保留 prepared origin/endpoint、参考时刻、source payload 和几何身份；遮挡/可见性尚未验证，`usable_for_precision=false`。纯 Python 当前真实样本耗时为秒级，不能默认启用。

实验后台图使用真实冻结内部几何和独立刚体 S_j、query X_q 状态，保留 accepted ledger、source overlap、几何版本和 Snapshot/Finalize；迭代间预算让出并保留所有因子。显式 experiment shapes 不称为独立物理信息。没有固定窗口，因此没有实现或声称边缘化先验；单次 dense 运算仍可能超过 wall budget。

真实到点评价器要求任务前冻结注册与用户目标，同时检查 XY、yaw、工作点速度、连续停稳和精确完成时刻真值支持，超时/失败/缺真值均入分母。不能从全程 SE(2) 对齐 ATE 或合成任务频率推导物理成功率。2cm/1°、至少95%仍为待验收目标。

原安装仍可作为运行回退；参数 `pre_resample_pose_output=false` / `fine_mode_diagnostics=false`关闭两条 AMCL 实验路径；Karto `SetReturnSelectedPoseScore(false)`为显式旧峰值返回兼容开关（完整旧行为可使用保留的旧 prefix），`SetSelectAttainedMode(false)`保持旧位姿选择。所有影子模块默认没有运行期接入。
