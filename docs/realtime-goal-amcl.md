# AMCL 分阶段状态与证据

本实现保留 PF 为跟踪后验所有者和 `AmclNode` 为唯一 map→odom 提交者。E0/E1/E2 阶段没有新增连续优化结果融合、proposal、观测独立性假设、物理校准、控制门或新的位姿发布者。2026-10-20 增加的显式全局初始化状态机见文末；它与常规 PF 跟踪分阶段运行，不作为第二个后验发布者。

E0 默认增加 `amcl_estimate_contract`（`diagnostic_msgs/msg/DiagnosticArray`，status `amcl/estimate_contract`）。它记录地图内容 identity 与 node-local generation、frame/raw stamp 来源、状态参考时刻、最后有效观测参考时刻、接收的 ROS/steady 时间、posterior 可用时刻及 TF stamp。`received_*` 是 **TF message filter 之后 AMCL 回调** 的时刻，不是驱动收包时刻。steady 时钟只可在同一主机 monotonic 域比较；不能和 ROS stamp 相减。当前 `first_available_steady_ns` 是 PF 观测更新完成后的最早可用时间，不包括之后重采样/发布/运输成本，不能当控制器已消费时间；实际控制边界由导航另记。

来源 frame+raw stamp 的唯一性未校准。诊断采用常量内存的 reference-time 高水位；相同或更老的扫描不会刷新最后有效观测年龄，但兼容 PF 路径仍可能消费它，`source_deduplication=not_enforced_compatibility_path` 明示此边界。本阶段没有声称实现数值观测去重。地图 identity 采用与目标记录器一致的 `occupancy_fnv1a64`：UTF8 frame+NUL，little-endian uint32 width/height，binary64 resolution、origin xyz/quaternion xyzw，再追加 cell bytes。它用于本地内容核对，非防碰撞安全哈希；generation 不跨进程比较。地图重载、初始位姿/全局定位重设、PF 动态重建和 lifecycle cleanup 都清空旧观测合同；细模式有效位同步清除。新增 `amcl_estimate_reset` 直接调用真实节点 callback/重建/cleanup 验证这些入口。

E1 增加 `pre_resample_pose_output`，默认 **false**。启用后，在观测后刷新 KD-tree 和加权 moments，保存独立拥有内存的 `shared_ptr<const PosteriorSnapshot>`，再按原策略重采样。发布均值、完整 XY–yaw covariance 和 map→odom 全部读取同一快照，仍按原最大质量 KLD 连通簇规则。观测模型、KLD、ESS、粒子和 RNG 不变。未通过真实 D0/D1 对照之前，默认继续使用原发布路径。

E2 增加 `fine_mode_diagnostics`，默认 **false**。启用后，在单次 PF 更新之后，用独立的加权 Gaussian KDE watershed 诊断细尺度模式；诊断 bins 为 bandwidth 的一半，KLD 0.5m/10° bins 不变。`fine_mode_position_bandwidth=0.01m`、`fine_mode_yaw_bandwidth=1°` 只是实验诊断分辨率，不是物理噪声参数。细模式均值、完整局部 covariance、其它模式质量及最大分离距离由原加权粒子计算；模式 ID 为该快照局部 ID，不作跨帧跟踪身份。

`likelihood_field_prob` 可只读复评分 coarse 最大簇均值和细模式最大质量簇均值，采用原观测的同一 effective beams/beam-skip mask、prepared endpoints、地图和 beta；值是 log-likelihood，不含 prior。`score_pose_scope=pre_resample_diagnostic_candidates_not_published_pose` 明确它们是预采样诊断候选，并以 binary64 可往返精度提供 `mean_pose_{x,y,yaw}`、`mode_pose_{x,y,yaw}`。兼容 post-resample 输出启用时，这两个分数不冒充实际发布位姿的复评分。它不会再次调用 PF update，也不修改 CUDA 更新计数。其它传感器模型评分保持 unknown。只有真实独立正负关联验证才能支持真实模式正确性。此实验不改变生产返回位姿，也不通过任意选峰抑制歧义；生产输出明确标记 `coarse_kld_cluster_not_verified_unimodal`。

`precision_ready` 始终 false；`physical_calibration`、`shared_map_uncertainty` 和环境几何可观性为 unknown。小 covariance 只代表固定地图/时间/外参假设下所选簇的条件散布。KDE 将稀疏支撑分成多个峰或合并近峰的风险尚未在保留真实数据上校准。当前没有独立连续表面信息矩阵，不能由粒子 spread 判定平行墙弱方向已可观；支持边界也明确 unknown。

可逆开关：两实验参数保持 false 即使用兼容估计与输出路径；旁路 E0 合同继续存在。只读 startup 参数修改后需重配节点。

## 已执行验证

每阶段独立源码副本、diff、argv、来源/二进制哈希和 stderr 存在本轮独立 output 的 `amcl/`；历史输出没有重跑覆盖。

- baseline 与 E0 新编 CPU/CUDA 组件，重复束、格点、双峰和 1000 个固定重采样 seed 的结果逐项完全相同；每个 CUDA arm 真实执行 5 次更新，无 fallback。
- E1 固定 posterior、100 个种子：当步快照均值 RMS=0；原后采样均值 RMS≈1.596mm；下一步粒子、权重、lineage、KLD 样本数和下一次 RNG 数逐项相同。快照不引用被复用的 PF buffers。
- E2 4/10/20cm 等质量双峰：KLD coarse cluster=1、细模式=2；跨 ±π 单峰正确；KLD bins/RNG 不变。CPU 和真实 CUDA（含 beam-skip）同支撑复评分最大差 2.22e−16。原 dual-wall 反例 mean 与峰的评分差仍被明确报告，未伪装为物理收益。
- 最终 E2 源码重编的 legacy 组件输出与 baseline 逐项相同，含全部 1000 个重采样 seed；真实 CUDA 5 次。上述 exact 是组件合同，不代替节点完整回放。

复现（输出目录必须尚不存在；已有容器内 `/work` 绑定项目，CUDA=/opt/cuda）：

```bash
docker exec duojin02-graph-update python3 /work/tests/realtime_goal/amcl/run_native.py \
  --source /work/src/duojin02_amcl_cuda --output /work/output/NEW/amcl-legacy --cuda
docker exec duojin02-graph-update python3 /work/tests/realtime_goal/amcl/run_native.py \
  --source /work/src/duojin02_amcl_cuda --output /work/output/NEW/amcl-modes --cuda \
  --probe /work/tests/realtime_goal/amcl/mode_probe.cpp
```

`amcl_posterior_snapshot` 加入包 CTest，根实施 agent 统一新 prefix 构建、节点回放和最终集成。真实正负关联样本、持续输入预算、未见序列和外部真值闭环任务仍是独立放行门；本模块组件结果不能声称获得新的 ATE、2cm/1° 或 ≥95% 导航联合成功率。

## 2026-10-20：显式全局捕获初始化

`global_acquisition_enabled` 是只读启动参数。**C++ 组件缺省值与项目入口 `src/duojin02_bringup/config/nav2.yaml` 均为 false。** 只有显式启用且调用 `reinitialize_global_localization` 才进入实验捕获路径。外部 `initialpose`、已有初值跟踪及关闭此参数时保持原估计路径。它针对“全图均匀 3000 粒子第一次观测后只剩一个错误全局祖先，且没有候选恢复”的能力缺口，不是厘米级跟踪系统误差的完整修复。

**2026-10-20 完整验证未放行：** 冻结 A03 二进制的 `dynamics_0` 全局查询由旧路径约 19.55/20.51 m 降为 0.0622/0.0747 m；但独立 `lt_changes_0` 查询 D0/D1 仍误捕获，固定训练坐标 RMSE 为 16.96/15.97 m，首个输出约偏 5.25/5.55 m、朝向接近反向。数据与实际加载核验均通过，不能归为评价或漏帧问题。端点分数、候选 margin 和连续三帧门不足以防止真实重复结构中的假捕获。试验中曾暂设 true，现已撤回为 false；保留实验代码和失败证据，不能把组件测试或单序列改善称为可靠全局重定位。完整证据见项目 `output/lilocbench-root-repair-20261020-100838/report.md`。

该路径目前只支持 `likelihood_field_prob`，以保持与其相同的束选择合同；普通 `likelihood_field` 的步长不同，`beam` 没有该距离场合同，均在启动与动态参数预检时拒绝。启用时要求 `max_beams>=2`、`sigma_hit` 正且有限，防止无效除法/尺度。捕获及首个有效 PF 更新待定期间冻结整段动态配置，整批参数在任何 live mutation 前拒绝；initialpose 与 cleanup 仍可取消。捕获待定时不运行将被初始化覆盖的 PF 运动更新，因此不为未发布的旧粒子消耗运动随机数。

启用后，global 请求令状态进入 `awaiting_first_scan`。旧 `map→odom` 有效位、已知位姿位、观测来源合同和细模式有效位被清除。已有订阅者 TF buffer 的历史记录不能被 AMCL 撤回；本节点停止重播旧 TF，不生成新的已定位 pose。调用者应消费 `global_acquisition_pending`/状态诊断，并按其定位生命周期处理此前缓存。

捕获阶段使用与正常观测相同的有效束索引和 prepared base-reference endpoints；不重新由 raw range 重建已去畸变端点。原始路径使用已有 base 中激光平移及已转换角度。每条日志保存 raw/reference stamp、source frame 和所选原始 beam indices。

对端点集合 \(b_i\) 和地图距离场 \(d_M\)，工程分数为

\[
S_s(T)=\frac1N\sum_i \exp\left[-\frac{d_M(Tb_i)^2}{2s^2}\right]
\mathbf1[d_M(Tb_i)<4s,\ Tb_i\text{ 在地图内}].
\]

机器人中心必须在 free cell 中。该有界平均分用于寻找几何支撑，**不是联合观测似然、后验概率或物理精度置信度**。没有将扫描计分与 PF likelihood 同时应用。没有对候选分数做指数乘积，从而没有将“归一化相对赢家接近 1”作为捕获条件。

2026-10-20 第二轮修订取消了精化前 top4096/top32 截断。旧方法在 `lt_changes_0` 上删除了正确模式和竞争模式，留下的 margin 被人为放大；旧失败证据保留在上述报告。当前方法对**明确的有限格点域**进行带上界的分支定界：平移名义步长 0.05 m，实际为整数地图 cell；全角度名义步长 0.3125°。端点先按每个 yaw 旋转并舍入为整数 cell offset，再加整数平移。该规范避免世界坐标原点消减，但不是任意连续位姿或 PF 世界坐标评分的逐位等价实现。

每个层级都使用全部接纳端点和相同的最终尺度。对平移块 B，每束的距离场分数最大值给出单束上界，所有单束上界的平均给出整块上界 U(B)。max-pool 金字塔与四个覆盖矩形的 dyadic square 实现该上界；float 存储向上取整、double 累加增加保守误差界。第一阶段找到有限域最大分数，第二阶段证明 0.40 m/15° 模式邻域外的候选上界。优先队列与已评分叶子保存完整域的分割，阶段间复用；不因 top-K 或 NMS 删除未排除竞争者。只有整个块都在模式邻域内时才免除其竞争检验。

只有 best≥0.65，且 best−竞争上界≥0.05，才产生有限域证书。`exhaustive_lattice`/`search_complete` 表示该域最大值已经证明，不要求枚举每个叶子；`certificate_valid` 另外要求模式间分离。竞争上界可能不紧，`runner_up_score` 只是实际已求值竞争点的下界，不能把它冒充全域第二高分。模式列表可以只有一个元素。连续三次**完成且有效的新 anchor 证书**还需在里程计 SE(2) 传播后模式一致，才能捕获；失败或歧义打断连续确认。相邻帧分数不作独立概率相乘。

预算保持 3,000,000 次 block/leaf 评分与 512 MiB solver 容量边界；地图准备另有遍历成本。部分根块或子块尚未入队时耗尽预算，竞争上界恢复通用值 1，结果未完成且不能捕获。solver 的容量核算不包含 executor 的地图副本和进程其余内存。executor 另限制地图至 16,777,216 cells。这些是资源边界，不是 wall-clock 硬实时承诺。

`global_acquisition_async` 为只读参数、默认 true。每个请求只有一个后台 worker、一份不可变地图和一个正在处理的 anchor。新扫描在 worker 运行时仅记录 pending 确认；不重复计算证据，不积压扫描任务。完成后才提交 anchor 的 raw/reference stamp、frame、beam indices、搜索耗时和 source age；未捕获则从当前新扫描启动下一次全域搜索。重复或逆序扫描不能收取结果或增加确认。分配失败、未知 worker 异常会清除请求证据；重建、地图变更、外部初值和 cleanup 等待旧 owned worker 退出后丢弃其状态。正常扫描回调不等待搜索完成，但首次地图复制及取消时 join 仍有成本。

捕获后，在证书 anchor 的实际最优模式上进行 `pf_init`，再用现有运动模型把粒子与该模型的运动噪声传播到当前扫描时刻；这不是对当前时刻的重新全域认证。XY 标准差为 \(s_f=\max(2\sigma_{hit},\mathrm{map\ resolution})\)，yaw 标准差为 1°；这只是后续粒子的启动搜索包络，**不是由分数推导的校准后验协方差**。捕获扫描不再经过 PF sensorUpdate，不发布其合成 pose。之后进入独立 `first_pf_confirmation_pending`，initial pose 仍为 unknown。下一条新扫描即使机器人未动也强制执行正常 PF 更新；只有 sensorUpdate 返回 true、本次 `update_count` 实际增加且状态为 `PF_SENSOR_OK`，才允许首次 pose/map→odom 发布并回到原跟踪逻辑。准备失败、无信息、非法观测或陈旧 OK 状态均保持未定位、消费已经传播的里程计参考、对后续新扫描保持强制重试，避免再次应用同段运动。捕获及此确认阶段已消费参考时间戳的高水位阻止同帧/旧帧随后重入 PF。外部初值、地图失效/变更、PF 重建及 lifecycle cleanup 清空捕获状态；一般兼容 PF 路径的来源去重策略没有被本次顺便改变。

完整日志前缀为 `AMCL_ACQUISITION raw_stamp_ns=... reference_stamp_ns=... state=...`。状态包括 `confirming`、`unsupported`、`ambiguous`、`acquired_engineering_gate`、`search_budget_exceeded`、`duplicate_or_nonmonotonic_source`、`invalid_or_insufficient_support`、`consumed_source_rejected`。首次 PF handoff 另使用 `AMCL_ACQUISITION_TRACKING` 前缀，区分失败重试与首个有效更新，不混入全局搜索的模式确认计数。每条捕获处理记录 `pose_published=0 scan_pf_likelihood_applied=0`，同时保存实际求值代表点的 `(x,y,yaw,score)`、竞争上界、有限域间隔、束索引、确认计数、搜索次数和耗时。`completed_search` 与 `completed_certificate` 分别标识新任务完成和其中证书有效；`anchor_*` 与当前扫描身份分开记录。回放器不能在第一条扫描后只等 pose 再发送第二帧，否则多帧确认必然死锁；必须记录初始化待定状态，正常继续后续输入，不填补假位姿。

`amcl_certified_lattice` 在随机小图上独立穷举所有格点并检查原生 block 上界、全域最佳值、模式外竞争界及预算拒绝。当前 `amcl_global_acquisition` 组件验证覆盖非对称场景的全局位置/角度、地图坐标平移、完整对称地图不捕获、重复来源不增加确认、无支撑/非法输入拒绝、预算不足不捕获、SE(2) 传播。`amcl_estimate_reset` 另测试 opt-in 请求清旧 TF/pose 有效性、PF 重建/cleanup 清状态、只读/动态配置及非法束数/尺度拒绝。`amcl_acquisition_handoff` 在同步和异步两种模式使用真实节点与合成非对称图进入实际捕获，核验 anchor 到当前时刻的运动传播，依次注入 TF 准备失败、sensor false、旧 OK 无新更新、无信息扫描，验证没有 pose/TF 泄出、同段 odom 只消费一次、静止新扫描仍重试，直到首个真正有效 PF 更新才发布。它们使用合成地图和真实 map 实现，不能代替 D0/D1 未见数据、实时节点或实际到点精度验证。原生扫描匹配仍基于占据距离场，没有增加遮挡射线一致性、连续表面模型或共享误差标定；本项不能声称解决所有假匹配或精定位偏差。

## 同日第二轮异步交接修订：保留离散运动组合

完整搜索接入后台后，发现把 anchor 到当前时刻的整个运动合成一次 `odometryUpdate` 会改变原模型。现有 Omni 的噪声项包含 `alpha * delta_trans²` 与 `alpha * delta_rot²`；一次总位移的平方通常不等于各个原运动门增量平方之和。保留总 SE(2) 运动不代表保留随机转移核。该问题与浮点下溢不同，也不能靠调小 alpha 掩盖。

当前 executor 在有有效 odom 后、激光准备前记录每个 anchor 后的运动历史，最多 4096 个样本。重复/逆序不重复入账，溢出、分配失败、非有限值或末端不一致都使历史失效；下一次新 advance 丢弃旧 worker 证书、清空确认，不把残缺历史合成大步继续捕获。已经准备失败但原路径会做预测的扫描也能保留运动机会。

节点从 anchor 的独立 odom 参考开始，按原 `abs(dx)>update_min_d || abs(dy)>update_min_d || abs(angle_diff)>update_min_a` 门依次调用同一运动模型；小步持续累计到门被触发。最后的非零余量显式强制消费，使后续 PF 从当前时刻开始；不使用等待期间每帧更新的 live `pf_odom_pose_` 作为历史参考。它保留的是原 scan-gated 离散转移的组合，加一个明示的末段条件；不是物理噪声已重新校准，也不是任意拆细步降低方差。

`AMCL_ACQUISITION_MOTION` 逐项保存实际 motion call 的 anchor/from/current reference stamp、odom、delta、普通门与尾段标志；summary 保存完整历史和实际调用数。`amcl_acquisition_motion` 独立预列 10 类调用序列，再用真实 Omni 和在线门对同初态、同随机种子做 30 组逐粒子/随机数状态一致性检验。

首个输出 covariance 接近零仍可能只是权重集中与重采样单祖先的结果。证书、PF 更新成功、很小的样本协方差各有不同语义，不能替代实际定位精度、观测相关性或地图目标误差的验证。第二轮完整报告在 `output/lilocbench-certified-acquisition-20261020-A02/report.md`，旧过程证据在 A01 同名 epoch；A01 为发现该模型问题后被替代的版本，不混入 A02 的正式结果。
