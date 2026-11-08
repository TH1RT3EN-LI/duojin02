# 建图 E2 / E3 / E5 的实现范围

本轮修复 Karto 的返回分数对象语义，新增可执行的原始扫描连续几何与真实刚性局部地图影子路径。**没有完成全局 scan-to-submap 后台替换、固定滞后边缘化或持续实时输入验收，也没有证明新的 ATE 或 2cm/1° 导航精度。**

2026-10-19 历史输出：`output/lilocbench-realtime-goal-implementation-20261019-212448/mapping/`。2026-10-20 根因修正与新测试：`output/lilocbench-root-repair-20261020-100838/mapping/`。历史输入只读，未操作硬件。

## E2：实际输出位姿的分数默认返回

持久修改为 `patches/slam_toolbox/realtime-goal-match-semantics.patch`，在 `source.json` 有序清单末尾显式登记，prepare 验证完整源码树。不是只修改 `.cache`。

`ScanMatcher` 默认保留现有最高响应点均值作为位姿，但现在返回该**实际位姿**在同一 correlation grid、同一源束分母和同一搜索惩罚下的复评分。`GetLastMatchDiagnostics().peak_score` 单独保存候选格点最高响应。`SetReturnSelectedPoseScore(false)` 显式恢复历史峰值返回语义。没有改变关联门、smear、噪声权重或搜索分辨率。

`SetSelectAttainedMode(true)` 是可执行的 C++ 实验入口，默认仍为 false；没有新增 ROS 参数，也没有默认部署尚无自然完整图/物理收益证明的模式重选。它不再机械选择遍历首峰：把候选格点上等高最高点按六邻域拆成平台，以原搜索中心为声明的 tie-break 先验，使用既有 distance/angle variance 尺度选择最近平台。平台内只在其圆周角均值/XY 均值实际复评分达到同峰（既有 `DoubleEqual` 容差）时使用均值；非凸平台均值落空时投到平台内原生最高格点。精确对称 tie 保留 `selection_tied=true`，最终格点顺序仅用于不可消歧时确定性返回，不能称为真峰推断。

`SetCollectMatchDiagnostics(true)` 或上述模式选择均保存平台代表、质心、大小和搜索中心距离；不枚举较低局部峰，也不提供模式概率。每个 `MatchScan` 的 coarse/扩展/fine 阶段保留独立中心、候选 lattice、输出位姿、peak/actual score、模式及歧义，最终 `unresolved` 是各阶段的历史歧义 OR，不能解释成最终 posterior 概率。不同 matcher 的外层 coarse/fine 结果仍分别读取其诊断，未新增跨调用贝叶斯融合。

默认路径新修复搜索中心与输出 `rMean` 别名：fine 调用常把同一 Pose2 作为输入和输出；进入 `CorrelateScan` 立即冻结输入中心。否则输出赋值后复评分会相对新输出计算 prior penalty，错误漏罚。现在搜索、shape、实际复评分和 stage 元数据使用同一原始中心。

返回时仍持有 immutable query/reference view bytes、reference pose/source roster、估计器参数及所有阶段 lattice。默认不再复制整张 correlation grid；实际输出 score 仍在同一未变化的 live grid 上原生复评分。`SetCaptureCorrelationGrid(true)` 才额外拥有完整 grid bytes，用于离线重放。默认 status 明确为 `exact_owned_query_reference_pose_lattice_grid_not_retained`，不能据此声称保存了逐字节 grid。生产图 `measurement::Evidence` 的来源/拥有合同不变。

2026-10-20 新验证：UBSan 原生单峰、双峰、空支撑、边界、跨 ±π，以及新增搜索中心左右偏移选峰、对称 tie、环状非凸平台和 coarse/fine 输入输出别名组合均通过。433 个冻结真实候选和 80 个 fine：默认实际分数路径相对 legacy 的位姿/gate 不变；实验 supported mode 改变 4 个 coarse 位姿、2 个 fine yaw（最大 coarse 10cm / 0.1396rad，最大 fine yaw 0.0008725rad），gate 不变。历史门标签不是物理正负真值，不能据 gate 一致证明选峰正确。完整数据与 capture 成本单独见新输出 `report.md`。

以下是 2026-10-19 的历史验证，不能替代上述新源码验收：

验证：

- 当前旧源码独立新编的 4/10/20cm 双峰反例仍复现峰值 1、均值位姿实际分数 0；新 score-only 返回 0，实验 attained 返回实际峰分数 1。
- 新编完整 Karto.cpp/Mapper.cpp，启用 UBSan 和 float-cast-overflow，单峰、分离等峰、空支撑、边界和跨 ±π 均执行。
- 全部 433 个预先冻结的历史 explicit-loop attempts（D0 221 / D1 212）重新执行真实原生 coarse，以及所有通过后的 fine（80 次）。legacy 与 score-only 的 coarse/fine gate 变化均为 0，位姿差为 0。旧 coarse 的 peak 与实际位姿分数有 4 次不同，最大 0.0229839704；fine 有 2 次不同，最大 0.0000863132。新语义全部与实际复评分一致。
- 初次 score-only 版本的 shared-host 计时中，legacy/new median 为 5.826/5.980ms，P95 为 41.624/42.111ms。A07 完整支撑所有权版本重新执行全部 433 项，最新成本在 `E2/A07/full-pool-summary.json` 独立记账，未复用前版计时。这是冻结候选 CPU 调用成本，包含验证复评分，顺序执行且共享主机，不能据此宣称隔离性能或传感器到控制端实时性。

整套新编后端另在独立 `backend-native/A02/install` 编译完成，并运行未修改的 `slam_replay_probe.py`：24 scan 合成 recorded 路径完成 24/24，EKF 预声明 warmup 跳过 5 后完成 19/19，invalid/outstanding 均为 0；Snapshot 保存、归档 reload 正常 rc0、corrupt graph 拒绝 rc1 且无强杀，全部原断言通过。新 network-none 容器无 GPU/硬件 device，旁路采集 7 组 `/proc/*/maps`，Karto、toolbox common、async backend 与 Ceres 插件均实际加载新 prefix 并逐库 SHA256 核验。A01 曾因输出目录 UID 权限在 probe 启动前失败，原日志保留，A02 以宿主 UID/GID 重跑。此为后端合同烟测，不是实际长序列建图或外部精度测试。证据在 `native-backend-smoke-A02/`。

全部样本和不变的门一起保存；正/负是历史门标签，不是独立物理关联真值。全池测试不重新注册图因子，不代表自然分支图回放已通过，更不代表 ATE 改善。

## E3：原扫描连续几何影子

`continuous_geometry.py` 从冻结原始扫描端点构造相邻束有限线段，显式跳过源束不连续、过短和跳变过大的片段。空间哈希只筛对应候选，残差本身是连续点到线函数。局部 Gauss–Newton 使用显式 Jacobian、Huber loss 和固定对应的回溯，保留单模式、初值和搜索半径限制，不写 PF 权重或 Karto 图。

`RawObservation` 保存 source、raw/reference stamp、端点、原始 beam index、每束 ray origin（可显式 unknown）、geometry kind、原始 payload/config identity 与 calibration version。实际 prepared 输入从原生保存的 prepared endpoint/origin 导出，没有用 ranges 重新刚性生成。identity 由显式字段计算，嵌套输入转为 tuple，外部 list 改动不能悄悄改变缓存的 geometry identity。

Hessian 只叫 `conditional_shape`，按声明的 task length scale 检查特征值，不 flooring 弱方向。`observable` 仅描述指定对应与参考下的局部秩；**visibility 始终 unvalidated，usable_for_precision 始终 false**。没有完成遮挡、可见性或动态物诊断。结果保存初末支撑数、支撑变化和 retention；丢失过多初始支撑会报告 `support_lost`，不能用删掉难点后的低残差冒充成功。

64 组有限差分、丰富/走廊几何、刚体等变、嵌套不可变、source/receipt 和预算失败保留等 7 项 unittest 通过。24 个预定 ±1/2/4mm 初值相位与四方向控制通过；它们只证明该合成局部模型的收敛，不构成优于原栅格或物理精度的比较。含额外条带的支撑丢失反例被明确报告为失败。

真实冻结输入来自上面全池中每臂按历史顺序的首个正/负，共 4 组，无 GT，query 从 reference source roster 排除。最终执行在 `E3/A02/`，逐点 residual、Jacobian、query/surface source、payload geometry identity 均保留。CPU Python 影子单样本耗时约 0.77–6.34 秒，**未达到实时要求**。参考位姿来自历史估计，数据仍可能含内部畸变，低残差不能证明其物理正确。

## E5：真正刚性参考与不丢 accepted 更新的影子

`local_submap.py` 的 `RigidSubmap` 内部 `FrozenScan.local_pose` 与所有 endpoint geometry 固定；共同 world pose 变化只增加 pose_version，不改变 geometry_version。改变任一内部 scan pose/point/source 配置会产生新的 geometry identity，`ScanSubmapObservation.validate_support` 拒绝旧测量继续绑定新几何。

`ScanSubmapObservation` 是真实 query-to-rigid-submap 的影子观测表达，记录 local pose、完整 reference ancestry 和 conditional shape。它不选择 nearest scan 代替整个参考，不注册 Ceres 因子，也不声称为已重构的全局后台。

builder 在 query-before-insert 的 `QueryReceipt` 上检验 exact query identity、当前 reference geometry 和 source roster；同 source 仅插入一次，改测量重用 source 会拒绝。实际 4 组 query 在评分后才作为历史已接纳 keyframe 加入后继 geometry version；原评分的不可变 view 与测量支持仍保留。此接纳身份来自历史归档，不是新连续 matcher 的质量认证。重叠地图保留共享 raw source roster，不从重叠次数猜相关系数。

`BudgetedUpdateQueue` 在 accepted 时登记全部 update，只有整个 callback 成功才移除队首；时间/条数预算到期及 callback 失败均保留 pending，后续继续。callback 必须原子或幂等，一次 callback 不可抢占、可能超预算。测试验证 10 个 accepted 更新经历失败、分批执行后全部完成；真实 4 组全部 reference+query accepted 更新闭合。

另有 `shadow_submap_graph.py` 实现独立的 S_j / X_q 小图后台影子，使用 caller 明示的未校准 PSD 实验 shape、SVD 零空间、accepted-prefix 原子提交和分预算续算。非作者复核发现并修复同 query 被拆成多个 X 的反向来源所有权缺口；最终 10 个测试独立通过。真实 4 query / 8 rigid submap / 8 因子执行保留全部 ledger 和 geometry，固定 anchor 不动；它们是 4 个连通分量，另外 3 分量没有 anchor，所以明确报告 9 维 gauge nullspace。这个给定候选的实验拟合没有关联真值或联合噪声校准，不能代替生产整图重构。独立执行与审查见本输出 `mapping/E5/review-background.md` 和 `review/E5-real-independent-A01/`。

尚未实现：生产全局 submap 状态和因子提交、内部重建后的全局因子重算、正确固定滞后/边缘化先验、持续输入的稳定吞吐/队列上界与后台求解预算。现有 Ceres 全图所有已接纳因子合同未被替换或削弱。现有源码全编和这些局部测试不能关闭上述门。

## 复现与失败保留

`tests/realtime_goal/mapping/run_native.sh OUTPUT SOURCE` 在现有 ROS 容器新编全部 Karto 源码并运行反例、真实冻结样本和全候选池；`summarize_pool.py` 输出所有门差异。`test_shadow.py` 与 `phase_probe.py` 使用 NumPy；`run_shadow.py --input .../frozen-real-scans.jsonl --output NEW` 执行真实连续/刚性地图影子。

首次 native 启动曾早于 verified source 就绪而失败；第二次发现复评分角度 offset=0 被既有严格 lookup 合同拒绝。两次日志保留，没有删除或算成通过；修复使用严格正浮点相邻 angle support，在同一点计算 response。后续 A03–A06 为新的源码/测试 epoch。所有产物哈希、源码 tree 校验、阶段 diff、命令和失败说明见输出 manifest。
