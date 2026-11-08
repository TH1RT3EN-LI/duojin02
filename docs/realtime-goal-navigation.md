# 任意地图目标、控制消费与任务评价

本次是 E0 来源/时间合同与 E6 采集/评价接口。现有 `converge`、XY 复查、连续停稳 dwell、smoother→mux→guard 命令链和 4 mm 估计状态容差均保留。执行器的时延、死区、负载响应没有标定，所以没有改控制律、增加 vτ 或新增基于观测年龄的硬门；4 mm 仍不代表物理 4 mm 保证。

## 原始请求与地图版本

导航入口会自动运行被动采集器，并默认在 `ROS_LOG_DIR`（未设置时为 `~/.ros/log`）生成唯一的 `task-goal-*.jsonl`；启动日志显示实际路径。文件持续记录相关话题，长时间运行需要按磁盘容量管理。也可在同一 ROS 图内独立运行并指定文件：

```bash
python3 -m duojin02_bringup.task_goal_capture --output /absolute/new-directory/task-ledger.jsonl --ros-args -p use_sim_time:=true
```

指定的输出文件必须不存在；独立运行时 `--no-record` 可关闭文件记录，在线发布仍然可用。节点仅订阅 `/goal_pose`、`/map`、命令链、odom 与诊断，不发布速度或导航 Action。`task_goal_contract` 使用 transient-local `diagnostic_msgs/DiagnosticArray`，保留原始 binary64 position/quaternion、frame、原 stamp、地图内容身份、节点内 generation、`base_footprint` 工作点与转换来源。地图身份与 AMCL 共用规范：UTF8 frame+NUL、little-endian `uint32 width,height`、8 个 double（resolution, origin xyz, quaternion xyzw）、int8 cells 的原始字节，再 FNV-1a64；`occupancy_fnv1a64:` 后 16 位 hex。generation 是各节点本地计数，不作跨节点版本相等的依据。

RViz `/goal_pose` 能被记录；直接 Action 请求没有公开原请求 topic。该路径需要客户端另存任务 ledger 或发布同一原请求。控制器对 plan endpoint 与观察到的原请求只作 exact 数值核对：`exact_endpoint_pose_match_action_identity_unknown` 不证明唯一 Action 身份；相同坐标的旧请求也可能匹配。未收到原请求、坐标转换过或规划端点改写时记 `unmatched_or_unavailable`，不能把 planner endpoint 冒称用户原浮点请求。目标不会因定位变化被自动迁移。

## 现有控制器真实消费

`PreciseMPPI` 消费上述合同及 `amcl_estimate_contract`，每个正常返回/计算故障刹停周期旁路发布 `task_state_at_control`。数值控制仍使用 Nav2 提供的 pose、measured twist 和现有 plan endpoint。合同记录实际终点转换的 exact query/latest fallback 及查询和使用 stamp、同一消费目标在车体系中的 XY/yaw 误差、measured twist 和本周期请求速度。

`control_boundary_steady_ns` 在等 mutex 前，`control_consumed_steady_ns` 在获得 mutex 后，`command_available_steady_ns` 在计算返回后。它们是同主机 monotonic 时间。Nav2 的 measured twist 接口没有 stamp，故明确 unknown。TF stamp、AMCL 物理 reference stamp、first available steady stamp 保持独立。没有在已经传播的 pose 上再次补偿同一段 vτ。

TF 与 AMCL 诊断是独立 topic；`estimate_association=latest_received_diagnostic_not_atomic_with_tf` 明确表示尚不能证明最新旁路诊断就是本周期实际 TF correction。map identity 不一致只报告，不改变控制。`task_uncertainty`、`command_queue`、`actuator_model`、`physical_arrival_guarantee` 中未标定项保留 unknown。这是可以检查的来源接口，并未完成原子化状态所有者或物理误差模型。

采集器记录各命令 topic 的本机收取时间与 odom 的源 stamp/测量速度。普通 Twist 没有 command ID，收取时间也不是机械生效时间，因此不能据 topic 间的时间差声明执行器延迟。任务文件可补充已辨识的 `command_application_pairs=[{command_id,issued_ns,applied_ns,clock_scope}]`，评价器才计算记录对的延迟分位数。

## 冻结任务评价

```bash
python3 -m duojin02_bringup.task_goal_evaluator \
  --protocol /absolute/protocol.json --tasks /absolute/tasks.json \
  --samples /absolute/samples.jsonl --output /absolute/new-evaluation.json
```

protocol 必须在所有任务开始前冻结，包含单一 `registration`（id、map_to_truth 的 SE(2)、frozen_at_ns、evidence、physical_calibration）以及评价 `limits`（xy_m、yaw_rad、linear_mps、angular_rps、dwell_s、max_sample_gap_s）。所有 tasks 都进入分母；每项包含 id、原始 goal_map、start_ns、deadline_ns、status、completion_ns，可选 base_to_task。samples 包含 task_id、stamp_ns、独立 truth_base 与在底盘坐标轴表达的 truth_twist。严格递增 stamp、不作任务后对齐或重新定义目标。

验收要求完成时刻有同 stamp 的独立真值，再同时要求位置、朝向、工作点平移速度、角速度与连续 dwell；不把最后一个有效采样延续到未来完成时刻，当前没有插值/运动上界支持时明确失败。缺真值、采样断裂、完成前漂移、失败和超时均不能算通过。工作点速度包含旋转杆臂项。报告终态误差、首次联合停稳时间、完成耗时、沿初始真实接近方向的最大 overshoot、有效/非法真值数、采样缺口及可确认的延迟；无初始运动基线时 overshoot 只能解释为相对首个有效采样的量。单次或合成任务比例不构成 ≥95% 的总体保证。

## 可执行组件与剩余证据

`tests/realtime_goal/navigation/run_native_components.py` 在独立新输出重编当前 goal checker 与 controller limit checks。`run_native_task_loop.py` 运行真实当前 PreciseMPPI 和真实 StableStoppedGoalChecker，使用明确声明的合成一阶 plant（τ=.15s、delay 0/4步、bias 0/.04m），保留独立合成 truth、完整任务和冻结注册。它还通过真实 ROS topic 验证合同的 publish/consume 与时间顺序。该组件没有启动导航 Action、外部 smoother/mux/velocity_guard、Gazebo 或硬件。

现有 sibling 仿真仅复制到本轮 output 的 `navigation/sim-compat`，原树不改。当前脚本要求的 `chassis_profile.py`/`chassis.yaml` 在原树、目标树与现有 sim image 中均缺失；其材质、激光与运动参数无法从已声明原合同读取。保留真实 preflight 失败，不能编造 profile 后冒称原 world/sensor/motion 保持，也不能把原生组件替代成完整 Gazebo 任务验收。
