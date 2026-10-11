# 路径候选 CUDA 实现与 RTX 5070 对照（2026-10-11）

本轮在独立 GPU 副本实现 `path_primitives` 候选模式，并将最小代价、指数权重和权重和归约移到 GPU。路径候选改善了原随机采样器在理想 L 转角中的覆盖，但真实 ROS/Gazebo 的 50 cm 转角仍失败，60 cm 转角的完成时间与旧版接近。保留实验选项，主配置仍使用原控制器。

工作区 `/home/th1rt3en/dev/forge/duojin02-gpu`，分支 `codex/gpu-structured-mppi-20261011`；对照基线为 `e3969fb`。原始 `/home/th1rt3en/dev/forge/duojin02` 未参与本轮编辑。

## 实现与入口

新增候选把当前估计姿态投影到剩余路径，合并前向共线边，保留弯折。12 组不同增益和限速比例的候选按照实际全向动力学推进，在分段点减速后切换方向；前 12 条无扰动，更多候选保留小幅探索。默认占比 25%，剩余候选继续使用原时间扰动。yaw 在终点附近跟踪目标角度，避免零角速度保存已有姿态误差。

完整轨迹的栅格足迹检查、CPU 加权序列复核、最佳合法候选回退、最终制动检查、定位门控和独立速度守卫继续生效。没有降低几何余量或增加速度上限。该实现受路径运动基元思路启发，属于本底盘候选族，未实现论文中的离线 OCP 基元库、SCP 或 SVGD。

只有新模式会在下一处明显弯折前提前选择采样分支；同一模式下的直线跟踪和精确停靠仍保留原快速路径。归约移到 GPU 后，中途的代价读回、CPU 权重计算和权重上传被移除，最终代价与控制序列仍读回，CPU 复核继续计入完整调用时间。

```bash
python3 scripts/gpu_profile.py --preset structured \
  --output .cache/gpu/nav2-structured.yaml
# 构建并 source 后，传给项目现有启动入口：
ros2 launch duojin01_bringup navigation.launch.py \
  params_file:="$PWD/.cache/gpu/nav2-structured.yaml"
```

启动命令使用真实硬件入口；本轮只运行了仿真。构建步骤、后端选择与参数默认值以 [README_GPU](../README_GPU.md) 为准。无设备时 `sampling_backend=auto` 回到原 Nav2 CPU MPPI，而不是运行新增候选的 CPU 数学参考。

## 配对结果

| 测量范围 | 旧版 | 新模式 | 解释 |
| --- | --- | --- | --- |
| 7 个静态诊断输入，128 条，2 种子 × 10 次 | CUDA 0/140 有合法序列 | CPU / CUDA 各 140/140 有合法序列 | 输入是近似静态诊断，不是实时地图重放 |
| 理想运动反馈：50/60 cm 正反向转角 | CUDA 严格到达 0/8 | CPU / CUDA 各 8/8 | 独立填充足迹检查；无定位、通信、守卫延迟 |
| 理想运动反馈：上述转角 + 开阔 + 绕障 | CUDA 严格到达 0/12 | CPU / CUDA 各 12/12 | 旧版指自定义随机采样器，不代表原 Nav2 整机成功率 |
| 1600 条完整采样，7 个相同静态输入 | 新算法 CPU 参考 38.27–47.23 ms | 新算法 CUDA 5.91–7.24 ms | 同算法约 6.35–7.15 倍；不是原 Nav2 CPU 加速倍数 |
| 原 Nav2 / 旧 CUDA / 新 CUDA 公共控制 API | 2.642 / 2.266 ms | 2.436 ms | 各 200 次，固定开阔偏离路径输入；不代表转角整机速度 |
| ROS/Gazebo 50 cm L 转角，2 种子 | 0/2 严格到达 | 0/2 严格到达 | 新模式 595 / 556 次实际 CUDA 采样，仍在转角前停止 |
| ROS/Gazebo 60 cm L 转角，2 种子 | 2/2，10.603 / 10.604 s | 2/2，10.603 / 10.603 s | 新模式实际采样 100 / 93 次；端到端时间基本相同 |

组件的合法初始几何任务与 8 次最终 ROS 试验均未观察到独立评估器的平面足迹碰撞。`blocked` 反例的初始足迹已覆盖未知栅格，所有模式均拒绝；原始 `collision_events=1` 是初始几何无效记录，不纳入可行场景的新碰撞分母。两种子仅构成小样本对照，不能推导通用可靠性。

严格到达要求位置 ≤2 cm、yaw ≤0.035 rad、停止速度 ≤0.02 m/s，并持续 ≥0.25 s。理想组件另检查角速度 ≤0.02 rad/s；ROS 的实际标准和因果真值轨迹保存在原始报告中。FollowPath 使用显式 L 路径隔离局部执行，未评估全局重规划、原生 Orin、真实底盘、远端 RViz 多订阅或长程连续任务。

## 尚未解决的闭环问题

最终 50 cm 新模式的两个任务停止在真值 x≈−0.056 / −0.042 m、y≈0.001 / 0.004 m。第一种子 573/595 次采样输出有效，但合法序列不等于第一条指令有推进量，也不等于指令通过后续制动与权限链。已有 `/cmd_vel_nav`、请求输出和 `/cmd_vel` 记录显示后段指令链停滞；尚无逐周期采样第一步、CPU `checked()` 拒绝理由与权限判定的完整对应证据，因此没有把失败归因于某一个守卫条件。

8 次最终 ROS 试验均无控制周期警告、CUDA 故障或超时，正常收尾。当前数据支持“改善候选覆盖且计算预算充足”，不支持“已解决 50 cm 实际转角”。下一步应在不改变边界的情况下冻结真实局部地图、估计姿态、实测速度及选中/复核/准入指令，定位首次失去推进的周期，再决定调整候选、路径进度状态或终点/守卫衔接。

## 验证与证据

RTX 5070、CUDA 12.8.93、驱动 580.178.04、Humble/Nav2 1.1.20。编译 `87;120` 仅证明架构代码可生成，本轮没有原生 Orin 部署或功耗结论。桌面已有 Isaac Sim / cuVSLAM GPU 作业，保留其运行，测量属于共享工作站条件。

CUDA 完整导航构建 21/21 检查通过（设备专项实际执行），CUDA-OFF 构建 15/15 通过，参数生成器 17 项通过。隐藏 GPU 的公共 API 30/30 调用进入原 Nav2 CPU，零指令 0 次。CPU/CUDA 数学一致性、扫掠、速度区域、动态限速、设备缺失及超时停止由原生检查覆盖。

原始结果、逐调用耗时、轨迹、源码/二进制冻结和失败阶段位于 [完整报告](/home/th1rt3en/Documents/Codex/2026-10-11/outputs/gpu-structured-controller/report.md)。v1 的残余 yaw 失败、初次 Gazebo 模型路径启动失败、v2 的 CPU 快速路径绕过 CUDA，以及最终 v3 的 50 cm 失败均保留，未覆盖成成功结果。

源码与复现观察器：`src/duojin01_navigation/gpu/sampling_math.hpp`、`gpu_backend.cu`、`src/precise_mppi.cpp`、`tests/performance/structured_sampling_benchmark.cpp`。独立观察器命令格式：

```text
gpu_structured_benchmark backend mode case seed batch steps repetitions output.json [recorded-input.txt map.bin]
```

`backend` 为 `cpu` 或 `cuda`，`mode` 为 `legacy` 或 `structured`；`case` 可选 `corner50`、`corner60`、`reverse50`、`reverse60`、`open`、`detour`、`blocked`、`recorded`。`repetitions=1` 的生成场景执行理想闭环；`recorded` 与重复调用测量静态输入。旧基线编译同一观察器时不定义 `DUOJIN_STRUCTURED_API`，只允许 `legacy`。
