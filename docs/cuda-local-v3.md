# 本机 MPPI / AMCL CUDA 实现

版本 `0.3.0-local-cuda.1`，分支 `codex/local-cuda-v3`。在 `duojin02-cuda-local` 上继续开发，原 `duojin02` 工作区未改动。目标平台是本机 RTX 5070，不包含 Jetson 适配。

## 源码与实际接入

两个实现固定到 [Navigation2 1.1.20](https://github.com/ros-navigation/navigation2/tree/a097086719c88f781aa59788eca29ac6ca5e56db)，提交 `a097086719c88f781aa59788eca29ac6ca5e56db`。各包的 `UPSTREAM.json` 保存原文件哈希，源码保留上游版权声明。MPPI 使用独立命名空间、pluginlib 注册和库名；AMCL 使用独立包、库名和运动模型注册表。系统安装的 Nav2 库不被覆盖。

| 部分 | CUDA 执行 | CPU 保留 |
| --- | --- | --- |
| MPPI | 固定种子的高斯采样、扰动控制、速度展开、XY/yaw 积分、控制正则项、softmax、加权控制更新、裁剪；Constraint、Goal、GoalAngle、Twirling、PathFollow 五类 critic 的批量评分 | 路径/TF 处理、critic 启用条件与目标点选择、CostCritic 足迹/地图评分、PathAlignCritic、平滑与移位、重试、最终制动复验 |
| AMCL | 粒子×光束似然、普通似然场三次和、概率似然场对数乘积、beam skipping 的一致性计数/掩码、每粒子有序光束累积 | 激光位姿组合、输入束选择、最终权重乘法及总和、归一化、运动模型、恢复统计、KLD 树与自适应重采样、TF/位姿发布 |

`PreciseMPPI` 现在继承新包中的控制器，优化器真实调用 CUDA。直接跟踪与终点微调仍可绕过 MPPI；这类调用不能当作优化器加速证据。MPPI 是混合 CPU/CUDA 优化器：为保留原地图与路径评分，候选数组仍需回传主机。

固定地图导航从项目自己的 `localization.launch.py` 启动 `duojin02_amcl_cuda/amcl`，节点名仍为 `amcl`，当前 `likelihood_field_prob`、120 束、700–3000 粒子和运动模型参数保持原配置。静态链接 CUDA runtime，安装产物无需额外设置 toolkit 的动态库目录。

## 参数、失败与行为变化

- 全局 `use_gpu:=false` 将几何、MPPI、AMCL 三条路径切到 CPU；`true` 使用各模块的配置策略。
- `FollowPath.optimizer_backend`、`amcl.observation_backend` 支持 `cpu`、`auto`、`cuda`。`cuda` 要求设备可用，`auto` 允许无设备时使用 CPU。独立调用控制器 API 时，几何 `use_gpu` 与 `optimizer_backend` 是各自独立的参数；启动层负责全局切换。
- MPPI 设备运行错误会锁定优化器，外层 `PreciseMPPI` 返回零指令并锁定停车；后续直接跟踪分支也不能绕过。生命周期重新配置才解除锁定。AMCL 在 `auto` 下设备故障后原子回退 CPU，禁用故障后端直到模型重建；严格模式报错。
- 默认 `optimizer_seed: 42` 使用 CPU/CUDA 共享整数流的 Box–Muller 高斯采样，便于复现；它不是上游 xtensor RNG 的相同随机序列。`optimizer_seed: -1` 保留原 CPU 噪声生成器，CUDA 仍可执行展开、评分和更新。`regenerate_noises: false` 保留固定噪声复用行为。
- CPU 保留 SSE4.2/AVX2/FMA 指令集优化，但为保留非有限值检查，禁用 fast-math 和隐式 FMA contraction。性能对照中的 CPU 是本源码分支的 CPU 后端，不能直接冒充系统 Nav2 二进制的时延。
- MPPI 限速在平滑后再次应用，并跨 reset/fallback 保留；重试计数改为实例成员。PathAlign 的最近路径点查找修复了查询超过末端时解引用 `end()` 的越界，以及非零起点被误返回为 0 的问题；CPU/CUDA 共用修复后的逻辑。AMCL 的 laser 对象改为明确所有权，地图或模型重建时释放 GPU 缓冲。
- AMCL 概率模型 beam skipping 的无效/填充槽初始化为中性似然 1，修复上游零值或旧扫描值污染；CPU/GPU 两条路径共同应用该修复。当前 `do_beamskip: false` 的有效输入路径保持原公式。
- AMCL 接近取格边界时会回到完整 CPU 观测更新，计入 `boundary_fallbacks`，严格 CUDA 模式也保留该数值保护。`1e-10` 取格阈值经过边界夹具验证，但不构成任意坐标尺度下的浮点形式化证明。射线 `beam` 模型仍为 CPU，严格 CUDA 会拒绝它。

完整接口见 [MPPI 包说明](../src/duojin02_mppi_controller/README.md)和 [AMCL 包说明](../src/duojin02_amcl_cuda/README.md)。MPPI 保留上游 MIT 全文及逐文件 Apache-2.0 声明，新增 CUDA 文件采用 Apache-2.0；AMCL 为 LGPL-2.1-or-later。完整许可证、来源清单和包说明同时随源码与安装包保存。

## 复现

使用已经具有 Humble/CUDA 构建依赖的本地镜像，不映射底盘或雷达。主验证器创建隔离容器，清理旧目标后重新编译，记录源码、安装文件哈希和每一阶段退出码；零测试、设备跳过、源码或安装产物变更均不能通过。

```bash
python3 scripts/validate_cuda_algorithms.py \
  --architecture 120 --jobs 2 --frames 60 \
  --cpu-image duojin02:hardware-humble-cpu
```

`--cpu-image` 指定一个没有 CUDA toolkit 和 GPU 设备的 Humble 依赖镜像，以额外执行完整 CPU 构建、安装、原生测试及定位启动。若未提供此参数，记录只覆盖 CUDA 构建中的 CPU 后端与无设备路径。新的机器需要先按项目 Dockerfile 构建所需依赖镜像；本次使用本机现有镜像，镜像 ID 记录在证据中。

在已进入 Humble/CUDA 环境的容器中：

```bash
BUILD_JOBS=2 CUDA_ARCHITECTURES=120 ./scripts/build_cuda_local.sh
source ./scripts/source_cuda_local.sh
```

纯 CPU 构建使用独立前缀：

```bash
DUOJIN_ENABLE_CUDA=OFF ./scripts/build_cuda_local.sh
DUOJIN_ENABLE_CUDA=OFF source ./scripts/source_cuda_local.sh
```

安装位置分别为 `install/cuda-local-v3` 和 `install/cpu-local-v3`；不要只 source 旧镜像后就假定使用了新库。完整硬件构建入口仍为 `scripts/build_hardware.sh`，会按包依赖关系编译新增包。独立 `localization.launch.py` 默认接受完整导航 YAML；仅提供 AMCL 参数文件时，应显式传 `use_gpu:=true/false`。

## 验证结果

最终使用 `validation-r3`：RTX 5070，驱动 580.178.04，CUDA 12.8.93，GCC 11.4.0，架构 `sm_120`。源码摘要为 `758146dee11ad1e2cc5c3d8c62701984dff4dc53802932b928996f9995191136`。构建前、构建后和测试后的源码相同，两个安装前缀在测试前后也保持相同字节。

- CUDA 构建：MPPI 3 项、AMCL 5 项、导航 22 项原生测试通过，零跳过。
- 无 toolkit、无 GPU 设备的 CPU 构建：2 + 3 + 19 项原生测试通过；设备专用测试不编入或明确排除，不算作 CPU 通过项。
- Python 301 项通过；五个启动入口参数检查通过。CUDA/CPU 安装中的实际定位启动均验证 CPU 与 auto 策略、生命周期 ACTIVE、三个子进程正常退出和进程组无残留。
- 实际 MPPI 插件 96 对同输入调用的最大指令分量差为 `6.7651e-6`，低于预设 `0.005` 门槛。CUDA 实际完成 96 次采样、展开和更新，288 次已启用 critic 批次。生产 `PreciseMPPI` 封装另有 12 对调用，最大差 `1.0431e-7`，确认未绕过优化器。
- AMCL 原生差分覆盖 13 类输入、49 帧，最大归一化权重差 `5.46438e-17`，相同种子的 KLD 采样结果相同。实际节点 60 帧回放的完整输入字段及源戳一致，位姿和协方差逐帧完全一致；CUDA 完成 60 次更新，地图上传 1 次，无边界回退或失败。合成真值位置 RMSE 两者均为 `0.021616 m`。

本源码分支 CPU 后端与 CUDA 后端的中位时间如下，均包含对应范围内的数据搬运和同步：

| 测量范围 | CPU | CUDA |
| --- | ---: | ---: |
| MPPI 完整 `computeVelocityCommands`，三条路径，1600 × 56 | 2.960–3.036 ms | 1.607–1.659 ms |
| AMCL `sensorUpdate`，700 粒子 × 120 个实际选中束 | 1.956 ms | 0.194 ms |
| AMCL `sensorUpdate`，3000 粒子 × 120 个实际选中束 | 8.360 ms | 0.431 ms |
| 实际 AMCL 节点，扫描发布至同源戳位姿接收 | 13.254 ms | 2.892 ms |

MPPI 每条路径各 32 次同输入调用，稳态表排除首调用；首调用原始时间仍保留。AMCL 组件计时预热 5 次、测量 30 次，含 `pf_update_sensor` 归一化而不含重采样。节点回放计时包含全部 60 帧，其 P95 为 CPU `13.703 ms`、CUDA `3.389 ms`；配置上限为 120 束，本夹具按原生抽样规则实际选中 108 束，粒子数经 KLD 在 701–3000 间变化。这里比较的是固定输入与固定参数，不代表所有地图或负载下的速度。

进程内 `cudaDeviceReset` 故障注入后，生产封装首个调用输出零速度，故障计数增加一次；换为可直接跟踪的短路径后仍输出零，GPU 完成计数不再增长。首个故障调用耗时 `80.260 ms`，这项检查证明错误处理和停车锁存，不证明控制周期截止时间。两个理想运动学轨迹均在 106 步内进入 8 cm / 0.08 rad 容差，末速约 0.129 m/s；这只是跟踪入口检查，不是停稳到达或 Nav2 Action 完成验证。

版本化证据：[摘要](evidence/cuda-local-v3/summary.json)、[构建与哈希](evidence/cuda-local-v3/validation.json)、[MPPI 实际插件](evidence/cuda-local-v3/mppi-probe.json)、[AMCL 实际节点](evidence/cuda-local-v3/amcl-replay.json)。完整原始日志在 `output/cuda-local-v3/validation-r3/`。PathAlign 的 ASan 缩减复现也随证据保存。

此前失败和修正过程保留在 `output/cuda-local-v3/`：r1 暴露 PathAlign 越界导致的指令偏差；r2 通过功能对比，但 MPPI 夹具的地图/膨胀层初始化分辨率不一致；r3 修正夹具并补齐安装元数据后重新完成全套验收。最终数据不混用这些历史轮次。

## 证据范围

对照包含同输入组件检查、实际 pluginlib 控制调用、理想运动学跟踪，以及实际 AMCL 节点的合成地图/里程计/激光回放。它们不能替代完整 Nav2 任务闭环、物理传感器数据、实物制动、定位可靠性或 Jetson 性能验证。公开控制调用时间包含路径变换、CPU critic、传输、同步和控制更新；AMCL scan→pose 时间包含 DDS、TF 过滤、PF 与调度，观测回调单项时间范围另行列出。GPU 与桌面共享，未进行独占、能耗或统计可靠性声明。
