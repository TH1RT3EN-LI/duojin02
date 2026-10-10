# duojin02 GPU 可选副本

本目录是当前 duojin02 工作区的独立副本，分支 `codex/cuda-optional-20261010`。原始工作区不参与修改。硬件接口、模型、导航主配置和独立速度守卫沿用当前版本。构建开关默认关闭；即使编译 CUDA，主配置仍运行原 CPU 控制器。

本版增加两个可独立选择的后端：

- `compute_backend`：将较大批量的完整底盘足迹与障碍物碰撞查询放到 CUDA；小批量 `auto` 查询继续走原 CPU 路径。接触边界不确定的结果由原 CPU 几何函数复核。
- `sampling_backend`：可选自定义 CUDA MPPI，使用时间插值的控制扰动、全向速度与加速度约束、栅格完整足迹和轨迹扫掠检查。它替换采样分支的优化器，代价函数与 Nav2 原有 critic 组合不同。此模式是实验性算法选项；默认值 `original` 保留原版 MPPI。

最后的速度限幅、实测速度制动检查、指令转换检查、定位门控和独立 `velocity_guard` 继续使用 CPU。AMCL、SMAC 全局规划和 SLAM 没有迁移到 GPU。CUDA 工作使用独立 stream，提交后等待自己的完成事件；没有新增控制器 CPU 工作线程。RViz 的显示桥和频率策略沿用原版本。

## 本机与 Jetson 构建

在具有 ROS 2 Humble 和项目原有依赖的系统上执行。项目本身直接运行，不需要 Docker；本次 RTX 5070 的 ROS/Gazebo 验证使用 Ubuntu 22.04 容器隔离本机 Ubuntu 26.04 环境。

```bash
cd /path/to/duojin02-gpu
# 根据本机实际 CUDA Toolkit 安装位置设置；nvcc 必须可用。
export PATH=/usr/local/cuda/bin:$PATH
# Jetson Orin Nano 的架构为 87：
./scripts/build_hardware.sh -DDUOJIN_ENABLE_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=87
# RTX 5070 使用支持 Blackwell 的 CUDA Toolkit 12.8 或更新版本：
# ./scripts/build_hardware.sh -DDUOJIN_ENABLE_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120
source ./source.sh
```

CUDA 选项也可通过标准 `colcon build --cmake-args` 传入。完全关闭 CUDA 的构建无需 CUDA Toolkit：

```bash
./scripts/build_hardware.sh -DDUOJIN_ENABLE_CUDA=OFF
source ./source.sh
```

本次已验证 RTX 5070 / CUDA 12.8.93 的运行与 `87;120` 编译。Orin 的原生 ARM 构建、JetPack 随附 Toolkit 和实际性能尚未验证；请在 Orin 上重新构建，不能复制本机 x86 二进制文件过去。

## 选择后端并启动

主配置不自动修改。生成单独的配置文件，再传给已有导航启动入口：

```bash
# 首次采用几何加速，保持原版 MPPI：
python3 scripts/gpu_profile.py --compute-backend auto --sampling-backend original \
  --output .cache/gpu/nav2-geometry.yaml
# 同时启用实验性 GPU 采样器：
python3 scripts/gpu_profile.py --compute-backend auto --sampling-backend auto \
  --output .cache/gpu/nav2-sampling.yaml
# 本机第二轮验证的更强探索配置（6400 条）：
# python3 scripts/gpu_profile.py --preset explore --output .cache/gpu/nav2-explore.yaml
# 更低开销的补充候选（3200 条；验证范围较小）：
# python3 scripts/gpu_profile.py --preset explore-lite --output .cache/gpu/nav2-explore-lite.yaml
ros2 launch duojin01_bringup navigation.launch.py \
  params_file:="$PWD/.cache/gpu/nav2-sampling.yaml"
# 在线建图导航同样支持：
# ros2 launch duojin01_bringup online_navigation.launch.py params_file:="$PWD/.cache/gpu/nav2-sampling.yaml"
```

串口、地图和定位初始化使用原项目文档的步骤。这些命令会启动真实硬件节点；本次对比测试只使用仿真。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `FollowPath.compute_backend` | `cpu` | `cpu` 原几何路径；`auto` 按任务量选择；`cuda` 强制设备可用 |
| `FollowPath.sampling_backend` | `original` | `original` Nav2 MPPI；`auto` 有设备时用新优化器；`cuda` 强制设备可用 |
| `FollowPath.gpu_min_work` | `262144` | 几何批处理的姿态数 × 障碍点数阈值 |
| `FollowPath.gpu_deadline_ms` | `40` | GPU 计算及整次控制调用的软时间预算，需小于控制周期 |
| `FollowPath.gpu_support_points` | `8` | 新采样器的时间扰动支撑点数，不减少预测状态数量 |
| `FollowPath.gpu_batch_size` / `gpu_time_steps` | 继承原 MPPI | GPU 独立搜索规模；不增加无设备时的原 CPU 搜索量 |
| `FollowPath.gpu_vx_std` / `gpu_vy_std` / `gpu_wz_std` | 继承原 MPPI | GPU 独立速度扰动标准差，不改变最大速度 |
| `FollowPath.gpu_temperature` | 继承原 MPPI | 新优化器的候选选择温度 |
| `FollowPath.gpu_path_weight` / `gpu_goal_weight` | `12` / `8` | 逐步路径代价 / 末端目标代价，不是原生 critic 权重 |
| `FollowPath.gpu_yaw_weight` / `gpu_cost_weight` | `1` / `2` | 末端 yaw / 栅格软代价权重 |

这些选项在 configure 时读取，更换后端需要重新启动或重新配置生命周期。`auto` 在设备不可用时使用原 CPU 控制器；`cuda` 在设备不可用时拒绝配置。设备计算出错或超时会返回零指令；已经超时的 CUDA 实例不重用仍在执行的缓冲区，需要重新配置。软预算无法中断驱动内的阻塞调用，不构成硬实时保证。CUDA 运行库静态链接，仍需要兼容 NVIDIA 驱动。

清理生命周期时日志中的 `GPU_CONTROL_SUMMARY` 给出真实原 MPPI / CUDA 采样调用次数、最佳候选回退次数、故障和超时；几何后端另给出 CPU / CUDA 批次数；这里只统计后端批处理调用，小任务直接走原 CPU helper 的调用不包含在 `cpu_batches` 中。请先确认目标任务实际进入加速分支，再解释任务性能。

`GPU_SAMPLING_CONFIG` 输出实际采用的搜索规模、噪声和代价权重；`GPU_SAMPLING_SUMMARY` 输出可行候选数量与整次采样耗时。生成器提供 `baseline`、`explore`、`explore-lite` 和仅增加数量的对照档 `coverage`。两个探索档扩大横向搜索并强化目标推进，速度、制动、足迹和默认配置保持当前值；显式增加预测步数还会检查完整全向足迹是否超出局部地图范围。

## 检查与复现

`./scripts/build_hardware.sh -DBUILD_TESTING=ON ...` 可以构建检查，然后执行 `colcon test` / `colcon test-result --verbose`。没有 CUDA 设备时设备专项检查显示跳过；本次设备专项检查实际执行，没有跳过。

独立组件也可在没有 ROS 的系统上构建：

```bash
cmake -S src/duojin01_navigation/gpu -B .cache/gpu-components \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DDUOJIN_ENABLE_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build .cache/gpu-components --parallel 2
ctest --test-dir .cache/gpu-components --output-on-failure
cmake --build .cache/gpu-components --target gpu_geometry_benchmark gpu_sampling_benchmark
```

`tests/performance/benchmark_gpu_components.py` 执行连续 ABBA 对比并保留逐调用 CPU / 墙钟时间、输入哈希、指令和退出码。`controller_gpu_benchmark` 可通过 `DUOJIN_ORIGINAL_SOURCE_DIR` 编译同一观察器对原始源码；链接器计数确认真实 Nav2 MPPI 调用。新增算法的 CPU 数学参考与原版 Nav2 MPPI 属于两种不同对比，报告中分别列出。

首轮方法与完整失败记录见 [首轮性能报告](docs/gpu-performance-report-2026-10-10.md)。第二轮调参、实际绕障轨迹和计算预算见 [调参报告](docs/gpu-tuning-round2-2026-10-10.md)。首轮固定穿障用例失败；第二轮 `explore` 的 12 个任务全部到达，`explore-lite` 的两个补充穿障任务到达。仍有公共仿真收尾异常，探索档在狭窄输入中的采样覆盖仍有限，均保持实验选项。整体导航尚无一致提速；GPU 控制器增加约 100 MiB RSS，Orin 原生预算尚未复测。
