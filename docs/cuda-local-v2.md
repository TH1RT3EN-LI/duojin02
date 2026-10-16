# 本机 CUDA 版本 0.2.0-local-cuda.1

这是提交 `bb08da1` 的历史记录。当前 MPPI/AMCL 实现与复现入口见 [v3 说明](cuda-local-v3.md)；下文的未实现范围和测试命令仅对应该历史版本。

本版本依据 `duojin02_cuda_acceleration_report_source.zip` 的阶段一及几何共享基础设施建议实现。开发目录为 `duojin02-cuda-local`，独立 Git 分支为 `codex/local-cuda-v2`；原工作区和未提交修改未被改写。基线提交 `5451efdb2e54b04ca30f825f38f296819efb38df` 保存了当时的源码状态，原报告源文件位于 `docs/cuda-report-source/cuda_report/`。

## 已实现的路径

- 几何后端拥有输入快照，相同内容复用场景版本与设备数组；地图、足迹、障碍半径变化后重新建立场景。
- CPU 与 CUDA 使用保守空间索引减少障碍扫描，填充足迹、圆盘半径、AABB 和接触容差遵循原谓词；数值不确定的设备结果由 CPU 原谓词复验。
- 调度以空间索引后仍需访问的候选数作为工作量门限，稀疏查询可留在 indexed CPU；`gpu_minimum_work=0` 仅用于强制设备测量。`willUseCuda(size)` 是总障碍数上界预判，实际执行以结果 `used_cuda` 和计数为准。
- 居中跟踪的间隙候选成批计算，保持原 lookahead 分组与稳定排序语义。
- 制动检查保留实测速度的 CPU 早停，将请求与过渡速度候选成批筛选，最终选中指令仍执行 CPU 制动复验。
- 保留纯 CPU 构建、无设备回退、CUDA 故障锁定和独立下游速度保护。严格本机验证要求实际执行 CUDA，跳过的设备测试不能算通过。

CUDA 计算异常会在控制器入口锁定停车，后续小负载 CPU 分支也不能绕过，重新生命周期配置后解除；异常上传路径会先同步所属流，保留快照存储，避免设备读取已经销毁的输入。`use_gpu=false` 保留原控制器串行 CPU 路径；基准中的 indexed CPU 是单独的几何 API 对照，不代表整个控制器已全面 CPU 向量化。

新版本没有提高速度上限、减少安全余量或扩大 MPPI 样本数。CUDA 仅处理明确接入的几何工作；直接跟踪、原生 MPPI、定位与 SLAM 的耗时仍须分别观察。

## 为什么本次没有声称完整 CUDA MPPI

本机镜像安装的 `ros-humble-nav2-mppi-controller` 版本为 `1.1.20-1jammy.20260915.232158`。`MPPIController` 包含具体 `mppi::Optimizer` 对象，`evalControl`、`optimize`、`generateNoisedTrajectories`、`integrateStateVelocities` 和 `updateControlSequence` 均非虚接口，当前继承插件无法替换这些内部计算。

后续完整迁移需要固定版本的优化器源码或独立控制插件，逐项处理七类 critic 的整批依赖、实测速度首步、复用噪声、动态限速、原地 Savitzky–Golay 更新、控制移位和失败重试。应先用固定噪声比较每步轨迹、各 critic 分项、权重与最终序列，再做成对闭环试验。配置开关不能代替这项实现。

## 验证边界

本次验证使用本机 GPU 和隔离 Docker ROS 环境，不连接底盘串口或雷达。独立几何差分与安装插件的固定输入检查能够验证后端执行、输入复用、候选选择及公开控制接口的一致性；不等同于 Gazebo 导航、实物安全、定位精度、能耗或 Jetson 性能结论。

公开插件测试从安装目录用 pluginlib 加载，并核对进程实际映射的两个本地库；六组场景包括空场景、稠密直连、窄道居中、居中加制动缩放、实测制动早停和请求速度缩放。CPU、强制 CUDA 和默认 GPU 门限三个配置使用相同固定输入；每次公开调用的计时包含快照、场景处理、设备调用和 CPU 复验。构造、首轮和稳态时延分开保留，小批量或 CPU 早停占优的结果照常报告。

几何 API 三后端循环调整执行顺序；插件三配置按正序/逆序/逆序/正序测量。每个几何条件 40 次、每个插件条件 24 次稳态采样，其 P99 是小样本描述统计，不能作为运行时尾延迟保证。`cold_backend` 只测热 CUDA 上下文内的新后端构造至查询返回，不含对象析构；`process_first_call` 包含该进程首次后端构造和查询，不含操作系统进程启动。资源 CSV 覆盖基准至导航构建和插件测试，包含桌面其他进程，只能报告采样观察值。

## 本机实测结果

验证时间：2026-10-15 23:09:31–23:10:05（Asia/Shanghai）。RTX 5070、驱动 580.178.04、CUDA Toolkit 12.8.93、ROS 2 Humble。单次固定输入软件实验，未固定 CPU 亲和性，GPU 与桌面共享。

20 项原生测试全部通过，0 跳过；严格设备与编译器缺失负例按预期失败。39,510 项几何差分及 1,600 组三后端配对无碰撞/容差差异，最大间隙差约 `2.78e-17`。安装插件每配置 150 次调用，三配置共 450 次，六组输入的指令最大差为 0。

以下是完整 `computeVelocityCommands` 稳态 P50，单位 ms：

| 固定输入 | 原 CPU 路径 | 默认调度 | 强制 CUDA |
| --- | ---: | ---: | ---: |
| 空场景直连 | 0.0214 | 0.0215 | 0.0216 |
| 稠密直连 | 0.8281 | 0.4506 | 0.1431 |
| 窄道居中 | 1.0692 | 0.4694 | 1.1895 |
| 小场景居中加制动批处理 | 0.0289 | 0.0298 | 0.1465 |
| 实测制动早停 | 0.0176 | 0.0177 | 0.0384 |
| 请求速度缩放 | 0.0211 | 0.0219 | 0.1133 |

默认门限 `262144`。强制配置真实执行 225 个 CUDA 批次，默认配置执行 25 个批次。默认配置在稠密直连和窄道居中分别约快 1.84 倍、2.28 倍；几个小场景仍有少量调度开销。强制所有批次上 GPU 会明显拖慢小场景，因此不能把 `gpu_minimum_work=0` 作为生产推荐。

几何 API 的场景复用模式（含 `setScene` 内容比较）的 P50 示例：

| 工作负载 | 原全扫描 CPU | indexed CPU | 强制 CUDA |
| --- | ---: | ---: | ---: |
| early_collision / collision | 0.0073 | 0.0199 | 0.0611 |
| large_no_collision / collision | 9.7625 | 0.0297 | 0.1315 |
| centered_clearance / collision_and_clearance | 17.7295 | 18.8483 | 1.1837 |

进程首次后端构造至查询返回约 96.00 ms，其中上下文/后端构造约 95.46 ms。场景重建与新对象数据均保存在原始 JSON，不能只用热场景结果概括首次导航调用。

固定栅格夹具在原生 critic 配置阶段保留了 inflation 初始化告警；六组公开插件数据的 `native_mppi_calls=0`。这组时延仅覆盖跟踪和几何检查，不能用于评价原生优化器/critic 的效率。原生 MPPI 限速、重置和无设备行为由独立 CTest 检查。

## 复现与证据

本机已有依赖镜像时，在新版本根目录执行：

```bash
python3 scripts/validate_cuda_local.py --architecture 120 --samples 40
```

默认创建唯一 UTC 命名的运行目录，避免覆盖失败记录；也可用 `--output output/cuda-local-v2/new-run` 指定全新目录。该命令仅构建本版导航组件并安装到 `install/cuda-local/`，完整项目其余 ROS 依赖来自所记录镜像。新机器先用 `IMAGE=duojin02:cuda-local ./scripts/build_image.sh` 准备镜像，再给验证命令追加 `--image duojin02:cuda-local`；本次复用了已有镜像，未另做完整镜像重建。

- [最终验证元数据与源码/产物哈希](../output/cuda-local-v2/validation-r2/validation.json)
- [几何原始配对输入结果和时延](../output/cuda-local-v2/validation-r2/geometry-benchmark.json)
- [安装插件三配置结果](../output/cuda-local-v2/validation-r2/plugin-probe.json)
- [原版工作树及报告来源清单](../output/cuda-local-v2/baseline-manifest.json)
- [已安装 Nav2 头文件/API 审阅](../output/cuda-local-v2/nav2-api-audit/audit.json)
- [可随源码保存的精简证据](evidence/cuda-local-v2/summary.json)

冻结源码集合 SHA-256：`60c1d422946dca6b9a391d3930a6f955e16f03210bb4c364a26cb09fe9ed7412`。源码与构建/安装产物由同一次验证记录绑定，不是跨机器可复现构建证明。此前 smoke、validation 和 superseded 记录均保留；最终以 `validation-r2/status=passed` 为准。

