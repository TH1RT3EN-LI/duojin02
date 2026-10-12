# Orin 原生路径候选验证（2026-10-11 / 10-12）

新版在 Orin 上已原生编译并测试，独立目录为 `/home/dj/duojin02-gpu-structured-20261011`。结果支持保留可选实验模式，继续使用原默认控制器；没有修改原部署或设为自启动。

测试算法冻结在 `38be60d5a1fa29f805b829653888dc0382b10194`，旧 CUDA / 原 CPU 对照为 `e3969fb`。本轮后续只更新报告和文档，端侧仍是已冻结的算法版本。设备为 Orin Nano Super 8 GB、L4T 36.5.0、CUDA 12.6.68、SM 87；MAXN_SUPER / schedutil 保持原值。580 个源码文件和 7 个新二进制在测试结束、10-12 重启前后核验一致，原两个部署目录未改动。

## 效率与质量

| 测量范围 | CPU / 旧版 | 新 CUDA | 结论 |
| --- | --- | --- | --- |
| 相同复杂地图、新算法、128 条，7 输入 × 2 种子 × 10 次 | CPU 数学参考组均值 13.23–16.65 ms | 22.86–34.39 ms | 小批量 GPU 较慢；两者各 140/140 合法输出 |
| 相同输入、新算法、1600 条，每组 3 次 | CPU 136.74–174.87 ms | 67.66–101.99 ms | GPU 约快 2 倍，仍超过 40 ms 控制预算 |
| 开机后公共控制 API，各 160 次 | 原 CPU 10.276 ms；旧 CUDA 6.444 ms | 6.716 ms，P95 7.354 ms | 新模式特定输入约快 1.53 倍；没有零指令、超时或故障 |
| 理想运动反馈：合法转角 + 开阔 + 绕障 | 新 CPU 参考 12/12；旧 CUDA 0/12 | 12/12 | 组件覆盖改善，不代表实际闭环修复 |
| PC 仿真、Orin 导航，60 cm 转角，2 噪声种子 | 原 CPU / 旧 CUDA 各 2/2 | 2/2，10.254 / 10.403 s | 三种模式完成时间接近 |
| 同样条件，50 cm 转角 | 原 CPU / 旧 CUDA 各 0/2 | 0/2 | 仍未解决，新模式停止得更早 |

完整 12 次闭环使用同一显式 L 路径的 FollowPath，地图 1 cm，里程/激光噪声开启。AMCL、控制器与独立速度守卫在 Orin，Gazebo 在电脑。未启动实体驱动，也没有加入全局重规划并发、RViz 订阅压力或长程任务。严格成功根据 Action 结果接收时已收到的独立真值判断：位置 ≤2 cm、yaw ≤.035 rad、线速度/角速度 ≤.02、停止 ≥.25 s、无碰撞。

新模式的四次闭环实际 CUDA 采样为 583/497/105/100 次，整次控制调用超时分别为 26/7/6/1，共 40 次，另有一次采样器全调用超时异常。采样器均值约 24–25 ms 不能代表包括地图、TF、准备及最终检查的整次调用符合 40 ms 预算。旧 CUDA 60 cm 的两次成功未进入采样分支，不能归因于 GPU 优化器。所有闭环均未观察到平面完整足迹碰撞。

CPU 数学参考目前仅用于组件；`sampling_backend=auto` 无设备时仍回到原 Nav2 CPU，而非新增路径候选的 CPU 参考。不要直接把桌面 `structured` 的 1600 条预设用于 Orin；本轮端侧使用 128 条、56 步、4 个扰动支撑点及原速度/足迹/制动边界，仍未满足整图预算。

## 检查、供电与证据

原生 CTest 为 20/21，设备专项实际执行。固定 3200 条的 `precise_mppi_structured_limit_checks` 因 CUDA 完成超时，随后推进量断言失败；未放宽预算或改断言。初次观察器误加载旧同名 GPU 库而 abort、容器默认入口启动失败、开机后 wrapper 与 ROS setup 的 nounset 兼容失败，都在独立原始阶段保留。修复观察器后按 arm 核验 ldd，实际图核验映射库哈希。

INA3221 标为 VDD_IN 的采样为 5.056–5.080 V，采样估计功率 5.00–8.66 W，最高温度 53.156 °C，OC1/2/3 采样均为 0。这些低频传感器数据不能排除短暂跌落，也没有测量外部电源插口电压或证明节能。所有自有进程组已退出；旧 CUDA 的一次 localization_manager 收尾退出 1，原始栈保留。

下一步应先提供新候选的实时 CPU 对照，并分解整次调用的额外成本，再决定 GPU 批量、常驻数据及准入策略；继续保持 40 ms 预算和现有边界。

- [完整方法、12 次结果、供电、失败及复现报告](/home/th1rt3en/Documents/Codex/2026-10-12/outputs/orin-structured-controller-resume/report.md)
- [端侧闭环机器可读汇总](/home/th1rt3en/Documents/Codex/2026-10-11/outputs/orin-structured-controller/summary.json)
- [开机复核汇总](/home/th1rt3en/Documents/Codex/2026-10-12/outputs/orin-structured-controller-resume/reboot-summary.json)
- [实际轨迹图](/home/th1rt3en/Documents/Codex/2026-10-12/outputs/orin-structured-controller-resume/orin-corner-trajectories.png)
