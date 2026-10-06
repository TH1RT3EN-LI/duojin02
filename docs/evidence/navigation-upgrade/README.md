# 导航升级证据

正式结论见 [研究与验证报告](../../navigation-upgrade-review.md)。`results.json` / `results.csv` 是冻结配置的 52 组最终对照；完整逐帧轨迹、场景、规划、刺激、运行日志、源码与二进制哈希位于 `final-trials.tar.gz`。正式结果包含失败或未到达，不能只选成功轨迹。

- `baseline-nav2.yaml`、`baseline-commit.txt`：旧 Nav2 配置和硬件提交。
- `results-table.md`：逐场景结果；`trajectories.png`、`accuracy-and-clearance.png`：实际真值轨迹与汇总图。
- `experiment-metadata.json`：桌面机器与仿真假设；CPU/RSS 不是 Orin 原生结果。
- `artifact-sha256.json`：本目录数据文件的 SHA256。
- `experiment-index.json`：探索试验结果原文与来源哈希；没有验证编译来源的旧探索记录明确标记，不能取代最终矩阵。
- `superseded-goal-checker.tar.gz`：到达判定修正前的 26 组数据，包含最高 69 mm 在线落点误差。
- `rejected-mppi-cost-weight.tar.gz`：软障碍权重为 4 的 26 组探索结果，仍为 19/20，且一次在线 SLAM 定位 P95 达 0.645 m，因此不选用。最终权重为 2；最终矩阵保留一次在线薄障碍安全停滞。
- `startup-failures.tar.gz`：两次启动失败及可用日志，包含已丢失原日志的明确说明；未冒充导航失败或删去重试。
- `benchmark-harness.tar.gz`：生成场景、运行、编译来源审计、汇总的脚本快照；CAD 模型依赖现有 sim 仓库，模型源归档/输出哈希见仿真验收 JSON。
- `release-*`、`final-config-*`、`stopped-goal-*`：七包构建、48 项检查、伪串口实际零速帧、11 项保护故障检查、几何 CTest 与启动入口检查。
- `simulation-*.json`、`simulation-ogre2.log.gz`：完整传感器/运动/雷达近距过滤验收及实际 NVIDIA 渲染证据。
- `gui-online-health.json`：交付时在线 SLAM、地图、实时雷达和导航/保护节点的实际状态与参数。

常规使用 1 cm 地图；旧 3 cm 地图单独测试。baseline 与 upgraded 共享仿真传感器和 SLAM 后端，仅切换 Nav2 配置/插件/新增速度保护。Gazebo 雷达噪声随机流未固定，不能称完全相同的传感器样本。配对种子固定里程计扰动和初值。雷达掉线期间缺失的定位 TF 不以零填补，覆盖率单独报告。`near_contact_events` 采用事后统一的 2 mm 阈值，属于几何间距提示，不是接触力测量。

在相邻的 duojin02 / duojin02-sim 仓库中复现：

```bash
cd ../duojin02-sim
./simulation/navigation_benchmark.sh --seeds 1 2 3 --cases open corridor_0.50 dynamic_crossing initial_pose_error online_thin
# 另一组使用不同输出目录，保留第一组结果
DUOJIN_NAV_OUTPUT="$PWD/simulation/results/navigation-boundaries" \
  ./simulation/navigation_benchmark.sh --seeds 1 --cases corridor_0.56 corridor_0.48 corridor_0.46 corridor_0.44 corridor_0.40 precision_doorway_0.46 thin_obstacles s_bend sensor_dropout unsafe_command legacy_map_3cm
```

`unsafe_command` 的正确结果是停车，不能当作“没有到达目标”。窄门安全拒绝也与普通可行场景的到达率分开报告。最终配置选择包含探索后的修正，固定地图/几何先验小房间结果不能推断所有真实未知环境都达到同样精度或成功率。

原始文本日志以 `.log.gz` 无损压缩保存；派生结果 CSV 仅将换行规范为 LF，数值未改动。
