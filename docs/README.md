# 文档索引

先读 [工作区状态](current-workspace-status.md) 和 [当前算法问题](algorithm-issues-20261106.md)，
再按任务查阅以下文档。文档中的历史测试只覆盖对应源码和环境；本机实验结果保存在忽略的
`output/` 中，Git 同步不会备份它们。

## 操作与维护

- [项目操作入口](../README.MD)：安装、构建、建图、定位、导航与遥控。
- [目录与 Git 协作](../CONTRIBUTING.md)：源码职责、产物边界和同步约定。
- [测试说明](../tests/README.md)：常规检查、原生组件与实验探针。
- [2026-11-08 维护记录](project-maintenance-20261108.md)：整理、验证与同步范围。
- [人工粗略起点与自动定位修正](local-initialization.md)：固定地图初始化。
- [雷达水平角记录](lidar-yaw-calibration-20261107.md)：暂定标定值及适用范围。
- [Foxglove 配置](../src/duojin02_bringup/config/foxglove/README.md)：建图和导航布局。

## CUDA 实现与漂移修正

- [CUDA v3](cuda-local-v3.md)：MPPI / AMCL 后端、参数与历史验证。
- [CUDA v2](cuda-local-v2.md)：几何加速的历史实现。
- [漂移修正总览](drift-remediation-gpu.md)。
- [扫描时间与逐束几何](drift-remediation-timing.md)。
- [AMCL 实现合同](drift-remediation-amcl.md)。
- [地图导出与图诊断](drift-remediation-map-graph.md)。

## 建图与测量合同

- [测量与共享证据模型](measurement-evidence-model.md)。
- [图更新、联合提交与快照](graph-update-contract.md)。
- [SLAM 补丁清单](../patches/slam_toolbox/source.json)。
- [地图服务补丁清单](../patches/nav2_map_server/source.json)。

## 任意地图目标与实验路径

- [实施与运行合同](realtime-goal-implementation.md)。
- [AMCL 分阶段状态与证据](realtime-goal-amcl.md)。
- [建图实现范围](realtime-goal-mapping.md)。
- [目标、控制消费与任务评价](realtime-goal-navigation.md)。
- [共享误差与证据采集](realtime-goal-statistics.md)。

影子模型和实验获取路径的存在不代表已改善生产定位或完成厘米级到点验收。

## 历史证据和报告

- `evidence/`：已纳入版本管理的历史验证记录。
- `chassis/`：底盘尺寸、测量数据和预览。
- [CUDA 报告源文件](cuda-report-source/cuda_report/README.md)：历史报告的源码与校验记录。
- [AMCL 许可证](licenses/AMCL-LGPL-2.1.txt)。
