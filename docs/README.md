# 文档索引与适用范围

这组文档服务于 `duojin02` 真机工作区，ROS 名称保留 `duojin01_*`。操作说明、研究摘要、冻结历史分别承担不同职责；阅读旧报告时不要把当时的参数、设备可达性或 GUI 状态当作现在的运行状态。

## 日常入口与维护位置

| 问题 | 首要文档 | 维护内容 |
| --- | --- | --- |
| 如何原生构建、识别串口和选择启动入口 | [主 README](../README.MD) | 操作顺序、各入口参数范围；Docker 为可选环境 |
| 如何在观看机看雷达、模型和导航话题 | [RViz 观察](remote-rviz.md) | 显示域、坐标系、模板差异和 QoS；不重复启动机器人 |
| 包有哪些，谁发布速度，节点怎样连接 | [开发接口](../README_DEVELOPMENT.md) | 7 包职责、当前话题和指令所有权 |
| 固定图怎样就绪、闭锁、恢复 | [定位健康](localization-health.md) | 状态机、证据窗口、地图和扫描语义 |
| 录包、保存、离线回放、续建 | [SLAM 数据工作流](slam-workflow.md) | 录包、快照、回放、续建；研究另列 |
| 修复机制与回归前提 | [导航修复](navigation-repairs.md) | 规划、控制、扫描与地图契约 |
| 如何启用 CTest、隔离检查和复现分项计时 | [导航验证](navigation-validation.md) | 构建开关、功能覆盖、组件与板端验收边界 |
| 模型从哪里来，哪些尺寸尚未标定 | [底盘测量](chassis/README.md) | CAD 来源、投影、TF、实测边界 |
| 雷达来源与裁剪范围 | [UPSTREAM](../src/lslidar_driver/UPSTREAM.md)、[上游评审](lidar-upstream-review.md) | 固定协议与项目补丁 |
| 现在有哪些优化证据、哪些未完成 | [优化状态](navigation-optimization.md) | 有限结果和候选状态 |
| 实时性候选与本地继续优化做了什么 | [实时性记录](navigation-latency-optimization.md) | 各阶段机制、功能 / 分项测量及待完成板端对照 |

当前参数以所检出的配置为维护位置，见开发接口中的配置表；历史报告的数值不是新的默认值。文档发生冲突时先对齐所用源码、安装产物和有效参数，再修正文档；不按旧性能结果绕过当前安全契约。首轮文档检查的静态范围见 [检查记录](documentation-review.md)；随后算法性能工作单独记录在实时性页，不回填为历史试验结果。引用的 Rolling / Jazzy 页面用于解释概念，不保证 Humble 提供同名参数或 API；具体功能需核对项目固定版本，不能从较新文档直接抄入运行配置。

## 冻结历史：解释结果，不指导当前部署

| 记录 | 原适用阶段 |
| --- | --- |
| [硬件减法](hardware-simplification.md) | 2026-10-06、`b3f68de` 基础上的 10→6 包；随后加入导航插件包成为 7 包 |
| [SLAM 升级记录](slam-upgrade-review.md) | 后端修复 / 工具链来源与 2026-10-05 冻结验证；操作步骤已独立 |
| [第一次导航升级](navigation-upgrade-review.md) | 2026-10-06 升级矩阵，含旧速度与容差 |
| [速度与精度 v4](navigation-performance-review.md) | `7bc9e10`，70/72 严格到达 |
| [窄门与停靠 v7](navigation-narrow-review.md) | `377f3ee` / `95a1455` / `9f179c7`，78/78 严格到达 |
| [单终点长程](navigation-continuous-review.md) | `c621f9c` 的本机仿真；旧启发式缓存与 Collision Monitor 链路已被后续修复替代 |
| [优化历史正文](navigation-optimization-history.md) | 10 月 7–8 日研究全过程，保留失败和未晋级候选 |
| [定位健康修正历史](localization-health-history.md) | 旧缺测规则的修正背景与原会话证据说明，操作已独立 |
| [首次升级证据索引](evidence/navigation-upgrade/README.md) | 原试验表和原始证据的说明，不改写成当前版本结果 |

截至 2026-10-10，修复在 `codex/navigation-repairs-20261007` 的未提交工作树，基于 main `6a20e6d`；这一状态不等于已合并或部署。以后读本文时应重新查看 Git 与部署记录，不能沿用这个截点。

## 结论口径

Nav2 Action 成功、独立真值下的严格到达、碰撞评分、采集完整、进程正常退出、设备性能分别报告。短程成功不覆盖长程，FollowPath 回访不覆盖全局搜索，桌面回放或 QEMU 功能不覆盖 Jetson 原生实时性。碰撞结论只覆盖报告声明的几何与插值假设；CAD 和仿真结果不证明实物安装、供电或制动。

本轮删减及矛盾修正见 [文档检查记录](documentation-review.md)。
