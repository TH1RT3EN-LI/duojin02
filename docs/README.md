# 文档索引与适用范围

这组文档服务于 `duojin02` 真机工作区，6 个自研 ROS 包统一使用 `duojin02_*`，第三方雷达包保持 `lslidar_driver`。操作说明、研究摘要、冻结历史分别承担不同职责；阅读旧报告时不要把当时的参数、设备可达性或 GUI 状态当作现在的运行状态。

## 日常入口与维护位置

| 问题 | 首要文档 | 维护内容 |
| --- | --- | --- |
| 如何原生构建、识别串口和选择启动入口 | [主 README](../README.MD) | 操作顺序、各入口参数范围；Docker 为可选环境 |
| 如何从旧包名升级，外部调用需要改什么 | [名称迁移](package-renaming.md) | 6 包映射、全新构建、环境、设备和 Docker 更新 |
| 如何选择 GPU / CPU，怎样保留同一优化器 | [GPU / CPU 统一入口](gpu-cpu.md) | `use_gpu`、可选 CUDA 构建、仿真 worktree 与目录归档 |
| 如何在观看机看雷达、模型和导航话题 | [RViz 观察](remote-rviz.md) | 显示域、坐标系、模板差异和 QoS；不重复启动机器人 |
| 包有哪些，谁发布速度，节点怎样连接 | [开发接口](../README_DEVELOPMENT.md) | 7 包职责、当前话题和指令所有权 |
| 遥控如何接管、导航怎样恢复、等待预算归谁 | [运行机制](runtime-mechanisms.md) | 按住接管、证据恢复、就绪候选与数据 I/O 边界 |
| 固定图怎样就绪、诊断、恢复 | [定位质量](localization-health.md) | 报警、AMCL 原生恢复、地图和扫描语义 |
| 哪些故障停车，哪些指标只报警 | [停车保护](safety.md) | 必要保护、唯一配置与删除边界 |
| 录包、保存、离线回放、续建 | [SLAM 数据工作流](slam-workflow.md) | 录包、快照、回放、续建；研究另列 |
| 修复机制与回归前提 | [导航修复](navigation-repairs.md) | 规划、控制、扫描与地图契约 |
| 如何启用 CTest、隔离检查和复现分项计时 | [导航验证](navigation-validation.md) | 构建开关、功能覆盖、组件与板端验收边界 |
| 模型从哪里来，哪些尺寸尚未标定 | [底盘测量](chassis/README.md) | CAD 来源、投影、TF、实测边界 |
| 雷达来源与裁剪范围 | [UPSTREAM](../src/lslidar_driver/UPSTREAM.md)、[上游评审](lidar-upstream-review.md) | 固定协议与项目补丁 |
| 现在有哪些优化证据、哪些未完成 | [优化状态](navigation-optimization.md) | 有限结果和候选状态 |
| 结构清理、参数入口和算法保留边界 | [维护记录](maintenance-cleanup.md) | 2026-10-13 清理范围与验证 |
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

2026-10-10 的原修复工作树已在 `f90a30c` 冻结，2026-10-13 的结构清理以该版本为算法基线，见[维护记录](maintenance-cleanup.md)。2026-10-14 的当前名称统一见[名称迁移](package-renaming.md)；冻结证据中的旧包名与当时真实镜像、外部路径仍按原版本解释。同日的运行机制精简见 [当前契约](runtime-mechanisms.md)；原导航、定位与 SLAM 算法结果仍按冻结版本解释。main 的源码状态与设备部署需分别核对。

## 结论口径

Nav2 Action 成功、独立真值下的严格到达、碰撞评分、采集完整、进程正常退出、设备性能分别报告。短程成功不覆盖长程，FollowPath 回访不覆盖全局搜索，桌面回放或 QEMU 功能不覆盖 Jetson 原生实时性。碰撞结论只覆盖报告声明的几何与插值假设；CAD 和仿真结果不证明实物安装、供电或制动。

本轮删减及矛盾修正见 [文档检查记录](documentation-review.md)。
