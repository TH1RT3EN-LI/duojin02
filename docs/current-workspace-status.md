# 本机 CUDA 工作区状态

更新时间：2026-11-08。本文记录本机工作区状态，不表示发布或硬件验收。

## 代码身份

- 原 `duojin02` 的 `main` 提交为 `89c2f3acae1d05e804b31f720fa52757b5ef190d`。
- 当前工作目录 `/home/th1rt3en/dev/forge/duojin02-cuda-local` 的分支为 `codex/local-cuda-v3`。2026-11-08 整理前的 HEAD 为 `721e99ffa735bb93452fa113389604e3f3079b89`；此前未提交的源码、配置、测试和文档现已按主题纳入开发分支。最新提交身份以 `git rev-parse HEAD` 为准。
- `codex/workspace-checkpoint-20261106` 是对当时工作树中 Git 已跟踪及未忽略的源码、配置、测试和文档的工作快照，包含实验性修改。它不是稳定发布，也没有合并到 `main`。原目录及其工作树修改不因建立快照而清空。
- 用户要求只保留一个目录：此前创建的检查点工作树已移除；本仓库仅保留 `/home/th1rt3en/dev/forge/duojin02-cuda-local` 一个检出目录。检查点分支仍在同一个 Git 仓库中，不占用第二个工作目录；`output/`、`build/`、`install/`、`log/`、本机地图和数据集不属于该 Git 快照。
- 当前开发分支同步到 `github/codex/local-cuda-v3`。`github` 指向 GitHub，原有 `origin` 本地远端保留；具体约定见 [目录与 Git 协作](../CONTRIBUTING.md)。本次未合并或更新 `main`。

## 代码用途与默认状态

| 内容 | 位置 | 状态 |
| --- | --- | --- |
| CUDA AMCL、MPPI 与导航几何 | `src/duojin02_amcl_cuda/`、`src/duojin02_mppi_controller/`、`src/duojin02_navigation/` | RTX 5070 本机组件对照与原生测试已有证据；不是 Jetson 或实车验收。 |
| 建图和扫描时间改动 | `patches/slam_toolbox/`、`src/duojin02_scan_geometry/`、`src/duojin02_slam_tools/` | 是开发分支中未统一整机验收的改动；有序补丁清单在 `patches/slam_toolbox/source.json`。当前配置中的扫描运动补偿默认关闭。 |
| 全局定位获取 | `src/duojin02_amcl_cuda/include/nav2_amcl/global_acquisition.hpp` | 实验功能；`global_acquisition_enabled` 在项目配置中默认 false，完整泛化和真实到点尚未验收。 |
| 连续几何、子地图和共享证据 | `src/duojin02_slam_tools/duojin02_slam_tools/` | 文档中标为影子/实验路径，不应假定已改善生产定位精度。 |
| 到点记录与诊断 | `src/duojin02_bringup/`、`src/duojin02_navigation/` | 新增被动目标和控制状态记录；2 cm/1°、95% 联合到达率仍未验收。 |
| V33 栅格增量及 20 Hz 缺帧修复 | 旧工作区 `output/new-architecture-implementation-20261027T032708Z-A01/` 及其独立候选 | 用户已要求暂停此压测及针对它的修复；候选不属于上述快照的已验收生产代码。 |

默认导航配置 `src/duojin02_bringup/config/nav2.yaml` 中 AMCL 的 `observation_backend` 和 MPPI 的 `optimizer_backend` 均为 `auto`。实际执行设备取决于构建、设备和运行时分支，须查加载库与计数，不能只看参数。

## 已有证据的位置

实验结果保留在原工作区，不随 Git 快照复制或提交：

- 2026-11-05 新旧版本组件对照、实际源码清单、构建/测试日志：`/home/th1rt3en/dev/forge/duojin02-cuda-local/output/paired-gpu-20261105/report.md`。其中记录 40 项原生测试、10 Hz 上限合成 AMCL 和固定输入控制器对照；结论范围和行为差异以报告为准。
- 约 10 Hz LILocBench 建图回放：`/home/th1rt3en/dev/forge/duojin02-cuda-local/output/lilocbench-10hz-20261105/README.md`。这是历史后端的离线建图，不是本快照完整建图链验证。
- 20 Hz 工作暂停记录：`/home/th1rt3en/dev/forge/duojin02-cuda-local/output/new-architecture-implementation-20261027T032708Z-A01/report.md`。
- 历史 CUDA v3 组件证据：[CUDA v3 说明](cuda-local-v3.md)。其 2026-10-16 测试不能自动证明其后的所有未提交修改。

部分旧实验目录是指向外部磁盘的符号链接；检查原路径及磁盘挂载状态后再使用。快照不备份这些工件，也不改变它们。

## 后续使用

2026-11-06 已在 `duojin02:hardware-humble` 容器、本机 x86_64 / RTX 5070 上重建当前 SLAM 补丁后端及六个 CUDA 相关组件。`install/slam_backend` 指向 `install/managed-cuda-20261106-clean/slam_backend`；接入粗略起点局部初始化后，`install/cuda-local-v3` 指向 `install/managed-local-start-20261106/cuda-local-v3`。更早目录保留为 `install/slam_backend-before-20261106`、`install/cuda-local-v3-before-20261106`；上一套 CUDA 安装留在 `install/managed-cuda-20261106-clean/cuda-local-v3`。旧产物不应混称当前源码。构建与检查记录在 `output/engineering-fixes-20261106/` 和 `output/local-start-20261106/`。

`source scripts/source_cuda_local.sh` 现会核对组件源码快照、SLAM 源版本/补丁、扫描几何源码与架构；不匹配时阻止通过该入口加载。常规 `source.sh` 或手动 source 安装目录可绕过此检查，系统库与实际设备仍需独立确认。重建顺序及记录行为见项目 README。算法问题与证据范围见 [当前算法问题清单](algorithm-issues-20261106.md)。

切换后，独立构建目录中的 MPPI 3 项、AMCL 14 项、导航 23 项原生测试均通过，包含 CUDA 专项；独立 ROS 域的单条目标消息也写入了默认 JSONL，进程映射核对了新导航及 SLAM 动态库。另有 12 项来源与记录相关轻量测试通过。上述是本机软件验证，不等同于真实雷达、Jetson 或完整闭环验收。

当前局部初始化功能、按原节奏回放的结果及一次尚未定位的早停异常，见 [人工粗略起点与自动定位修正](local-initialization.md)。

2026-11-08 整理时，当前 SLAM 安装的来源校验通过；`install/cuda-local-v3` 的源码清单与现有 Foxglove 配置、仲裁配置等输入不一致，CUDA 加载入口会要求重建。本轮未改写已有安装或构建产物，也未重新执行 GPU 算法验收。运行当前代码前按 README 的顺序重建后端与 CUDA 组件。整理内容、进程退出竞态修正及本轮测试记录见 [2026-11-08 维护记录](project-maintenance-20261108.md)。

`scripts/validate_cuda_algorithms.py` 会在其所在工作区使用 `DUOJIN_CLEAN_BUILD=ON` 重建对应构建目录；不要在需要保留旧构建产物的目录里直接运行它。

后续测试应以实际 10 Hz 雷达或保持原始节奏的录包为基准；约 10 Hz 的 LILocBench 抽帧只是公开数据对照。当前版尚无实际雷达、Jetson 和完整建图→定位→导航的验收结果。
