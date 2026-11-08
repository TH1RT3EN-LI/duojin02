# 目录与 Git 协作

日常构建、启动和硬件配置见 [项目 README](README.MD)，实现合同和实验边界见
[文档索引](docs/README.md)。当前开发分支是 `codex/local-cuda-v3`。

## 目录约定

| 目录 | 内容 | 版本管理 |
| --- | --- | --- |
| `src/` | 10 个 ROS 包的源码、配置、模型和包内测试 | 提交 |
| `patches/` | 固定版本的上游来源、校验值和有序补丁 | 提交 |
| `scripts/` | 环境、构建、验证与开发入口 | 提交 |
| `tests/` | Python 回归、集成探针及小型固定输入 | 提交 |
| `docs/` | 操作说明、算法合同、历史报告与选定证据 | 提交 |
| `docker/` | 容器入口与 CUDA 工具链声明 | 提交 |
| `build/`、`install/`、`log/`、`.cache/` | 可重新生成的构建、安装和缓存 | 忽略 |
| `output/` | 本机实验结果、日志、候选构建和大型证据 | 忽略 |
| `maps/`、`bags/`、`tmp/` | 本机地图、原始录包和临时文件 | 忽略；保留 `maps/.keep` |

`output/`、`bags/`、`tmp/` 同时排除于 Docker 构建上下文。忽略规则不会删除或备份这些文件；
需要复现实验时，另行保留数据、来源清单和校验值。新的小型回归输入放入
`tests/integration/fixtures/`，说明来源、用途和校验值。现有文档与证据路径保持稳定。

## 源码入口

| 包 | 职责 |
| --- | --- |
| `duojin02_base_driver` | 底盘串口通信和反馈 |
| `duojin02_description` | 底盘模型、尺寸和传感器安装变换 |
| `lslidar_driver` | 激光雷达驱动；包位于同名上游目录内 |
| `duojin02_scan_geometry` | 扫描时间、逐束几何和共享接口 |
| `duojin02_amcl_cuda` | AMCL 定位、观测后端和初始化 |
| `duojin02_mppi_controller` | MPPI 控制器及 CPU/CUDA 后端 |
| `duojin02_navigation` | 导航插件、几何验证和速度保护 |
| `duojin02_bringup` | 启动文件、配置、定位就绪和目标采集 |
| `duojin02_slam_tools` | 地图快照、离线回放和实验模型 |
| `duojin02_teleop` | 键盘、手柄和人工接管 |

补丁应用顺序以各依赖的 `patches/*/source.json` 为准。修改构建输入后，先重建 SLAM 后端，
再重建 CUDA 组件；不要手工改写来源校验记录。具体命令见 README。

## 验证与提交

项目维护者及本地新增代码署名统一为 `litianshun <litianshun.cn@gmail.com>`；
使用该身份创建 Git 提交，保留上游原始版权与许可证。署名范围和历史别名见
[AUTHORS.md](AUTHORS.md)。新克隆可在仓库内执行：

```bash
git config --local user.name litianshun
git config --local user.email litianshun.cn@gmail.com
```

```bash
git status --short --branch
git diff --check
./scripts/test.sh quick
```

根据改动运行 `navigation`、`slam`、`lidar` 或全部测试；CUDA 算法验证入口和范围见
[测试说明](tests/README.md)。按功能提交源码、对应配置和测试，文档应说明默认行为、实验开关与
实际验证范围。格式遵循 `.editorconfig`、`.clang-format`、`.cmake-format.json` 和
`pyproject.toml`；避免把整库格式化混入功能提交。

## 本机 Git 同步

本机保留两个远端：`origin` 指向原本地仓库，`github` 指向
[TH1RT3EN-LI/duojin02](https://github.com/TH1RT3EN-LI/duojin02)。
`codex/local-cuda-v3` 跟踪 `github/codex/local-cuda-v3`：

```bash
git remote -v
git fetch github --prune
git status --short --branch
git log --oneline HEAD..github/codex/local-cuda-v3
git log --oneline github/codex/local-cuda-v3..HEAD
git push
```

在工作树干净且确认远端只有可快进更新后，可用 `git pull --ff-only` 更新当前分支。
若本地与远端都有新提交，先检查差异再处理分叉。开发分支的同步不表示已合并到 `main`，
也不表示硬件或完整闭环已经验收。

从 GitHub 新克隆时，`origin` 会直接指向 GitHub；以 `git remote -v` 和当前分支的实际跟踪配置为准。
