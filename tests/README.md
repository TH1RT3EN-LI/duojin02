# 测试说明

常规入口是仓库根目录的 `./scripts/test.sh`。本机没有 ROS 2 Humble 时，该入口会构建并使用
Docker 测试环境；已有 Humble 时直接运行，也可显式传 `--docker`。

## 常规检查

| 命令 | 范围 |
| --- | --- |
| `./scripts/test.sh quick` | Python 回归和启动参数解析 |
| `./scripts/test.sh navigation` | 导航原生检查及命令链集成探针 |
| `./scripts/test.sh slam` | 补丁后端、地图保存、录制与合成回放 |
| `./scripts/test.sh lidar` | 雷达驱动原生检查和伪串口探针 |
| `./scripts/test.sh` | 上述全部 |

结果默认写入 `build/tests/`；`TEST_ROOT`、`BUILD_JOBS`、`BUILD_WORKERS` 可调整产物目录和
构建并发。测试入口会检查所需原生测试是否实际被发现。SLAM 组使用来源校验通过的补丁源码；
已有源码可通过 `PREPARED_SLAM_SOURCE` 指定。

直接在未准备的宿主机执行 `python3 -m pytest` 可能缺少 ROS、NumPy 或源码包导入路径。
旧 Docker 镜像中的 SLAM 服务也可能与当前补丁不一致；应通过项目构建入口准备当前环境。
不要把环境导入失败当成测试通过。

## 目录入口

- `test_*.py`：环境脚本、配置、定位就绪、地图快照和进程生命周期等回归。
- `integration/`：ROS / C++ 集成探针；其中 `karto_*`、`map_graph/` 检查测量和图更新合同。
- `integration/fixtures/`：小型归档与实际扫描输入，旁边的 JSON 记录来源和校验值。
- `realtime_goal/`：AMCL、建图、控制、共享证据和回放的实验检查；部分入口需要独立构建、
  已冻结的地图、录包及本机历史产物，先读 [实施与运行合同](../docs/realtime-goal-implementation.md)。
- `../src/*/test/`：各包的 C++、CUDA 或 Python 测试；完整列表以对应 `CMakeLists.txt` 为准。

## CUDA 与专项回放

底盘反馈队列、错帧恢复和姿态更新有效性的 C++ 检查已纳入 `navigation` 组。
需要比较旧版与候选版时，在 Humble 环境构建两个独立安装后运行：

```bash
python3 tests/integration/base_driver_hardening_probe.py \
  --baseline /path/to/baseline/lib/duojin02_base_driver/duojin02_base_driver_node \
  --candidate /path/to/candidate/lib/duojin02_base_driver/duojin02_base_driver_node \
  --output-dir output/base-driver-comparison
```

该探针在隔离 ROS 域中启动真实节点，通过 PTY 输入协议字节；测试专用 `/clock` 用于比较
相同接收时刻下的数值，不用于推断串口突发帧的真实采样时间。它检查正常输出相等、突发顺序、
错帧恢复、溢出计数、时间跳变和非法参数，并保存 JSON 与节点日志。

```bash
python3 scripts/validate_cuda_algorithms.py --architecture 120 --jobs 2 --frames 60
```

这是 RTX 5070 / x86_64 的现有本机入口，会重建所选目录并生成新的 `output/cuda-local-v3/`
记录。`120` 是该设备使用的架构参数；其他设备需选择匹配架构。不要在需保留旧构建的目录中
直接使用会清理构建的验证入口。

原生组件和合成回放只能证明相应软件合同；真实雷达、Jetson 和完整建图→定位→导航闭环
的验收状态见 [工作区状态](../docs/current-workspace-status.md)。
