# 导航验证与性能复现

本页维护当前导航检查的执行方法。机制见 [导航修复](navigation-repairs.md)，结果与版本见 [优化状态](navigation-optimization.md)；旧报告的通过数只属于其冻结版本。

## 构建与检查的区别

`scripts/build_hardware.sh` 默认传入 `BUILD_TESTING=OFF`。已有生产构建目录可能没有 CTest，`validate_hardware.sh` 的 Python 检查和 launch 参数解析也不替代 C++ / ROS 算法回归。需要工作区测试构建时，可在隔离环境中运行 `./scripts/build_hardware.sh -DBUILD_TESTING=ON`；脚本将后置参数传给 CMake，覆盖默认 OFF。不要把“未发现测试”记作通过。

仅检查导航包不需要重新构建 SLAM 后端：可在具备 Humble、Nav2、KDL 和测试依赖的环境中使用独立 CMake 构建。源码目录与输出目录分开，不能把独立包安装的 `share/duojin01_navigation/local_setup.bash` 当成完整 colcon 工作区的 `setup.bash`。

## 隔离的本地算法回归

下例使用本次已验证、预先存在的本机测试镜像 `duojin02:sim-humble-harmonic`，不是项目默认部署镜像，也不自动构建或拉取它。所有测试在无外部网络、无设备映射的容器中运行；源码只读，报告写入独立临时目录。原生真机运行仍不需要 Docker。

在仓库根目录执行：

```bash
TEST_IMAGE=duojin02:sim-humble-harmonic
validation_output=$(mktemp -d /tmp/duojin-nav-check.XXXXXX)
docker run --rm --init --network none --cpus 2 --memory 3g --pids-limit 256 \
  -v "$PWD":/ws:ro -v "$validation_output":/check -w /ws \
  --entrypoint bash "$TEST_IMAGE" -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    cmake -S src/duojin01_navigation -B /check/build \
      -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
    cmake --build /check/build -j2
    ctest --test-dir /check/build --output-on-failure
  '
```

检查 CTest 实际列出的项目和退出码。详细日志在输出目录的 `build/Testing/Temporary/LastTest.log`；缓存 ROS 探针另写 `build/guard-association-probe.json` 和对应日志。测试使用当前源码构建的可执行文件，不能只看旧安装路径中有无文件。

| 检查 | 覆盖范围 |
| --- | --- |
| 几何、跟踪、guard 查询与关联缓存 | 填充足迹、完整制动、保守裁剪、关联失效与独立参考一致性 |
| 点索引 | 与冻结哈希实现及完整点集对照：有序访问、计数、半径、负坐标 / 格边界、连续存储上限、稀疏回退和隐藏离群点 |
| 路径目标选择 | 与安装版 Humble `doTransform` 的完整路径计算对照，检查返回值、位姿精确位表示和头信息 |
| 扫描有效性、限速、停稳 | 非法输入、所有输出分支的速度约束和持续停稳时间 |
| 实际 lattice / MPPI 公共 API | 搜索、地图变化、缓存、软代价、限速与配置；不代替闭环路径效果 |
| 注册的缓存 ROS 探针 | 真实 guard、重映射合成话题，缓存失效不能刷新扫描心跳 |

额外扫描故障与 TF 探针见 `tests/integration/scan_fault_probe.py` 和 `guard_tf_probe.py`，CLI 使用 `--guard-binary`、`--params-file`、`--json-out`、`--log-dir`。它们也应在上述隔离环境中执行，使用本轮构建产物和当前 YAML。CTest 内缓存探针与额外运行的同名探针不能重复计入不同场景数；通过断言和正常进程退出分别核对。

## 组件性能

两个组件基准均是按需构建目标，不进入 CTest 的速度判据。前面的 `docker run --rm` 完成后容器已退出，不能在宿主直接运行 `/check/...`。以下是仓库根目录的一条完整命令：使用独立输出目录重新配置 / 构建，并在同一隔离容器内串行 ABBA；每次语料使用不同路径。

```bash
TEST_IMAGE=duojin02:sim-humble-harmonic
component_output=$(mktemp -d /tmp/duojin-nav-component.XXXXXX)
docker run --rm --init --network none --cpus 2 --memory 2g --pids-limit 128 \
  -v "$PWD":/ws:ro -v "$component_output":/check -w /ws \
  --entrypoint bash "$TEST_IMAGE" -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    cmake -S src/duojin01_navigation -B /check/build \
      -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
    cmake --build /check/build --target path_target_benchmark spatial_index_benchmark -j2
    slot=0
    for arm in baseline candidate candidate baseline; do
      slot=$((slot+1))
      /check/build/path_target_benchmark "$arm" 1000 reuse "/check/path-$slot-$arm.bin" \
        > "/check/path-$slot-$arm.json"
    done
    slot=0
    for arm in baseline candidate candidate baseline; do
      slot=$((slot+1))
      /check/build/spatial_index_benchmark "$arm" compact_queries "/check/index-$slot-$arm.bin" \
        > "/check/index-$slot-$arm.json"
    done
    sha256sum /check/*.bin > /check/corpus.sha256
  '
```

该例只运行 1,000 点路径复用和紧凑点集查询两个案例，展示执行环境，不替代完整冻结对照。同一案例四次语料的 SHA 见 `corpus.sha256`；输出指纹及工作量见各 JSON。确认一致后才比较计时；程序 / 容器非零退出时保留输出，不重命名为通过。

`path_target_benchmark` 的参数依次为比较臂、点数、`reuse` 或 `replace-every-cycle`、输入语料输出路径。`reuse` 仍在每周期使用变化的 TF，并完整扫描路径；不是复用旧目标。输出 JSON 包含线程 CPU / 墙钟均值、P95、最大值、周期数与输出指纹。源码、编译参数、二进制、KDL / tf2 版本和语料哈希一起冻结；串行 ABBA 运行并比较输入字节与输出指纹，不能只保留最快一轮。

此基准仅覆盖路径目标选择；替换模式另包含子类输入预处理。原生 MPPI `setPlan`、TF 获取、地图快照、碰撞 / 制动、ROS 和导航均不在计时内。首次预处理有成本，完整最近点搜索仍为 O(N)，不推导整机提速。关联缓存的另一基准见 `tests/performance/prior_association_benchmark.cpp`；其测量范围另列于 [实时性记录](navigation-latency-optimization.md)。

`spatial_index_benchmark` 的参数依次为比较臂、模式、输入语料输出路径。每个计时批次包含 256 组近邻 / 关联查询及有序访问结果写入；`compact_queries` 不计重建，另用 `compact_rebuild_query` 和 `sparse_rebuild_query` 分别检查逐批重建与超限回退。点集和查询生成、指纹计算在计时外。三个模式都按串行 ABBA 比较语料、输出指纹与访问计数，保留稀疏场景的负收益；不把紧凑点集结果推广到全部地图。关联构造对照必须在两臂都启用已有缓存，单独冻结原哈希头文件，才能隔离本轮索引变化。

## 目标设备验收

本地 API / 合成基准、ARM64 编译和 Jetson 原生闭环测量是不同层级。板端比较需冻结完整基线 / 候选源码、安装产物及有效参数，使用相同地图、路线、传感器、显示负载和资源状态。记录首次全局规划、控制 / guard 有效周期、末端命令间隔、各进程 CPU / RSS、导航结果与独立到达指标；完整保留启动、导航和清理失败。

RViz 无查看器、一个查看器、多个查看器分别比较；控制 / 显示域隔离仍是已有实验夹具，不是默认部署功能。内部电源轨数据、通信掉线与实物供电稳定性分别判断。部署和验收进度以 [优化状态](navigation-optimization.md) 为准；本页维护方法，不证明已经完成板端长程或供电验证。
