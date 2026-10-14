# GPU / CPU 统一入口

当前主线把已有 CUDA 几何查询合入同一个 `duojin02_navigation` 包。唯一运行选择为 `use_gpu`，默认从所选 `nav2.yaml` 的 `controller_server.ros__parameters.FollowPath.use_gpu` 读取，随附配置为 `true`。没有第二份 GPU 导航配置。

```bash
# 默认尝试 GPU 几何加速
ros2 launch duojin02_bringup navigation.launch.py map:=maps/site_a.yaml
# 同一套参数和优化器，使用 CPU
ros2 launch duojin02_bringup navigation.launch.py map:=maps/site_a.yaml use_gpu:=false
# 在线 SLAM 导航也使用同一开关
ros2 launch duojin02_bringup online_navigation.launch.py use_gpu:=false
```

固定图、在线导航和独立 `navigation_stack.launch.py` 都传递这个布尔值。显式参数只覆写这一项；自定义 YAML 可以设置自己的默认值。该值在控制器配置阶段确定，改模式时重新启动导航。

## 执行范围

两种模式均使用原来的 Nav2 MPPI 优化器、目标选择、候选排序、速度约束和制动保护。CUDA 只批量计算直接接近路径上的填充足迹碰撞谓词；小批次继续使用原 CPU 循环。设备浮点边界不明确的行由原 CPU 谓词复核。clearance 打分仍在 CPU 上，防止数值差异改变候选排序。

`use_gpu=false` 不创建 CUDA 执行器。`true` 在已编译 CUDA 且设备可用时允许加速；工作量达到 `gpu_minimum_work` 才提交 GPU。未编译 CUDA、无设备或初始化不可用时明确报告 CPU 回退。已开始的 GPU 调用失败或超过 `gpu_deadline_ms` 时停止本次运动请求，不能把运行期故障伪装成成功输出。

这两个性能参数与开关一起维护在 [nav2.yaml](../src/duojin02_bringup/config/nav2.yaml)。SLAM、AMCL、全局规划、EKF、雷达、最终制动检查与 VelocityGuard 沿用 CPU 实现；开关不宣称这些模块已经 CUDA 化。

独立 GPU 工作区原有的替代采样优化器、EDT / study 优化和调参预设保留在历史归档，不进入当前生产路径。其原生 Orin 性能证据不能用于断言本次几何加速的速度或实时性。

## 构建

Jetson 原生构建使用板端已安装的 CUDA Toolkit。构建会探测 CUDA 编译器；缺少编译器时仍可构建完整 CPU 版本，启动时报告回退。强制不编译 CUDA：

```bash
./scripts/build_hardware.sh -DDUOJIN_ENABLE_CUDA=OFF
```

Docker 默认准备固定版本的最小 CUDA 编译工具，并生成可使用 GPU 的安装；部署镜像不携带 CUDA 编译器。`testing` 镜像保留编译工具用于回归。只构建 CPU 镜像：

```bash
IMAGE=duojin02:hardware-humble-cpu ./scripts/build_image.sh --build-arg DUOJIN_ENABLE_CUDA=OFF
```

`DUOJIN_ENABLE_CUDA` 是构建能力选择，运行开关仍为 `use_gpu`。CUDA 版本及组件 SHA-256 的唯一记录在 [cuda-toolkit.json](../docker/cuda-toolkit.json)。准备工具只写指定目录，不安装或更改主机驱动。组件归档、平台和校验方式遵循 [NVIDIA CUDA 12.8.1 归档文档](https://docs.nvidia.com/cuda/archive/12.8.1/cuda-installation-guide-linux/index.html#tarball-and-zip-archive-deliverables)。

Docker 默认编译 Orin `87` 与本机验证使用的 `120` 目标；其他设备可通过构建参数 `CUDA_ARCHITECTURES` 指定。容器需要 NVIDIA Container Toolkit 提供计算设备和驱动库；`dev.sh` 默认在检测到 NVIDIA 设备时请求它们。仅开放设备不会强制控制器使用 CUDA。

## 仿真与归档

`sim` 是独立 Git 分支，`duojin02-sim` 是共用仓库的 worktree，保留其未提交模型、实验和数据。无需重复维护硬件 CPU / GPU 项目目录。当前 `sim` 已在另一个 worktree 中检出，不应在主工作区强制切换或删除 `.git/worktrees`；可用 `git worktree list` 查看。

本次[验证记录](evidence/gpu-cpu-merge/validation.json)：161 项 Python 检查、12 个 launch 入口、CPU CTest 17 通过 / 1 个 GPU 专项跳过、真实 RTX 5070 CUDA CTest 19/19 通过。14,472 个几何查询没有碰撞判定差异；CPU、GPU、无设备回退分别完成 198 个控制器 API 操作。导航指令链、雷达和 SLAM 集成通过，部署镜像的安装库也实际执行了 CUDA。SM87 已编译，本轮未连接 Orin 或实体底盘。

特定密集碰撞早退输入下，CPU 约 0.093 ms、GPU 约 0.895 ms；该结果保留在验证记录中，不宣称普遍提速。工作量阈值没有据此改动，端侧性能仍需在实际设备和路线重新测量。

本机归档位于 `/home/th1rt3en/dev/archive/duojin02/2026-10-14/`，其 `archive-index.md` 记录恢复位置、完整校验及旧停止容器的绑定说明。独立 GPU 项目整体保存，包括 `.git` 和未提交文件。四个历史缓存压缩归档，15,933 个成员逐项验证后清理源目录；只清理可重建 Python 缓存，保留活动 SLAM 源码缓存。主目录当次盘点约 757 MiB，减少约 1.24 GiB；同分区移动 GPU 目录本身不回收磁盘空间。

地图、录包、原始证据、`sim` 的 HEAD / 未提交列表和共享 Git 链接均通过保留检查。历史文件保持原字节，新的主线默认值不覆盖旧实验记录。
