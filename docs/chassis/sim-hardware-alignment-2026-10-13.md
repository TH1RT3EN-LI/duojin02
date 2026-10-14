> 当前配置与启动方式已收敛至 `duojin01_bringup/config/chassis.yaml`；本文原参数文件名属于历史记录，当前唯一来源见 [配置说明](configuration.md)。

# 仿真雷达位姿以当前真机模型为准

> 后续用户要求恢复原仿真高度：当前仿真扫描面为 **143.308 mm**，真机模型基准仍为 **130.233 mm**。显式覆盖同时应用于传感器、TF 和裁剪坐标变换；以下取消覆盖的 130.233 mm 验证属于先前实验。随后投影裁剪策略改为严格内部删除（边界保留带 0），不扩大到投影外。见[高度恢复记录](/home/th1rt3en/Documents/Codex/2026-10-13/outputs/sim-height-restoration-20261013/report.md)。

2026-10-13。用户要求以真机硬件的位置和高度调整仿真。只修改仿真兼容层及其测试；真机描述、CAD 实体和历史试验文件保持原样。

当前真机启动使用 `duojin01.xacro`，其中固定关节链为 `base_link → lidar_mount → laser`。展开后的完整变换为：

- XYZ：`[0.0924721442817, -8.97794961929e-06, 0.1302326658155] m`
- RPY：`[0, 0, 0] rad`

这是当前真机代码的模型基准；原测量记录中的“光学高度代理尚未实物标定”说明保留。

原仿真 profile 将扫描平面抬到 0.14330766582 m，额外增加 13.075 mm。现在取消这个覆盖，由独立 `hardware_geometry.py` 自动展开真实 Xacro、合成固定关节完整 XYZ/RPY，分别计算 SDF 与仿真 URDF 中各自父坐标系的相对位姿。不会把 CAD 雷达实体的旧原点误当成当前硬件安装基准，也不移动显示网格来配合扫描。

同一硬件变换应用于 GPU 传感器、SDF laser frame、RSP laser joint 和适配器。普通场景与 benchmark 共用生成逻辑；benchmark 显式使用当前 `/hardware` 描述。旧 profile 的高度若与硬件不一致会拒绝，不能成为第二个独立来源。

代码同步到两处仿真入口：

- [独立 sim 兼容层](/home/th1rt3en/dev/forge/duojin02-sim/simulation/src/duojin02_sim/duojin02_sim/scene.py)
- [GPU 项目的仿真 fixture](../../tests/performance/navigation_harness/sim_fixture/duojin02_sim/duojin02_sim/scene.py)

每次场景生成写入 `hardware-laser.json`，记录完整变换、三个 Xacro 的 SHA256、展开 URDF 的 SHA256及校准状态。未知引用、循环、运动关节链和非法姿态拒绝；实体几何和源文件不写入。

验证：

- Humble 容器中 25 项测试通过，包括非零 XYZ/RPY、不同父链、硬件位置、去除旧偏移、实体保留及不一致 profile 拒绝。
- 静态 Gazebo 与 RSP 中，4 帧 raw scan 均有 540 束；运行中的父 link 位姿、传感器相对位姿及 `laser` TF 合成为同一硬件坐标。两个受控进程正常退出。
- 新高度下四帧全部 raw 返回低于 0.30 m，最近约 26.77 mm。这暴露了该平面中的 CAD 近距遮挡，不能通过再次抬高扫描平面规避。没有把导航包络当成真实光学窗口，也没有修改 range 下限。

初次运行检查错误地把 Gazebo 消息的 `worldPose` 当作绝对位姿；实际该消息由 `Sensor::Pose()` 填充，在当前运行中是父系相对位姿。保留失败记录后，增加运行 scene service 的父 link 位姿，再合成检查，通过。[Gazebo Lidar 源码](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/src/Lidar.cc)、[Sensor 源码](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/src/Sensor.cc)。

[测试与静态运行证据](/home/th1rt3en/Documents/Codex/2026-10-13/outputs/sim-hardware-geometry-20261013/validation.json)；[四帧运行结果](/home/th1rt3en/Documents/Codex/2026-10-13/outputs/sim-hardware-geometry-20261013/runtime/result.json)。

本轮验证位姿一致性，没有重跑完整导航矩阵或实机测距。旧 143.308 mm 输入下的定位支持、立柱回波、导航结果及过滤回放仍属于旧版本证据，不能直接移用于这次变更。下一项问题是核对真实光学窗口/壳体模型，以及硬件光学高度标定。

## 后续真机接收对照

2026-10-13 随后取得 15 秒真实串口数据，149 圈完整扫描；保留全部双回波，不做量程或底盘裁剪。最短非零原始距离为 113 mm，并有大量外部远处回波。新仿真的全向约 27 mm 遮挡与实际传感器输出不匹配，需核对自身壳体/光学窗口、周边底盘表面与扫描平面，不能靠裁掉投影外电线/网线或再次抬高雷达消除差异。2D 扫描不独立证明光学高度，设备近距处理也需要区分。

本次投影外且不足 0.3 m 的 323 次回波已全部保留；仍未确认具体线缆身份，旧 ROS 量程门也没有因此自动修复。[原始数据与对照报告](/home/th1rt3en/Documents/Codex/2026-10-13/outputs/real-lidar-cables-20261013/report.md)。

继续将真机数据与当前硬件固定链合成后的 CAD 截面叠加，名义投影内、原始距离不足 0.3 m 的 11,051 次候选回波到任意截面的最近距离中位数 3.455 mm、90 分位 9.572 mm。用户指出明显错配后，审查发现其中 55.4% 实际选中 shell，而非锁定对应金属部件；该近邻指标掩盖了错配，不能认定“主要结构已匹配”或外参正确。六簇具有共同约 2–3° 角偏，用户当前未确认三角零度朝正车头，原 yaw=0 仍是未验证假设。

约 27 mm 全向仿真交点已追到 native `79.ply` 的“1001-05 下层装饰外壳”：雷达轴周围直径约 54 mm 的圆形孔边竖壁 Z=124.808–131.808 mm，代理面 130.233 mm 正好切进它。N10 自壳体已被仿真 visibility mask 排除，不能把该现象归因于雷达自己的显示网格。需实测真实光学平面并核对装配/CAD 孔边；[截面、源零件与完整对应报告](/home/th1rt3en/Documents/Codex/2026-10-13/outputs/real-lidar-cad-match-20261013/report.md)。

固定六个候选构件、三个估计/三个留出的有限角敏感性中，+3° 假设将留出点到固定外环的指标从 7.145 mm 降至 5.126 mm；它只解释部分角偏，仍有回波拖尾和光学可见性差异。没有写入任何真机外参或量程补偿，投影外线缆完整保留。[撤回原匹配解释与重新定位](</home/th1rt3en/Documents/Codex/2026-10-13/outputs/real-lidar-cad-mismatch-20261013/report.md>)。
