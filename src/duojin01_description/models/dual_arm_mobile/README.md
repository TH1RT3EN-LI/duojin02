# 双臂移动机器人模型

从 `PackAndGo 0000 双臂移动机器人-2026-09-21.zip` 的原生 SolidWorks 缓存三角网格和装配变换转换。此处保存提高精度后的 `high` 版本，适配 Gazebo Harmonic / Gazebo Sim 8。模型已布置到 `duojin02` 的 `sim` 分支，作为后续仿真接入的独立资产。

![模型预览](preview.png)

## 文件

| 文件 | 用途 |
| --- | --- |
| `model.sdf` / `model.config` | Gazebo 模型、关节、简化碰撞体和现有模型插件配置 |
| `meshes/*.dae` | 45 个按刚体与材质合并的显示网格 |
| [dual_arm_mobile.urdf](../../urdf/dual_arm_mobile.urdf) | 同步的 URDF，使用 `package://duojin01_description/` 引用网格 |
| `preview.glb` / `preview.png` | 带颜色的静态整机预览与实际网格渲染图 |
| `conversion_report.json` | 保留/移除部件、网格统计、装配信息及转换质量记录 |
| `asset_manifest.json` | 原始压缩包摘要、转换来源与每个导入资产的 SHA-256 |

SDF / URDF 使用米、弧度及 Z 向上的机器人坐标系；GLB 按 glTF 约定使用 Y 向上。GLB 是静态外观预览，运动关节以 SDF / URDF 为准。资产导入保持源文件一致，未再次减面。

## 精度与结构

| 项目 | 当前版本 |
| --- | ---: |
| 原装配展开三角面 | 1,918,229 |
| 去除内部细节后、减面前 | 939,467 |
| 最终显示三角面 | **516,650** |
| 物理刚体 | 25 |
| 整机外形约 | 0.540 × 0.407 × 1.040 m |

保留外壳轮廓、麦轮滚子、双臂、安装板、夹爪和传感器外形，去除紧固件、舵机内部齿轮、PCB 等细节。双臂每侧五轴，另有升降、两只双指夹爪和四个轮轴。配色按部件类别重建。显示网格与碰撞体分开，碰撞体采用盒体和圆柱，以降低实时仿真开销。

质量、惯量、关节轴/限位、力矩与控制参数来自几何推定或先前仿真整定，尚需硬件参数校准。未恢复 CAD 配合约束；夹爪传动简化为两指平移，自碰撞关闭。轮胎接触采用整体圆柱，底盘的 `VelocityControl` 与 `MecanumDrive` 配置用于运动演示，不能据此评估真实麦轮摩擦、牵引力或打滑。

## 资产安装与加载

description 包的安装规则会同时安装完整模型目录和 URDF。现有真机启动入口仍使用原先的 `duojin01.xacro`。本阶段范围为模型资产和安装打包；Gazebo 场景、ROS 桥接与启动入口在后续接入时配置。

在已有 ROS 2 Humble 开发环境中，从仓库根目录构建资产包：

```bash
colcon build --packages-select duojin01_description
source install/setup.bash
```

供以后独立 Gazebo 场景引用时，把包含 `dual_arm_mobile` 的父目录加入资源路径：

```bash
model_root="$(ros2 pkg prefix --share duojin01_description)/models"
export GZ_SIM_RESOURCE_PATH="${model_root}${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
```

场景中的模型引用为：

```xml
<include>
  <uri>model://dual_arm_mobile</uri>
</include>
```

直接使用源码资产时，父目录是 `src/duojin01_description/models`。模型名称和目录层级须保持一致，以解析 SDF 内的 `model://dual_arm_mobile/meshes/...` 引用。

## 来源与验证范围

本次导入核对了源资产摘要、网格面数、SDF / URDF 结构与网格路径，并在本地容器中验证 SDF 和 description 包的安装结果。

`asset_manifest.json` 中 Gazebo 8.12.0、ROS 2 Humble 及转换工具版本记录来自原模型转换阶段；此前的运动与传感器测试不代表本分支已完成系统接入。原 CAD 与派生外形的权利归原权利人。原生文件格式解析参考 [cadmpeg 格式研究文档](https://github.com/cadmpeg/cadmpeg/blob/52c8b0a4ceb9c5fbcdf2a8c21aecced98e6bceed/docs/formats/sldprt.md)，作者 cadmpeg，文档许可 CC-BY-4.0。
