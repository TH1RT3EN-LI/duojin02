# 唯一底盘配置

当前唯一人工维护的参数文件是 [chassis.yaml](../../src/duojin01_bringup/config/chassis.yaml)。真机 bringup、当前 Jetson 导航和 canonical 仿真均从它派生所需参数；运行时生成的标准 ROS YAML 是缓存产物，不是第二份配置来源。

```yaml
motion:
  forward_mps: 0.5
  reverse_mps: 0.5
  lateral_mps: 0.5
  yaw_radps: 0.65
  narrow_mps: 0.5
```

前三项分别控制前进、后退、左右横移，填写正数幅值。比如设置 0.5 / 0.2 / 0.3，会自动导出 controller 的 `vx_max=0.5`、`vx_min=-0.2`、`vy_max=0.3`，并同步 smoother 的最大/最小速度。当前仍为各单轴 0.5 m/s，斜向没有新增合成速度限幅。

总配置的其它段：

| 段 | 内容 |
|---|---|
| `geometry` | 未膨胀车体多边形和 Z 范围；导航、guard、裁剪共用 |
| `lidar` | 硬件外参、量程、原始/过滤话题和投影裁剪策略 |
| `components` | 当前导航、定位健康、SLAM、EKF、串口底盘、速度仲裁、遥控和显示参数 |
| `offline_mapping` | 离线回放所需 SLAM 覆盖项 |
| `runtime_environment` | 当前算法的 OpenMP 与 GPU 执行参数 |
| `simulation` | 仿真传感器、材质、底盘；雷达高度覆盖为 0.14330766582 m |

量程下限只维护在 `lidar.min_range_m`，自动传给驱动、AMCL、SLAM 和仿真发布端。硬件雷达外参同时用于 robot_state_publisher 的 laser TF、裁剪和仿真 XY/RPY；仿真只单独覆盖光学 Z。CAD 网格与其它关节保持原结构。当前零方向仍未做实体校准。

在 GPU 工作区修改配置后执行：

```bash
python3 scripts/chassis_config.py check
python3 scripts/chassis_config.py adopt
```

`check` 校验结构、独立方向速度、数值和派生字段冲突；`adopt` 更新当前 release 对总配置及派生组件的校验值，不重编算法、不启动设备。部署到 Jetson 时同步该总配置及 release 清单，并执行 `python3 scripts/verify_orin_top1.py`。该工具校验配置和原生安装文件，不证明修改后具有新的精度或实时性能。

当前 Jetson 固定图导航入口仍为：

```bash
./scripts/navigation_top1.sh map:=/absolute/path/to/map.yaml
```

独立雷达、建图、在线导航、键盘等 bringup launch 只接受一个 `chassis_profile` 配置输入；目标地图、回放时间、输出目录和是否开启显示仍是每次运行的输入。Nav2 内部的 `params_file` 用于传递派生结果，不是旧参数文件的用户覆盖入口。

已删除旧 Nav2 双配置、局部驱动/guard/EKF/SLAM/遥控 YAML、独立仿真 profile，以及 GPU harness 的模拟器与评估器副本。仿真实现只维护在相邻 `duojin02-sim/simulation`，测试通过 `DUOJIN_SIM_SOURCE` 指向它。历史试验数据和归档源码保留在证据目录；它们不参与当前启动的版本选择或回退。
