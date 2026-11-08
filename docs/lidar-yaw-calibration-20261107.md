# 雷达水平角暂定修正（2026-11-07）

在 Jetson 实物静止时读取雷达近距离回波，单独诊断进程临时设置最小距离 0.05 m，未修改正式 0.30 m 配置，也未发送运动指令。六组稳定回波与模型六处柱状截面进行固定关联，按名义雷达扫描高度 0.1302326658 m、名义 XY 外参，最小化每组点到对应 CAD 截面折线距离（单点距离截断 20 mm，各组等权）。此几何拟合为近场自检，不是已知真值标定。

第一组 30 帧：拟合角 +3.7°，代价由 6.877 mm 降至 4.234 mm；前后各 15 帧分别为 +3.8° 和 +3.5°，六组单独拟合为 +2.9° 至 +4.5°。另一组稍后静止采集的 30 帧、相同场景：固定 +3.7° 的代价由 5.844 mm 降到 2.976 mm；该组自行拟合最优约 +2.9°。两次同一静态场景并不独立证明真实物理外参精度。

若允许雷达水平位置在 ±12 mm 内随角度一起变化，第一组粗网格最优为角 +2.5°、x/y 均 -3 mm，表明角与位置存在耦合。CAD 模型高度是未经实测的代理值；雷达近距离测量、遮挡、部件对应和物理装配误差均未标定。故 +3.7° 是**暂定设置**，需用已知物理靶标或其他独立数据确认，不能作为实车定位/安全验收。

`lidar_yaw_deg` 启动参数设置水平角，默认 +3.7°（从上方看绕 +Z 逆时针），传入 `0` 可回到原始方向。适用于 `base.launch.py`、`mapping.launch.py`、`navigation.launch.py` 和 `online_navigation.launch.py`；在 Xacro 中转换为弧度后由 `lidar_joint` 发布 TF，驱动数据和最小测距不变。现有已运行的 robot_state_publisher 不会自动更新，需重启相关 launch。修改前远端模型文件备份于 `~/duojin02-cuda-local-20261107/output/lidar-yaw-20261107/`。

拟合代码、输入和结果：`output/lidar-chassis-20261107/fit_alignment.py`、`fit-alignment.json`；新采集检查为 `output/lidar-chassis-validation-20261107/`。图示分别为 `output/lidar-chassis-20261107/lidar-cad-overlay.png` 与 `lidar-cad-overlay-corrected.png`。这些 output 文件为本机工作区数据，可能不随 Git 跟踪。
