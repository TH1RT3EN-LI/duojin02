# 当前 Jetson 导航工作区

底盘参数的唯一来源为 [chassis.yaml](src/duojin01_bringup/config/chassis.yaml)，使用 [配置派生模块](src/duojin01_bringup/duojin01_bringup/chassis_profile.py) 为所有节点生成标准 ROS 参数。前进、后退和左右横移仍有独立入口，当前均为 0.5 m/s。

```bash
python3 scripts/chassis_config.py check
# 修改总配置后更新当前配置校验值
python3 scripts/chassis_config.py adopt
# Jetson 上检查原生配置与算法产物
python3 scripts/verify_orin_top1.py
./scripts/navigation_top1.sh map:=/absolute/path/to/map.yaml
```

最后一个命令启动实体导航。`--show-args` 仅查看入口声明。当前算法沿用已选 ARM v4 库，控制 20 Hz、local costmap 更新 10 Hz、发布上限 20 Hz；单轴限幅不等于全局平移模长限幅。

字段、真机与仿真共享方式及迁移边界见 [唯一配置说明](docs/chassis/configuration.md)。旧 Nav2 配置和重复的仿真/评估器实现已移除，canonical 仿真只维护在相邻 `duojin02-sim/simulation`。历史性能、失败与原始源归档见 [文档索引](docs/README.md)，不作为当前版本回退入口。

速度选择的历史试验见 [速度收尾记录](docs/chassis/speed-envelope-2026-10-13.md)。本次迁移保持原有效参数，未据此增加精度、实时性或实体制动能力的结论。
