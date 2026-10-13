# Duojin01 N10Plus 串口雷达

本目录是官方 V5.1.3 的项目裁剪版，只服务当前统一硬件。来源、许可证和裁剪范围见 [UPSTREAM.md](UPSTREAM.md)。

先在工作区根目录完成构建并加载 `source ./source.sh`。独立观察雷达时执行；已有建图 / 导航入口运行时不要重复启动驱动：

```bash
ros2 launch lslidar_driver lslidar_x10_launch.py serial_port:=/dev/lslidar
```

输出标准 `/scan` 和 `/lslidar_point_cloud`。协议固定为 460800、8N1、108 字节双回波、540 格扫描；保留串口、话题、坐标系和距离过滤参数。配置见 `lslidar_driver/config/duojin01_n10plus.yaml`。

独立驱动不发布机器人到地图的 TF；只看雷达时 RViz 的 Fixed Frame 使用扫描坐标（默认 `laser`）。完整模型、远程域与 QoS 排查见 [RViz 观察](../../docs/remote-rviz.md)。

本项目不提供其他雷达型号、网络连接、旧参数或自定义设备服务兼容层。雷达测试只使用伪终端和构造数据，在工作区根目录执行：

```bash
./scripts/test_lidar.sh
```
