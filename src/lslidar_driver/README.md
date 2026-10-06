# Duojin01 N10Plus 串口雷达

本目录是官方 V5.1.3 的项目裁剪版，只服务当前统一硬件。来源、许可证和裁剪范围见 [UPSTREAM.md](UPSTREAM.md)。

```bash
ros2 launch lslidar_driver lslidar_x10_launch.py serial_port:=/dev/lslidar
```

输出标准 `/scan` 和 `/lslidar_point_cloud`。协议固定为 460800、8N1、108 字节双回波、540 格扫描；保留串口、话题、坐标系和距离过滤参数。配置见 `lslidar_driver/config/duojin01_n10plus.yaml`。

本分支不提供其他雷达型号、网络连接、旧参数或自定义设备服务兼容层。雷达测试只使用伪终端和构造数据：

```bash
./scripts/test_lidar.sh
```
