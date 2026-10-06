# 雷达驱动来源与项目裁剪

源自 [Lslidar/Lslidar_ROS2_driver](https://github.com/Lslidar/Lslidar_ROS2_driver/tree/ecd9a836a922db714236aba7a0f54508adcb7e0e)，固定提交 `ecd9a836a922db714236aba7a0f54508adcb7e0e`，上游发布标识 `V5.1.3_250822`。本地包版本保留 `5.1.3`，使用 Apache-2.0，许可证见 LICENSE。

2026-10-06 按统一硬件裁剪为 N10Plus 串口路径。保留官方协议的角度、距离和强度解码；删除 CX / CH / LS、其他 X10 型号、网络 / PCAP、设备信息服务、矩阵变换、设备时间和多回波输出模式。不承诺兼容上游所有型号或旧配置参数。

| 项目 | 当前实现 |
| --- | --- |
| 连接 | `/dev/lslidar`，固定 460800、8N1 |
| 协议 | 108 字节、16 组双回波；校验头与累加和 |
| 输出 | `/scan`、`/lslidar_point_cloud`，默认 frame 为 `laser` |
| 扫描 | 当前 10 Hz 硬件对应 540 格；每格最近有效距离 |
| 角度 | 固定顺时针协议转 ROS 角度；±π 同格 |
| 时间 | 主机接收时间推算帧结束；逐束时间未知，time_increment 为 0 |
| 配置 | 串口、话题、坐标系、min_range / max_range |
| 电机 | `/motor_control`，0 停转 / 1 旋转 |

原始包改为栈上的 108 字节数组，删除未发布的 lslidar_msgs 包。LaserScan 直接读取本帧 PCL 点云，只为标准 PointCloud2 输出执行一次序列化。采集与发布各用一个工作线程，ROS 回调用单线程执行器；每次发布捕获独立帧和时间，析构时排空发布队列。

串口累积分段输入，保留暂时无数据与设备断开的区分；坏包丢弃，传输失败终止节点。删除按未知型号猜测包长的路径。验证使用构造协议和伪终端，见 `scripts/test_lidar.sh` 与 [精简记录](../../docs/hardware-simplification.md)。实际铭牌、固件和真机采集仍由板端联调确认。
