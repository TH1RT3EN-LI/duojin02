# Duojin01 N10Plus serial lidar

This is a project-specific subset of the official V5.1.3 driver for one hardware configuration. See [UPSTREAM.md](UPSTREAM.md) for source provenance, Apache-2.0 licensing and the removed features.

```bash
ros2 launch lslidar_driver lslidar_x10_launch.py serial_port:=/dev/lslidar
```

The driver publishes standard LaserScan and PointCloud2 messages. The protocol is fixed at 460800 baud, 8N1, 108-byte dual-echo packets and 540 scan bins. Serial path, topics, frame ID and range filtering remain configurable. Other models, network transports and legacy parameters are unsupported.

Run `scripts/test_lidar.sh` from the workspace root for offline protocol and pseudo-terminal tests.
