# Duojin02 N10Plus serial lidar

This is a project-specific subset of the official V5.1.3 driver for one hardware configuration. See [UPSTREAM.md](UPSTREAM.md) for source provenance, Apache-2.0 licensing and the removed features.

Build and source `./source.sh` from the workspace root first. Use this entry
for standalone observation; do not duplicate a driver already owned by mapping
or navigation launch.

```bash
ros2 launch lslidar_driver lslidar_x10_launch.py serial_port:=/dev/lslidar
```

The driver publishes standard LaserScan and PointCloud2 messages. The protocol is fixed at 460800 baud, 8N1, 108-byte dual-echo packets and 540 scan bins. Serial path, topics, frame ID and range filtering remain configurable. Other models, network transports and legacy parameters are unsupported.

The standalone driver does not publish the robot-to-map TF chain. For a scan-only view, use the scan frame (default `laser`) as RViz's Fixed Frame. See the [RViz observation guide](../../docs/remote-rviz.md) for display modes, remote domains and QoS checks.

Run `scripts/test_lidar.sh` from the workspace root for offline protocol and pseudo-terminal tests.
