# RViz 观察：雷达、模型与导航

机器人在 Jetson 运行，RViz 在观看机运行。观看机只启动 RViz，不重复启动底盘、雷达、SLAM、Nav2 或模型 / TF 发布节点。机器人启动入口与参数范围见 [主 README](../README.MD)，定位许可与恢复见 [定位健康](localization-health.md)。

## 先统一环境

两端使用 ROS 2 Humble。显示模型时，观看机需有同版本 `duojin01_description` 和网格，按本机架构构建并加载；ARM64 安装产物不能直接复制给 x86 观看机。以下示例在观看机已构建的仓库根目录运行，机器人已在另一端启动：

```bash
source ./source.sh
export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0
```

`20` 是项目默认值；机器人使用其他域时两端改为该实际值。`source.sh` 保留已有域与 RMW 设置，默认 RMW 为 Cyclone DDS；不要以为重新 source 会清除之前的实验域。直接跨机器订阅还要求机器人进程的 `ROS_LOCALHOST_ONLY=0`、DDS 配置允许远端发现及传输。改变终端环境不会修改已启动进程的环境。

只看扫描可使用已安装 Humble / RViz 的最小环境；上例的 `source.sh` 需要已构建工作区。完整模型显示须确认 `package://duojin01_description/...` 在观看机可解析。机器人使用自定义 RMW / DDS 网络配置时，按其实际部署核对发现和 QoS；本页不提供另一套桥接配置。

## 只看雷达

```bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=false
```

在 RViz 添加 LaserScan，Topic 选 `/scan`，Reliability 选 Best Effort、Durability 选 Volatile、Decay Time 设为 0。按机器人实际运行内容选择 Global Options → Fixed Frame：

| 已运行的内容 | Fixed Frame | 能看到什么 |
| --- | --- | --- |
| 仅独立雷达驱动 | 扫描 `header.frame_id`，默认 `laser` | 雷达自身坐标中的当前扫描，无需地图 TF |
| 底盘 / EKF 与雷达，尚无定位或建图 | `odom`，确认存在到扫描坐标的 TF | 里程计坐标中的扫描 |
| 建图或已就绪的固定图导航 | `map`，确认完整 `map → odom → base_footprint → laser` | 地图坐标中的扫描 |

独立雷达不发布上述机器人 TF 链，不能加载默认 `map` 模板后就把空白判成雷达没数据。先看 LaserScan 的 Status；显示平稳也不证明采集时序、导航实时性或供电稳定。

## 看模型和导航

观看机使用仓库模板启动，下面是固定图 / 在线导航的示例：

```bash
ros2 run rviz2 rviz2 -d "$PWD/src/duojin01_bringup/config/rviz/navigation.rviz" \
  --ros-args -p use_sim_time:=false
```

建图时改用 [mapping.rviz](../src/duojin01_bringup/config/rviz/mapping.rviz)。两个模板的 Fixed Frame 均为 `map`，不能用于没有地图 TF 的独立雷达观察。

当前 [navigation.rviz](../src/duojin01_bringup/config/rviz/navigation.rviz) 有一项名为 `GlobalCostMap`，实际 Topic 是 `/local_costmap/costmap`。要检查全局代价地图，应在该显示项将 Topic 改成 `/global_costmap/costmap`，先确认实际发布者；显示项名称不代表话题来源。需要保存观看设置时使用自己的 RViz 配置文件。

RobotModel 接收 `/robot_description`，网格由观看机本地解析；机器人端已有模型 / TF 发布者，观看机不再启动一套。模板中的 RobotModel，以及建图模板的 Map，当前 Durability 为 Volatile。晚加入时如历史模型 / 地图未出现，先用发布端实际 QoS 核对对应显示项；发布端为 Reliable / Transient Local 时将接收项匹配到它，不能靠重复发布另一套 TF 解决。导航模板的 `/map` 已配置 Reliable / Transient Local。

## 按现象排查

| 现象 | 核对顺序 |
| --- | --- |
| 观看机找不到 `/scan` | 域 → 已启动进程的网络限制 / DDS 配置 → 发布者；不要再启动一个驱动掩盖问题 |
| 有话题，但 LaserScan 无数据 | `ros2 topic info /scan --verbose` 的实际端点和 QoS → RViz 订阅配置 |
| 扫描 Status 报缺变换 | Fixed Frame 是否属于当前模式 → 精确扫描时间的 TF → 两端时钟及 `use_sim_time` |
| 雷达可见，模型空白 / 网格错误 | `/robot_description` 接收 QoS → 本地描述包与网格路径 → TF |
| 地图 / 规划项空白 | 实际对应节点是否 ACTIVE、话题是否发布、显示项实际 Topic 与 QoS；当前导航模式不保证模板每一项都有数据 |

系统时钟用于当前真机图；离线录包的 `/clock` 只属于其独立回放域。不要让真机 RViz 等待未发布的仿真时钟。

## 显示负载与证据边界

只看雷达时先启用 LaserScan，按需要增加地图、模型和路径。独立进程 / 线程仍可能增加 DDS、序列化和网络成本；远程查看并不自动隔离控制资源。两个模板当前渲染上限为 60 FPS，可在观看机降低 Frame Rate 或关闭不需要的显示项，机器人传感器与安全周期保持原值。关闭显示项不证明其订阅已销毁；负载对照用零个、一个、多个 RViz 进程，并确认实际订阅与进程数。

已有控制 / 显示分域桥接属于实验夹具，尚无默认部署入口；域 77/78 不替代上面的直接订阅默认域 20。当前验证范围见 [优化状态](navigation-optimization.md)，板端显示负载验收方法见 [导航验证](navigation-validation.md)。观察说明不证明跨机器发现、晚加入显示或多端性能已经验收。
