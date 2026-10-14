# 固定地图定位质量诊断

固定图启动 `localization_monitor`，按扫描实际时间查询 `map → laser`，核对有限返回端点与占据格，并用逐格 DDA 检查返回之前的射线是否穿过墙体。原有几何评分保持不变，质量阈值集中在 [localization_monitor.yaml](../src/duojin02_bringup/config/localization_monitor.yaml)。

监视器只发布 `/localization/status` 的 JSON 与限频警告，不消费或发布速度，不激活 / 停用 Nav2，也不自动重定位。输出包含 `advisory_only: true`、`quality`（`consistent` / `inconsistent` / `unavailable`）、扫描时间和原始证据分母。没有评分通过窗口、失配锁存、恢复重试或速度缓存。扫描缺失时不会产生新的评分；保留的旧状态必须结合 `emitted_ros_ns` 与 `scan_stamp` 阅读。

固定图与在线 SLAM 统一使用 `Nav2 → velocity_smoother → twist_mux → VelocityGuard → 底盘驱动`。启动只等待必要的新鲜扫描与 TF，停车条件见 [停车保护](safety.md)。质量评分低、AMCL 协方差大均不直接停车；无效地图或不可用的坐标变换仍可能令使用这些数据的导航 / 制动计算无法完成，不能把它们视为可使用的几何数据。

## 观测语义

项目 N10Plus 的 `+Inf` 表示过滤后没有有效返回的角度格，不能证明该方向无障碍。质量模型不给这种格增加命中、已知支持或空闲证据。有限返回超过比较范围时仍检查其前方射线，但不算端点命中。未知与中间占据概率不提供已知支持；`NaN`、`-Inf`、低于 `range_min` 的返回及错误元数据作为无效观测报告。

评分保留端点支持率、已知比例、穿墙矛盾比例、有效命中数与有界射线工作量。地图遵循固定 Humble AMCL/Karto 的最近格约定，不移动保存地图的原点；旋转地图不在此评分模型支持范围内。

这些阈值尚未经过实物标定。动态遮挡、地图不完整与场景变化会影响评分；重复走廊等全局对称位置可能获得高分。因此评分用于排查定位问题，不能证明位置正确或运动安全。

## 初值与恢复

在 RViz 用 **2D Pose Estimate** 发布真实初值，检查地图与扫描对齐，再发送目标。需要全局重定位时先取消正在执行的导航目标，使用 AMCL 原生服务：

```bash
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty '{}'
ros2 topic echo /localization/status
```

服务响应只表示 AMCL 接受了重初始化；候选位置是否正确仍需检查。当前没有项目自建的 `/localization/recover` 服务，也没有质量恢复后重放速度的路径。旧状态机与缺测规则的历史证据保留在 [定位健康修正历史](localization-health-history.md)，不作为当前部署契约。
