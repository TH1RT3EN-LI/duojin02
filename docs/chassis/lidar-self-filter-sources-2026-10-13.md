# 底盘雷达自过滤：社区与论文筛查

访问与核验日期：2026-10-13。范围仅为底盘；论文中上装的例子只用于解释通用误删机制，不纳入本项目过滤模型。

## 结论与证据边界

已有可用的几何方案能处理偏置安装雷达和非圆底盘。当前应先修正量程下限、外参和自身实体模型，再比较过滤结果；本轮没有找到足以支持引入大模型、语义网络或动态物体剔除算法的强相关证据。

固定径向量程裁剪、实体包含判定、射线遮挡判定是三个不同操作。模型有误或包络过大时，几何过滤同样会误删有效近距点；换成 URDF 包并不能自动消除这个问题。

## 社区原始实现

### 1. ROS 官方 `laser_filters`：固定核验 2.0.9

- [2.0.9 polygon_filter.h 原始源码](https://github.com/ros-perception/laser_filters/blob/2.0.9/include/laser_filters/polygon_filter.h)
- [2.0.9 package.xml](https://github.com/ros-perception/laser_filters/blob/2.0.9/package.xml)

`LaserScanPolygonFilter` 通过 TF 将扫描点投到指定多边形坐标系，再判定内外，删除值为 NaN。`StaticLaserScanPolygonFilter` 适用于雷达与底盘外参固定的情况，缓存多边形变换和三角函数。该版本的包元数据为 ROS 2 的 `ament_cmake`。

本项目适用性判断：固定底盘不需要复杂网络，现成多边形过滤可以作为基线。不过它只处理用户给定的几何，不会自动判断导航 footprint 是否等于激光能照到的实体截面，也不会恢复已经被上游删除的回波。

### 2. CTU-VRAS `robot_body_filter`：本轮核验主线为 ROS 1

- [作者仓库与参数说明](https://github.com/ctu-vras/robot_body_filter)
- [master package.xml 原始文件](https://raw.githubusercontent.com/ctu-vras/robot_body_filter/master/package.xml)

该实现以 URDF 与 TF 做实体包含和射线遮挡测试，量程裁剪可独立配置；可指定参与过滤的 link 和不同 padding。核验的主线包版本为 1.3.2，构建工具是 `catkin`，属于 ROS 1。

本项目适用性判断：可借鉴其几何和分类接口，不能将它直接写成已验证的 Humble 可用包；分支名或索引中的 ROS 2 标记不代替构建验证。射线遮挡和膨胀模型应单独评估，不能为消除自身残点而扩大到附近墙面。

### 3. leggedrobotics `robot_self_filter`：ROS 2 候选

- [作者仓库与使用说明](https://github.com/leggedrobotics/robot_self_filter)
- [main package.xml](https://github.com/leggedrobotics/robot_self_filter/blob/main/package.xml)

核验的包版本为 1.1.0，使用 `ament_cmake`。它通过 URDF 实体与 TF 区分内部、外部和遮挡点，输入为 PointCloud2。示例仍配置 `min_sensor_dist: 0.3`，因此不能照搬配置；本轮未验证其在本机 Humble 的构建和速度。

本项目适用性判断：适合需要真实三维实体过滤的扩展版本，配置仅选择底盘 link。默认量程和模型 padding 必须另行校核，不能当作本型号雷达的硬件规格。

## 学界原始论文

以下均为简短转述，没有大段摘录；没有将论文的整机实验结果当成本底盘的性能证据。

### 4. Rusu 等，IROS 2009

题名：*Real-time Perception-Guided Motion Planning for a Personal Robot*，IEEE/RSJ IROS 2009，4245–4252，DOI 10.1109/IROS.2009.5354396。

- [作者实验室论文页](https://www.kavrakilab.org/publications/rusu-sucan2009real-time-perception-guided-motion.html)
- [作者实验室 PDF](https://www.kavrakilab.org/publications/rusu-sucan2009real-time-perception-guided-motion.pdf)
- [另一作者的 PDF](https://files.sucan.ro/ioan/files/pubs/perceptionplanning_iros09.pdf)

PDF 第 3 页、§III：用机器人部件的凸包判定自身测量。第 4 页末段同时记录：夹爪凸包大于真实部件，造成手持物体的有效点被删。这是“过大的几何模型也会误删外点”的直接反例。

本项目推论：导航碰撞包络和膨胀安全边界不能直接作为光学自掩膜。这里借用误删机制，不处理夹爪或机械臂。

作者实验室 PDF 已通过 web 打开为 8 页，并经内存下载核验：1,920,696 bytes，文件头 `%PDF-1.4`，SHA256 `b9b4dfa2a0c3aaf59391592b2072a7cbc688b1b98c00927b7350400740c3602f`。本轮未在本地保存 PDF。

### 5. Holz 等，ISPRS 2013

题名：*Towards Multimodal Omnidirectional Obstacle Detection for Autonomous Unmanned Aerial Vehicles*，ISPRS Archives XL-1/W2，201–206，DOI 10.5194/isprsarchives-XL-1-W2-201-2013。

- [出版社论文页](https://isprs-archives.copernicus.org/articles/XL-1-W2/201/2013/)
- [出版社原始 PDF](https://isprs-archives.copernicus.org/articles/XL-1-W2/201/2013/isprsarchives-XL-1-W2-201-2013.pdf)

PDF 第 3 页、§4.3：使用机器人几何模型判断测量是否与自身部件重合，区分本体和附近障碍点。该系统为旋转三维雷达 UAV；论文没有给出本底盘的近距外点误删率。

本项目推论：模型匹配是可借鉴机制，但其速度或整机实验不能直接作为本项目验证结果。

## 本任务必须保留的限制

### 扫描高度与模型

自过滤模型应描述实际激光扫描平面中能被照到的底盘实体，或用完整三维实体与射线进行判定。导航 footprint 是底盘的 XY 碰撞投影；当雷达在某块实体上方时，该投影内的点不一定是自身回波。以上属于本任务的几何推论，现成多边形包不会替使用者保证模型正确。

必须采用雷达到模型坐标系的完整外参，包括平移、旋转和高度。固定外参可以缓存；若模型或安装状态改变，必须更新。小而准确的实体模型比扩大包络更符合避免误删外点的目标。

### 双回波与信息丢失

本地 N10Plus 驱动的数据解码会读取每个测量组的两个 echo。LaserScan 每个角度 bin 只存一个距离：若先选最近回波、再做下游自过滤，近处自身 echo 会压掉远处有效外部 echo；过滤后的单值扫描无法恢复被压掉的信息。

因此，若两个 echo 在上游可用，应先对每个回波做自身分类，再在剩余有效回波中选择最近距离。任何已经被量程下限、设备协议或发布格式丢掉的回波，都无法由后续几何算法凭空恢复。这是数据接口限制，不是上述论文证明的改进收益；也不意味着所有被遮挡方向都实际存在可用第二回波。

### 边界三态：本项目建议，尚未被上述论文实证

可将结果分为“确定自身”“确定外部”“边界不确定”，硬删除仅针对有充分模型依据的自身点。外参与测距误差附近的模糊点保留原始值并单独标记，避免把“靠近模型”直接等同于自身；下游如何使用这类点需要单独验证。

这是偏向保护外部近距点的保守设计建议，不是声称引用论文已经证明最优或零误删。误差带不能仅凭参数偏好确定，应通过实测校核。删除自身点也不提供其后方为空闲的证据，输出语义不能误触发清障。

后续比较至少应分开统计外部近墙点误删率、自身点漏删率、保留回波数和运行耗时；只看删点数量或定位健康计数不能证明过滤更正确。
