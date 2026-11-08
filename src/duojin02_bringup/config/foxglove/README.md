# Foxglove 布局

| 文件 | 对应模式 |
| --- | --- |
| `mapping.json` | 建图 |
| `navigation.json` | 固定地图导航 |
| `online_navigation.json` | 在线建图与导航 |

在 Foxglove 的 **Layouts → Import from file** 中选择对应 JSON，再通过 Foxglove WebSocket 连接 `ws://机器人IP:8765`。布局需要导入到运行 Foxglove 的电脑；连接 bridge 不会自动从机器人读取布局。构建安装后，这些文件也位于 `share/duojin02_bringup/config/foxglove/`。

3D 面板固定坐标系为 `map`，默认显示 `/map`、`/scan` 和 `/robot_description`。导航布局额外配置 `/plan`、`/local_plan`、`/received_global_plan` 和机器人轮廓；局部与全局代价地图默认关闭，可在 3D 面板话题设置中开启。实际出现的话题取决于运行模式和当前任务状态，路径通常要发送导航目标后才产生。

3D 面板的 Publish 工具预设目标位姿话题 `/goal_pose`、初始位姿话题 `/initialpose`。固定地图导航先选择 2D pose estimate，在地图上设置初始位置和朝向，并确认扫描与地图对齐，再选择 2D pose 发送目标。执行前请确认场地安全及机器人状态。机器人模型依赖 `/robot_description`、TF 和网格资源可用；若模型不显示，请检查 Foxglove Problems 面板及 Bridge 的资源访问设置。

这些布局基于项目现有 RViz 配置和 Foxglove JSON 布局格式制作。Foxglove 版本可能改变布局格式或显示默认值；导入后应检查话题、TF、模型和发布工具，且目标发布功能需要在实际环境中验证。
