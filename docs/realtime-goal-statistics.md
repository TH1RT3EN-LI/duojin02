# 共享误差实验模型与证据采集

`duojin02_slam_tools.shared_evidence` 是可执行的离线/影子算子，不连接 AMCL 权重、TF 或控制命令。默认定位仍由 PF 唯一拥有。独立真实标定、保留查询和实际停止政策验收缺失时，该模型不进入默认生产权重。

旧近似将每束、每帧的势函数独立相乘，无法表达持续地图/表面误差。本算子在明确的固定线性化下使用

`y = J x + B eta + epsilon`, `V = R + B Sigma_eta B^T`，

其中共享变量必须给出物理意义、来源和寿命；R 仅为条件剩余误差。`gaussian_nll` 使用完整二次型、logdet 和正规化常数。状态相关 V 的独立导数算子包含 logdet 导数；窗口类本身明确要求固定 J/B/线性化版本，不能拿它冒充已实现的非线性共享状态滤波器。

跨扫描追加后，以整个联合块的 NLL 差作为条件增量。同一 eta 不逐帧重置。相同 evidence ID、相同内容是无操作；不同 ID 重用物理 sample、相同 ID 改内容、几何换版和窗口溢出均原子拒绝，保留旧历史。窗口不删因子，也未声称有固定滞后边缘化。引用的 ndarray 由不可写 bytes buffer 持有，调用者不能重新启用 writeable 来绕过已记录的 fingerprint。

这只是指定生成模型的正确算子，不证明变量选择、独立性、物理噪声值或可观性正确。机器人—目标互协方差、外参和时钟误差均未获得物理标定；不会填入一个小方差或声称已满足 95%。

## 执行

在具有 NumPy 的项目 ROS 环境运行：

```bash
python3 -B tests/realtime_goal/statistics/test_shared_evidence.py
python3 scripts/realtime_goal/shared_model_shadow.py --input explicit-model.json --output new-result.json
python3 scripts/realtime_goal/export_calibration_capture.py --residuals raw-shadow-residuals.jsonl --output new-capture.json
```

`explicit-model.json` 必须提供 state_dimension、variables、shared_covariance、geometry_identity、linearization_identity、max_observations、evaluation_state、covariance_assumption_provenance 及 blocks；每个 block 完整给出 evidence_id、measurement_ids、query_source、geometry_identity、reference_time_ns、y/J/B/R。没有默认噪声。完整示例及运行结果在本轮独立实施目录 `statistics/synthetic-input.json`，其数字仅为代数 fixture。

真实采集索引保留 query source/beam、surface source、geometry/version 和重复使用关系。D0/D1 重用同一束不变成两次独立测量。当前 E3 残差经过拟合位姿和关联选择，不能用其较小的样本方差证明独立物理覆盖。

独立测试覆盖闭式共享误差信息上限、独立极限、条件似然链、排列不变、坐标密度常数、logdet 导数、Schur/联合 MAP 消元、去重和不可变所有权。均值收益与覆盖收益均未在独立真实数据上证明。
