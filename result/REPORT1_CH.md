# Sphere-VIO MSCKF 改进日志

## 基线

在 `manual_small_1-sync.bag` 上运行的全序列 MSCKF 先验结果仅使用跨相机立体 `LandmarkTrack` 特征。共创建了 162 条轨迹，并出现发散：ATE 平移 RMSE = 5208.22 m。
旧的离线处理流程需要中间转换生成的 Sphere-VIO bag，并复用当前帧的 `point_b` 配合任意较早的克隆位姿，这对于时间上跟踪的特征而言在几何上不一致。

## 算法改动

- 直接 D2SLAM 数据接入：每帧解码一次 `/arducam/image/compressed`，将其分割为四张同步的单通道灰度图，避免生成中间全尺寸 bag。修正了每个相机的图像循环逻辑，确保在组装帧之前四张子图均已入队。
- 原始 D2SLAM IMU 选择：当拼接后的 IMU 话题存在时，使用 `/dji_sdk_1/dji_sdk/imu`，而不再使用转换后 bag 中的 `/imu_data_raw`。
- MSCKF 特征模型：引入 `MsckfFeatureAccumulator`，使每个相机的单目时序特征能够进入滤波器，而不仅仅是立体确认的轨迹。当可用时，不同相机间的 Landmark 轨迹会被合并。
- 特征锚点一致性：特征根据其自身的克隆观测进行三角化，并使用三到五次 Gauss-Newton 重投影步骤进行优化，不再在锚点位姿不一致的条件下复用机体坐标系下的点。
- 基于特征层的卡方检验用于剔除离群/过时的时序特征。
- MSCKF 参数移至 `config/system.yaml` 中的 `backend.msckf` 下；同时提供命令行覆盖选项以便快速试验。
- 增加了规范格式的 `trajectory.csv` 输出，列顺序为：`timestamp,x,y,z,qx,qy,qz,qw`。
- 将跨相机匹配限制为数据中实际可见的两组重叠对，即 `C0-C1` 和 `C2-C3`。

## 全数据集结果

| 序列 | ATE_SE3 (m) | ATE_Sim3 (m, ref) | RPE_1m (m) | RPE_5m (m) | RPE_10m (m) | coverage | success |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| eight_noyaw_1 | 241109.33 | 1.5663 | 102.36 | 102.39 | 102.50 | 0.99994 | False |
| eight_noyaw_3_short | 85400.17 | 1.5668 | 50.50 | 50.57 | 50.88 | 0.99982 | False |
| eight_noyaw_4 | 288121.44 | 1.5533 | 109.13 | 109.15 | 109.24 | 0.99979 | True |
| eight_noyaw_5 | 283089.84 | 1.5577 | 110.78 | 110.80 | 110.89 | 0.99986 | False |
| eight_yaw_1 | 67796.28 | 1.5628 | 34.82 | 34.93 | 35.46 | 0.99433 | False |

所有序列均顺利完成，未出现 NaN/Inf 且覆盖率较高。剩余的较大 `ATE_SE3` 主要由尺度/漂移主导：`ATE_Sim3` 约为 1.55 m，表明轨迹形状更接近真值，但绝对度量尺度和长期漂移尚未修正。

## 待完成工作

- [已完成] IMU 偏置和重力方向现已通过短时静止 IMU 窗口进行初始化（`initialization_duration`、`minimum_initialization_samples`），估计陀螺仪偏置、加速度计偏置以及重力对齐的初始姿态。
- [部分完成] 特征深度现已保存在以稳定特征 id 为键的持久化 `feature_positions_` 映射中，并用于为每次 Gauss-Newton 优化提供热启动。这是一个持久化的点估计，尚未成为滤波器协方差中的路标块。
- [部分完成] MSCKF 现在会在积累足够克隆位姿后执行一次性的在线相机/IMU 时间偏移估计，并在将视觉观测与克隆位姿关联时使用该偏移。
- [部分完成] 增加了可配置的机体系外参旋转/平移摄动，在投影过程中施加。这些摄动的在线估计仍然缺失。
- [部分完成] 增加了运行时调控器，当某帧超过 `budget_ms` 时降低每个相机的特征数量，并限制了 `maximum_matches_per_pair`。一次 10 秒的冒烟测试从约 78 秒改善至约 18.6 秒墙钟时间。调控器现在还会在达到特征数量下限后降低 LK 金字塔层数，并且 ORB 描述子上限（`maximum_descriptors`）减少了描述子提取。当前冒烟测试的 `avg_frame_time_ms` 约为 96 ms，仍高于 62.5 ms 的预算。

## 下一步增量

- 将相机/IMU 时间偏移和一个小幅外参摄动及其投影雅可比加入 MSCKF 状态。
- 将持久化点地图提升为真正的路标协方差块，或加入逆深度边缘化，使长轨迹能够约束度量尺度。
- 继续降低运行时间，直到 `avg_frame_time_ms <= 62.5 ms`，可能的途径包括减小 LK 搜索窗口、并行化描述子提取，或采用帧步进的后端调度。

## 文件

- 各序列报告：`result/report_<sequence>.md`
- 完整运行日志：`result/run_<sequence>.log`
- 输出轨迹：`output/msckf_<sequence>/trajectory.csv`

