# ESKF / IMU 后端与 ROS 输出层说明

## 1. 来源

后端实现主要参考：

- `open_vins/ov_msckf/src/state/Propagator.cpp` 的
  `fast_state_propagate()`：15 状态 IMU 误差状态、中点积分、离散
  `F/G/Qd` 传播公式。
- `open_vins/ov_core/src/types/IMU.h` 的误差状态变量组织方式，即
  `[orientation, position, velocity, gyro_bias, accel_bias]`。
- `OmniNxt/README.md` 与 `misc/OmniNxt.pdf` 中“统一全景表示 + 视觉惯性
  后端”的思想；该目录只有论文和说明，没有可直接复制的源码。
- 本仓库已有的 `TriangulationCandidateEvaluator` 和
  `LandmarkTrackManager`：ESKF 的视觉观测来自当前 LandmarkTrack 最近一次
  三角化诊断，而不是重新实现一套特征前端。

## 2. 功能

当前新增功能是一个可运行的初版后端，不是完整 MSCKF 或 bundle adjustment。

核心后端：

- 15 维误差状态：`[theta, p_wb, v_wb, bg, ba]`。
- 名义状态包含 `q_wb, p_wb, v_wb, bias_gyro, bias_accel`。
- IMU 中点积分：角速度、加速度去偏后积分位置、速度和四元数。
- 协方差传播：离散状态转移矩阵、噪声雅可比和 IMU 噪声协方差。
- 位置更新：把 LandmarkTrack 的 `point_b` 与后端维护的 `point_w` 作为
  位置观测，修正姿态与平移。
- 速度更新：预留了零速度/外部速度更新接口。
- LandmarkMap：从三角化候选初始化世界系地标，并在后续观测中作为位置
  观测约束漂移。

ROS1 交互层：

```text
/sphere_vio/odometry   nav_msgs/Odometry
/sphere_vio/path       nav_msgs/Path
/sphere_vio/landmarks  sensor_msgs/PointCloud2
TF                     world -> body
```

离线保存：

```text
output_dir/odometry.csv
output_dir/landmarks.csv
```

## 3. 实现方式

新增核心库 `sphere_vio_backend`：

```text
include/sphere_vio/backend/eskf.hpp
include/sphere_vio/backend/landmark_map.hpp
src/backend/eskf.cpp
src/backend/landmark_map.cpp
```

新增 ROS 输出层：

```text
include/sphere_vio/ros/ros_output.hpp
include/sphere_vio/ros/output_recorder.hpp
src/ros/ros_output.cpp
src/ros/output_recorder.cpp
```

`OfflineFeatureRunner` 已经接入后端。启用方式：

```bash
rosrun sphere_vio sphere_vio_feature_runner \
  --config /root/catkin_ws/src/sphere_vio/config/offline.yaml \
  --bag /data/sphere_algorithm_test.bag \
  --cross-camera-matching \
  --triangulation-candidates \
  --landmark-tracks \
  --esfk \
  --output-dir /output/esfk
```

如果需要 ROS 发布，再加上 `--publish-ros`，并确保 `roscore` 已运行：

```bash
rosrun sphere_vio sphere_vio_feature_runner \
  --config /root/catkin_ws/src/sphere_vio/config/offline.yaml \
  --bag /data/sphere_algorithm_test.bag \
  --esfk \
  --output-dir /output/esfk \
  --publish-ros
```

后端处理顺序是：

```text
四路图像装配
  -> 提取上一帧到当前帧的 IMU 区间
  -> TemporalFrontend
  -> ORB 跨相机匹配
  -> 三角化候选
  -> LandmarkTrack 管理
  -> ESKF 地标位置更新
  -> CSV / ROS / TF 输出
```

## 4. 当前限制

- 视觉更新是简化的“地标世界点作为位置测量”，没有实现 MSCKF 滑动窗口
  clone、地标状态增广或严格重投影雅可比。
- 没有在线估计 IMU 到相机外参和时间偏移。
- 噪声参数仍是初始值，需要用真实四目鱼眼加 IMU 数据调参。
- `--esfk` 依赖前端先产生可靠的三角化候选；在纹理不足或同步不好时，
  后端会退化为纯 IMU 积分。

## 5. 验证

在 Docker 容器内重新编译并通过全部单元测试，新增 `test_eskf` 覆盖：

- IMU 均值与协方差传播。
- 位置更新。
- LandmarkMap 新建与重复观测。

合成 bag 上已实际运行 `--esfk --output-dir`，成功生成
`odometry.csv` 和 `landmarks.csv`。
