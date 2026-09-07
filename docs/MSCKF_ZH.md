# MSCKF 滑动窗口与球面重投影说明

## 1. 来源

该实现参考：

- `open_vins/ov_msckf/src/state/StateHelper.*`
  - 克隆增广、协方差扩展、旧克隆边缘化。
- `open_vins/ov_msckf/src/state/Propagator.cpp`
  - 15 状态 IMU 传播与 `F/G/Qd` 离散化。
- `open_vins/ov_msckf/src/update/UpdaterHelper.cpp`
  - 特征 Jacobian 零空间投影。
- `open_vins/ov_msckf/src/update/UpdaterMSCKF.cpp`
  - 特征筛选、残差堆叠、QR 测量压缩和 EKF 更新流程。

本仓库的相机模型使用真实 `OmniRadtan` 投影，而不是针孔模型；投影 Jacobian
通过对 `OmniRadtan::project()` 做中心差分得到，因此保留了全向鱼眼模型。

## 2. 功能

- 15 状态 IMU 核心：`[theta, p_wb, v_wb, bg, ba]`。
- 滑动窗口 pose clone：每个同步图像帧增广一个 `[theta, p]` 六维 clone。
- clone 边缘化：超过 `maximum_clones` 后边缘化最旧 clone。
- MSCKF 特征更新：
  - 从 `LandmarkTrack` 的跨相机观测构造全局 3D feature。
  - 使用真实 omni-radtan 投影和中心差分 Jacobian。
  - 对 `H_f` 做左零空间投影，消除 feature 位置依赖。
  - 堆叠所有特征，QR 测量压缩后执行 EKF 更新。
- 输出沿用 odometry、path、TF、landmark point cloud。

## 3. 实现方式

新增文件：

```text
include/sphere_vio/backend/msckf.hpp
src/backend/msckf.cpp
test/test_msckf.cpp
```

`Msckf` 使用动态协方差：

```text
covariance = [ current_imu 15x15 | clone_1 6x6 | clone_2 6x6 | ... ]
```

启用方式：

```bash
rosrun sphere_vio sphere_vio_feature_runner \
  --config /root/catkin_ws/src/sphere_vio/config/offline.yaml \
  --bag /data/sphere_algorithm_test.bag \
  --cross-camera-matching \
  --triangulation-candidates \
  --landmark-tracks \
  --msckf \
  --output-dir /output/msckf
```

需要 ROS 发布时再加 `--publish-ros`。

## 4. 当前边界

- 特征位置使用全局 3D 表示，不在协方差中显式增广地标状态；这是标准 MSCKF
  零空间投影做法。
- IMU 到相机外参仍固定为 `cameras.yaml` 标定值，不在线标定。
- 投影 Jacobian 使用中心差分，而不是手工解析导数；它调用真实 omni-radtan
  投影，因此不是针孔近似。
- 当前合成 bag 没有可靠跨相机匹配，MSCKF 更新路径只在结构上验证；真实四目
  数据到位后需要统计可更新 feature 数量和收敛情况。
