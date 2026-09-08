# D2SLAM 数据集接入说明

## 1. 数据目录

```text
C:\HHQ_STUDY\RM_QY\Drone\oiv_and_slam\D2dataset\manual_quadcam_7inch_n3_2022_10_26
```

主要内容：

```text
d2vins*.yaml / single.yaml        D2VINS 实验编排
Configs/                          本下载中为空，需另取标定
manual_calib*.bag                 相机/IMU 标定数据
manual_small_*                    同步飞行序列与 groundtruth
manual_vo_test*                   VO 测试序列
vins-mono_*.csv                   已有 VINS-Mono 结果
```

## 2. 原始 bag 结构

完整 `-sync.bag`：

```text
/arducam/image/compressed   5120x800 四合一 JPEG
/dji_sdk_1/dji_sdk/imu     IMU
/SwarmNode1/pose           PoseStamped
/calib_path                Path
/calib_pose                PoseStamped
```

`-sync-split.bag` 中图像话题是 `/arducam/image_3/compressed`，单路 `1280x800`。

## 3. 转换到 Sphere-VIO

第一步，切分四合一图：

```bash
python3 scripts/convert_d2slam_quad.py \
  --input /data/manual_small_1-sync.bag \
  --output /output/manual_small_1_sphere.bag \
  --max-frames 20
```

第二步，下载 D2SLAM 标定并转换：

```text
https://github.com/HKUST-Aerial-Robotics/D2SLAM
config/quadcam/quad_cam_calib-camchain-imucam-7-inch-n3.yaml
```

```bash
python3 scripts/convert_d2slam_calib.py \
  --input config/d2slam/quad_cam_calib-camchain-imucam-7-inch-n3.yaml \
  --output config/cameras_d2slam.yaml
```

当前工作区已经生成：

```text
config/cameras_d2slam.yaml
config/d2slam/*.yaml
```

## 4. 运行

```bash
rosrun sphere_vio sphere_vio_feature_runner \
  --config config/offline.yaml \
  --bag /data/manual_small_1_sphere.bag \
  --cameras config/cameras_d2slam.yaml \
  --cross-camera-matching \
  --triangulation-candidates \
  --landmark-tracks \
  --msckf \
  --output-dir /output/msckf_d2slam
```

注意：D2SLAM 的 `T_cam_imu` 方向与 Sphere-VIO loader 相反，转换脚本会显式
取逆，因此不要直接复制 D2SLAM YAML 当作 `cameras.yaml`。

## 5. 已完成的完整序列验证

对 `manual_small_1-sync.bag` 的 2358 帧完整序列，当前结果如下：

```text
completed four-camera frames: 2358
dropped images: 0
timestamp mismatches: 0

global triangulation candidates:
  input                   18098
  triangulation successes 17572
  admitted                16254

LandmarkTrack:
  created tracks             162
  tracks ever active         140
  final active               71
  association conflicts      3433
```

MSCKF 后端已生成 `odometry.csv` 和 `landmarks.csv`。用
`scripts/evaluate_d2slam.py` 与 groundtruth 对齐后：

```text
matched poses: 2328
ATE translation RMSE: 5208.22 m
ATE translation mean: 4355.82 m
ATE translation max: 13722.13 m
relative translation RMSE @1s: 166.07 m
```

该结果表明前端几何链路已经工作，但当前 MSCKF 后端在完整序列上仍然明显发散，
不能作为可靠 VIO 结果。下一步应优先检查 IMU 时间戳对齐、IMU 噪声/零偏初始值、
图像与 IMU 时间偏移，以及 MSCKF 视觉更新中的外点剔除和关联冲突。
