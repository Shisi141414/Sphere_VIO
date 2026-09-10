# Sphere-VIO

Sphere-VIO 是一个面向四目鱼眼相机 + IMU 的视觉惯性里程计研究实现，运行环境为
ROS 1 Noetic / Ubuntu 20.04，主要后端是滑动窗口 MSCKF。

当前代码已具备：

- D2SLAM 四合一压缩 rosbag 的直接流式读取，无需中间转换。
- 四路同步图像帧装配。
- FAST 网格特征检测与金字塔 LK 时序跟踪。
- ORB 描述子与跨相机匹配。
- 当前帧两视图三角化候选与 LandmarkTrack 关联。
- IMU 初始化：重力方向、陀螺零偏、加速度计零偏估计。
- 滑窗 MSCKF：clone 增广、边缘化、特征零空间投影更新。
- 持久 landmark 协方差状态。
- 相机/IMU 时间偏移与外参扰动的在线估计。
- `trajectory.csv` 规范轨迹输出与评测脚本。

## 目录结构

```text
config/       标定、离线 runner、MSCKF 前端/后端参数
docs/         使用说明、数据集说明、评测指标、实现指南
include/      头文件
src/          源码
scripts/      数据转换、评测、可视化脚本
test/         gtest 单元测试
launch/       ROS launch 文件
data/         本地 smoke bag 和转换后 bag，不进入 Git
output/       运行结果 CSV、日志和全景图
result/       全量评测报告、改进日志和汇总表
```

## 环境

- Ubuntu 20.04
- ROS Noetic
- OpenCV 4.2
- Eigen 3.3
- yaml-cpp
- CMake / catkin

推荐直接使用 Docker 镜像：

```powershell
docker build -t sphere_vio:noetic .
```

## 快速开始

容器内运行：

```bash
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

rosrun sphere_vio sphere_vio_feature_runner \
  --config /repo/config/offline.yaml \
  --frontend-config /repo/config/system.yaml \
  --cameras /repo/config/cameras_d2slam.yaml \
  --bag /data/quadcam_7inch_n3_2023_1_14/eight_noyaw_3_short-sync.bag \
  --msckf \
  --output-dir /repo/output/msckf_eight_noyaw_3_short
```

Windows PowerShell 中挂载目录运行：

```powershell
$repo = "~\Sphere_VIO"
$data = "~\quadcam_7inch_n3_2023_1_14"     # 替换为真实库路径和数据集路径

docker run --rm -v "$repo:/repo" -v "$data:/data" `
  sphere_vio:noetic bash -lc `
  "source /opt/ros/noetic/setup.bash; source /root/catkin_ws/devel/setup.bash; `
   rosrun sphere_vio sphere_vio_feature_runner `
     --config /repo/config/offline.yaml `
     --frontend-config /repo/config/system.yaml `
     --cameras /repo/config/cameras_d2slam.yaml `
     --bag /data/quadcam_7inch_n3_2023_1_14/eight_noyaw_3_short-sync.bag `
     --msckf `
     --output-dir /repo/output/msckf_eight_noyaw_3_short"
```

## 全量数据集

完整数据集目录结构、文件说明和运行评测方法见：

- [docs/FULL_DATASET_ZH.md](docs/FULL_DATASET_ZH.md)
- [docs/USAGE_ZH.md](docs/USAGE_ZH.md)

## 评测

```powershell
docker run --rm -v "$repo:/repo" -v "$data:/data" `
  sphere_vio:noetic python3 `
  /repo/scripts/evaluate_sphere_vio.py `
    --groundtruth /data/eight_noyaw_3_short-groundtruth.txt `
    --trajectory /repo/output/msckf_eight_noyaw_3_short/trajectory.csv `
    --output-period 0.05 `
    --output-report /repo/result/report_eight_noyaw_3_short.md
```

轨迹可视化：

```powershell
docker run --rm -it `
  -e DISPLAY=host.docker.internal:0.0 `
  -v "$repo:/repo" -v "$data:/data" `
  sphere_vio:noetic python3 `
  /repo/scripts/visualize_trajectory.py `
    --groundtruth /data/eight_noyaw_3_short-groundtruth.txt `
    --trajectory /repo/output/msckf_eight_noyaw_3_short/trajectory.csv
```

## 测试

```bash
catkin_make run_tests -DCATKIN_WHITELIST_PACKAGES=sphere_vio
```

## 文档

- 使用与运行：[docs/USAGE_ZH.md](docs/USAGE_ZH.md)
- 全量数据集：[docs/FULL_DATASET_ZH.md](docs/FULL_DATASET_ZH.md)
- 评测指标：[docs/INDEX_LIST.md](docs/INDEX_LIST.md)
- MSCKF 说明：[docs/MSCKF_ZH.md](docs/MSCKF_ZH.md)
- ESKF 说明：[docs/BACKEND_ESKF_ZH.md](docs/BACKEND_ESKF_ZH.md)
- 改进日志：[result/IMPROVEMENT_LOG.md](result/IMPROVEMENT_LOG.md)

## 结果说明

全量序列测试已完成，具体指标和剩余风险见：

- [result/SUMMARY.md](result/SUMMARY.md)
- [result/IMPROVEMENT_LOG.md](result/IMPROVEMENT_LOG.md)

当前 MSCKF 在 `ATE_Sim3` 下约为 1.55m，但绝对尺度 `ATE_SE3` 仍明显发散，
说明视觉惯性里程计的可行性链路已打通，但还需后续标定与后端优化才能达到部署级
精度。
