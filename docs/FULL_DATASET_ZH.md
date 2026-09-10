# 全量数据集说明

本文描述用于 Sphere-VIO 全量评测的 D2SLAM 四合一压缩图数据集。当前离线 runner
已经支持直接读取原始 `-sync.bag`，无需先运行 `convert_d2slam_quad.py` 生成中间
Sphere-VIO bag。

## 1. 数据位置

```text
C:\HHQ_STUDY\RM_QY\Drone\oiv_and_slam\D2dataset\quadcam_7inch_n3_2023_1_14
```

建议把该目录挂载到容器内的 `/data`：

```powershell
-v "C:\HHQ_STUDY\RM_QY\Drone\oiv_and_slam\D2dataset\quadcam_7inch_n3_2023_1_14:/data"
```

目录结构：

```text
quadcam_7inch_n3_2023_1_14/
  Configs/
    quadcam/
      mask.png
  outputs/
    d2vins-5-sync/
      swarm1/
  quadcam_7inch_n3_2023_1_14/
    eight_noyaw_1-sync.bag
    eight_noyaw_1-groundtruth.txt
    eight_noyaw_3_short-sync.bag
    eight_noyaw_3_short-groundtruth.txt
    eight_noyaw_4-sync.bag
    eight_noyaw_5-sync.bag
    eight_yaw_1-sync.bag
    eight_yaw_1-sync.orig.bag
  eight_noyaw_1-groundtruth.txt
  eight_noyaw_2-groundtruth.txt
  eight_noyaw_3_short-groundtruth.txt
  eight_noyaw_4-groundtruth.txt
  eight_noyaw_5-groundtruth.txt
  eight_noyaw_6-groundtruth.txt
  eight_noyaw_7-groundtruth.txt
  eight_yaw_1-groundtruth.txt
  eight_yaw_2-groundtruth.txt
  eight_yaw_3-groundtruth.txt
  eight_yaw_4-groundtruth.txt
  eight_yaw_5-groundtruth.txt
  eight_yaw_6-groundtruth.txt
  eight_yaw_7-groundtruth.txt
  vins-mono_noyaw_1.csv
  vins-mono_noyaw_2.csv
  vins-mono_noyaw_5.csv
  vins-mono_noyaw_6.csv
  vins-mono_noyaw_7.csv
  vins-mono_yaw_1.csv
  vins-mono_yaw_2.csv
  vins-mono_yaw_3.csv
  vins-mono_yaw_4.csv
  vins-mono_yaw_5.csv
```

### 1.1 根目录 GT 文件

根目录共包含 14 个 groundtruth 文件，覆盖 7 个无 yaw 序列和 7 个含 yaw 序列：

| 文件 | 功能与特点 |
| --- | --- |
| `eight_noyaw_1-groundtruth.txt` | `eight_noyaw_1` 的 GT，格式 `timestamp x y z qx qy qz qw` |
| `eight_noyaw_2-groundtruth.txt` | `eight_noyaw_2` 的 GT，当前目录未提供对应 bag |
| `eight_noyaw_3_short-groundtruth.txt` | `eight_noyaw_3_short` 的 GT |
| `eight_noyaw_4-groundtruth.txt` | `eight_noyaw_4` 的 GT |
| `eight_noyaw_5-groundtruth.txt` | `eight_noyaw_5` 的 GT |
| `eight_noyaw_6-groundtruth.txt` | `eight_noyaw_6` 的 GT，当前目录未提供对应 bag |
| `eight_noyaw_7-groundtruth.txt` | `eight_noyaw_7` 的 GT，当前目录未提供对应 bag |
| `eight_yaw_1-groundtruth.txt` | `eight_yaw_1` 的 GT |
| `eight_yaw_2-groundtruth.txt` | `eight_yaw_2` 的 GT，当前目录未提供对应 bag |
| `eight_yaw_3-groundtruth.txt` | `eight_yaw_3` 的 GT，当前目录未提供对应 bag |
| `eight_yaw_4-groundtruth.txt` | `eight_yaw_4` 的 GT，当前目录未提供对应 bag |
| `eight_yaw_5-groundtruth.txt` | `eight_yaw_5` 的 GT，当前目录未提供对应 bag |
| `eight_yaw_6-groundtruth.txt` | `eight_yaw_6` 的 GT，当前目录未提供对应 bag |
| `eight_yaw_7-groundtruth.txt` | `eight_yaw_7` 的 GT，当前目录未提供对应 bag |

每行包含 8 个浮点数，分别表示时间戳、位置 x/y/z 和单位四元数 qx/qy/qz/qw。
时间戳为 ROS Unix 秒，评测时脚本会将其插值到估计轨迹时间戳。

### 1.2 子目录 `quadcam_7inch_n3_2023_1_14/`

该目录内是目前实际可运行的全量 bag：

| 文件 | 功能与特点 |
| --- | --- |
| `eight_noyaw_1-sync.bag` | 无 yaw 8 字轨迹，约 548s、10954 张四合一图像、219456 条 IMU |
| `eight_noyaw_3_short-sync.bag` | 短版无 yaw 序列，约 401s、7922 张四合一图像、160698 条 IMU |
| `eight_noyaw_4-sync.bag` | 无 yaw 8 字轨迹，约 602s、12039 张四合一图像、241013 条 IMU |
| `eight_noyaw_5-sync.bag` | 无 yaw 8 字轨迹，约 604s、11827 张四合一图像、241792 条 IMU |
| `eight_yaw_1-sync.bag` | 含 yaw 机动序列，首次使用前需 reindex |
| `eight_yaw_1-sync.orig.bag` | `rosbag reindex` 时生成的原始备份，正常测试无需读取 |
| `eight_noyaw_1-groundtruth.txt` | 子目录内保留的同名 GT 副本 |
| `eight_noyaw_3_short-groundtruth.txt` | 子目录内保留的同名 GT 副本 |

注意：评测脚本使用的 GT 路径是 `/data/eight_<seq>-groundtruth.txt`，也就是数据集
根目录的文件；子目录内的副本不是默认输入。

### 1.3 `Configs/quadcam/`

| 文件 | 功能与特点 |
| --- | --- |
| `mask.png` | D2SLAM 四目鱼眼相机的公共遮罩图，可用于排除机身、桨叶等无效区域；当前 Sphere-VIO 全量评测未强制使用 |

### 1.4 `outputs/d2vins-5-sync/swarm1/`

该目录是 D2VINS/D2SLAM 原始输出目录的占位示例，不属于 Sphere-VIO 输入或输出。
运行 Sphere-VIO 时应把结果写到仓库的 `output/`，不要覆盖该目录。

### 1.5 根目录 `vins-mono_*.csv`

根目录提供部分 VINS-Mono 参考结果，可用于粗粒度对比：

| 文件 | 对应序列 |
| --- | --- |
| `vins-mono_noyaw_1.csv` | `eight_noyaw_1` |
| `vins-mono_noyaw_2.csv` | `eight_noyaw_2` |
| `vins-mono_noyaw_5.csv` | `eight_noyaw_5` |
| `vins-mono_noyaw_6.csv` | `eight_noyaw_6` |
| `vins-mono_noyaw_7.csv` | `eight_noyaw_7` |
| `vins-mono_yaw_1.csv` | `eight_yaw_1` |
| `vins-mono_yaw_2.csv` | `eight_yaw_2` |
| `vins-mono_yaw_3.csv` | `eight_yaw_3` |
| `vins-mono_yaw_4.csv` | `eight_yaw_4` |
| `vins-mono_yaw_5.csv` | `eight_yaw_5` |

这些文件不是全量评测必需输入，只用于算法结果对照。

## 2. 测试序列

| 序列 | bag | GT | 说明 |
| --- | --- | --- | --- |
| `eight_noyaw_1` | `quadcam_7inch_n3_2023_1_14/eight_noyaw_1-sync.bag` | `eight_noyaw_1-groundtruth.txt` | 8 字轨迹，无 yaw 机动 |
| `eight_noyaw_3_short` | `quadcam_7inch_n3_2023_1_14/eight_noyaw_3_short-sync.bag` | `eight_noyaw_3_short-groundtruth.txt` | 短序列 |
| `eight_noyaw_4` | `quadcam_7inch_n3_2023_1_14/eight_noyaw_4-sync.bag` | `eight_noyaw_4-groundtruth.txt` | 8 字轨迹，无 yaw 机动 |
| `eight_noyaw_5` | `quadcam_7inch_n3_2023_1_14/eight_noyaw_5-sync.bag` | `eight_noyaw_5-groundtruth.txt` | 8 字轨迹，无 yaw 机动 |
| `eight_yaw_1` | `quadcam_7inch_n3_2023_1_14/eight_yaw_1-sync.bag` | `eight_yaw_1-groundtruth.txt` | 包含 yaw 机动 |

已从 rosbag 观测到的部分序列规模：

| 序列 | 数据时长 | 四合一图像消息数 | IMU 消息数 |
| --- | ---: | ---: | ---: |
| `eight_noyaw_1` | 约 548s | 10954 | 219456 |
| `eight_noyaw_3_short` | 约 401s | 7922 | 160698 |
| `eight_noyaw_4` | 约 602s | 12039 | 241013 |
| `eight_noyaw_5` | 约 604s | 11827 | 241792 |

`eight_yaw_1-sync.bag` 原始文件可能未建立索引，首次使用前需执行：

```powershell
docker run --rm -v `
  "C:\HHQ_STUDY\RM_QY\Drone\oiv_and_slam\D2dataset\quadcam_7inch_n3_2023_1_14:/data" `
  sphere_vio:noetic bash -lc `
  "source /opt/ros/noetic/setup.bash; rosbag reindex /data/quadcam_7inch_n3_2023_1_14/eight_yaw_1-sync.bag"
```

## 3. 原始 bag 话题

```text
/arducam/image/compressed   5120x800 四合一 JPEG
/dji_sdk_1/dji_sdk/imu      DJI IMU，约 400Hz
/SwarmNode1/pose            数据集内 GT PoseStamped
```

离线 runner 会把 `/arducam/image/compressed` 解码并按宽四等分，生成四路同步
`mono8` 图像：

```text
1280x800 C0
1280x800 C1
1280x800 C2
1280x800 C3
```

配置位于：

```text
config/offline.yaml
config/system.yaml
config/cameras_d2slam.yaml
```

`config/offline.yaml` 已配置：

```yaml
topics:
  d2slam_stitched_image: /arducam/image/compressed
  d2slam_imu: /dji_sdk_1/dji_sdk/imu
```

## 4. 相机标定

使用 D2SLAM 7-inch-n3 四目标定转换后的：

```text
config/cameras_d2slam.yaml
```

该文件由 D2SLAM 的 `quad_cam_calib-camchain-imucam-7-inch-n3.yaml` 转换而来。
标定模型为 omni + radtan，图像分辨率为 `1280x800`。

## 5. 运行方式

单序列示例：

```powershell
$repo = "C:\HHQ_STUDY\RM_QY\Drone\oiv_and_slam\Sphere_VIO"
$data = "C:\HHQ_STUDY\RM_QY\Drone\oiv_and_slam\D2dataset\quadcam_7inch_n3_2023_1_14"

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

批量运行全部序列的完整脚本见：

```text
docs/USAGE_ZH.md -> 11. D2SLAM 全量数据集运行方法
```

## 6. 评测

GT 文件格式：

```text
timestamp x y z qx qy qz qw
```

估计轨迹使用 `output/msckf_<seq>/trajectory.csv`，格式与 GT 一致：

```text
timestamp,x,y,z,qx,qy,qz,qw
```

评测命令：

```powershell
docker run --rm -v "$repo:/repo" -v "$data:/data" `
  sphere_vio:noetic python3 `
  /repo/scripts/evaluate_sphere_vio.py `
    --groundtruth /data/eight_noyaw_3_short-groundtruth.txt `
    --trajectory /repo/output/msckf_eight_noyaw_3_short/trajectory.csv `
    --output-period 0.05 `
    --output-report /repo/result/report_eight_noyaw_3_short.md
```

指标定义见：

```text
docs/INDEX_LIST.md
```
