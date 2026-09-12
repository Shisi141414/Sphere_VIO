# Sphere-VIO ATE 调参与本机（Windows）测试指南

> 适用范围：**本机 Windows + Docker** 离线运行 rosbag，目标序列为首个门禁序列
> `eight_noyaw_3_short`。
>
> 目标：先把 `ATE_SE3` 从 `~1e4 m` 量级降下来。当前瓶颈是**尺度错误**，不是单纯
> 漂移。因此本文先讲清楚“如何判断问题”、再讲“Windows 上怎么跑”、最后给“按什么
> 顺序调参”。
>
> **本文不再涉及 Jetson / aarch64 / CUDA / TensorRT / ONNX-GPU**。SuperPoint、GPU
> 等属于后续增量，不属于本次 ATE 修复主线。

---

## 0. 先读懂问题：ATE_SE3 大 ≠ 一定在漂移

`result/SUMMARY.md` 里 `eight_noyaw_3_short` 的关键数字是：

| 指标 | 数值 | 含义 |
| --- | ---: | --- |
| `ATE_SE3` | 85400.17 m | 不做尺度估计、只做刚体（旋转+平移）对齐后的轨迹误差 |
| `ATE_Sim3` | 1.5668 m | 额外估计一个全局**尺度**后的轨迹误差 |
| `ATE_Sim3 scale` | ~1e-5 | 为对齐 GT 需要把估计轨迹放大的倍数 |

这三行放在一起，结论非常明确：

- **轨迹的“形状”是对的**：把估计轨迹做一次相似变换（旋转 + 平移 + 缩放）后，它和
  GT 的误差只有约 `1.57 m`。这说明视觉前端重建出来的相对几何结构基本正确。
- **但“米”这个单位错了**：`scale ~ 1e-5` 表示估计轨迹整体比真实轨迹缩小了约十万倍，
  需要乘 1e5 才能和 GT 对齐。

在视觉惯性里程计（VIO）里，**单目视觉本身是尺度不可观的**——它只能给出“形状”，
不知道真实尺寸。真实的“米”这个单位，是由 **IMU** 提供的：加速度计测量的是真实
物理量（`m/s²`），把它积分成位移后，尺度就被固定下来了。

所以 `scale ~ 1e-5` 几乎必然指向 **IMU 这一侧出了问题**，而不是“特征不够好”。最
常见的三个原因（按可能性排序）：

1. **相机与 IMU 时间没有对齐**。时间偏移错了，IMU 积分窗口和视觉观测对不上，滤波器
   在错误的时刻做传播和更新，尺度直接崩。
2. **重力方向 / 重力模长错误**。初始姿态靠“静止时加速度≈重力”来定，重力定错会污染
   位置积分。
3. **IMU 噪声 / 零偏参数不对**，导致积分出的位移被错误地压制或放大。

> 一个关键判据：**调参时始终盯住 `ATE_Sim3 scale`**。当它接近 `1.0` 时，`ATE_SE3`
> 会自动收敛到和 `ATE_Sim3` 同一个量级。反过来，如果 `ATE_Sim3` 也在快速变大，那才
> 说明出现了真正的漂移/发散，需要回到前端特征和后端门控去查。

---

## 1. Windows 本机环境说明

本项目是 ROS 1 Noetic 包，依赖 Linux 下的 Eigen / OpenCV / ROS。Windows **不能原生
编译**（没有这套工具链），所以本机测试统一走 **Docker**。

- 需要安装并启动 **Docker Desktop（WSL2 后端）**。
- 代码和数据集都在 Windows 磁盘上，运行时用 `-v` 挂载进容器即可，**不需要把数据拷
  进 Linux**。
- 只有两个东西要准备好：
  1. **仓库目录**（含代码、config、scripts）。
  2. **D2SLAM 数据集目录**（含 `.bag` 和 `-groundtruth.txt`）。

先用下面命令确认 Docker 可用：

```powershell
docker version
```

如果报 `error during connect` 或类似错误，说明 Docker Desktop 没启动，先把它打开，再
继续。

---

## 2. 一次性准备：构建镜像

在仓库根目录（`C:\HHQ_STUDY\RM_QY\Drone\oiv_and_slam\Sphere_VIO`）执行：

```powershell
docker build -t sphere_vio:noetic .
```

镜像基于 `ros:noetic-ros-base-focal`（Ubuntu 20.04），编译好 `sphere_vio` 包，并把
`docker/entrypoint.sh` 设为入口，容器启动时会自动 `source` ROS 和 catkin 环境。
注意：`catkin_make` 的 `devel` 空间不会把 `devel/lib/sphere_vio/` 加进 `PATH`，所以
**不能用裸的可执行文件名**（如 `sphere_vio_feature_runner`），必须用
`rosrun sphere_vio <可执行文件名>` 来启动（见第 3 节）。

### 关键区分：改 YAML 不用重编，改 C++ 才要重编

- **YAML 配置**（`config/*.yaml`）是在运行时通过 `-v "$repo:/repo"` 挂载读取的，改完
  直接重跑即可，**不需要重新 build 镜像**。
- **C++ 源码**（`src/*.cpp`、`include/*.hpp`）在 `docker build` 时被编译进镜像，改完
  必须重新执行上面的 `docker build`（或另做源码挂载 + 容器内 `catkin_make`）。

调 ATE 时，绝大多数旋钮都在 YAML 里，所以主要工作流是“改 YAML → 重跑”，效率很高。

---

## 3. 跑一条基线并评测

### 3.1 定义路径

在 PowerShell 里先定义两个变量（后面命令复用）：

```powershell
$repo = "C:\HHQ_STUDY\RM_QY\Drone\oiv_and_slam\Sphere_VIO"
$data = "C:\HHQ_STUDY\RM_QY\Drone\oiv_and_slam\D2dataset\quadcam_7inch_n3_2023_1_14"
```

注意 `$data` 挂载的是**父目录** `quadcam_7inch_n3_2023_1_14`，因为：

- `.bag` 在它的子目录里：`quadcam_7inch_n3_2023_1_14\eight_noyaw_3_short-sync.bag`
- `.groundtruth.txt` 在它的根下：`eight_noyaw_3_short-groundtruth.txt`

这样挂到容器里的 `/data` 后，两条路径分别是：

- bag：`/data/quadcam_7inch_n3_2023_1_14/eight_noyaw_3_short-sync.bag`
- GT：`/data/eight_noyaw_3_short-groundtruth.txt`

### 3.2 运行前端 + MSCKF

```powershell
$out = "msckf_eight_noyaw_3_short"

docker run --rm `
  -v "${repo}:/repo" `
  -v "${data}:/data" `
  sphere_vio:noetic `
  rosrun sphere_vio sphere_vio_feature_runner `
    --config /repo/config/offline.yaml `
    --frontend-config /repo/config/system.yaml `
    --cameras /repo/config/cameras_d2slam.yaml `
    --bag /data/quadcam_7inch_n3_2023_1_14/eight_noyaw_3_short-sync.bag `
    --msckf `
    --output-dir /repo/output/$out
```

说明：

- `--msckf` 会连带打开跨相机匹配、三角化候选、地标关联和 IMU/MSCKF 后端。
- 输出写在 `/repo/output/$out/` 下的 `odometry.csv`、`trajectory.csv`、
  `landmarks.csv`。
- 评测只用到 `trajectory.csv`（TUM 格式：`timestamp,x,y,z,qx,qy,qz,qw`）。

### 3.3 评测

```powershell
docker run --rm `
  -v "${repo}:/repo" `
  -v "${data}:/data" `
  sphere_vio:noetic `
  python3 /repo/scripts/evaluate_sphere_vio.py `
    --groundtruth /data/eight_noyaw_3_short-groundtruth.txt `
    --trajectory /repo/output/$out/trajectory.csv `
    --output-period 0.05 `
    --output-report /repo/result/report_$out.md
```

评测脚本会打印 `ATE_SE3`、`ATE_Sim3`、`ATE_Sim3 scale`、`RPE_*`、`coverage`、
`success` 等，并把结果写进 `/repo/result/report_$out.md`。

---

## 4. 读诊断输出

调参不是“改完看一个数”，而是先看诊断定位到哪一段链路断了。项目里有两条独立的
命令可以看不同层面的诊断。

### 4.1 IMU / 时钟诊断（先看这个）

用 `sphere_vio_bag_runner` 跑一遍 bag（只统计，不做前端/后端）：

```powershell
docker run --rm `
  -v "${repo}:/repo" `
  -v "${data}:/data" `
  sphere_vio:noetic `
  rosrun sphere_vio sphere_vio_bag_runner `
    --config /repo/config/offline.yaml `
    --bag /data/quadcam_7inch_n3_2023_1_14/eight_noyaw_3_short-sync.bag
```

它会输出 `Offline rosbag summary`，重点看这几行：

- `IMU message count`：是否接近“总时长 × IMU 频率”。D2SLAM 标称 IMU 约 400 Hz。
- `minimum / maximum / average IMU interval`：正常应约 `0.0025 s`；如果出现大量
  `abnormally large IMU interval` 或 `non-monotonic IMU count`，说明 IMU 时间轴本身
  有空洞/乱序。
- `minimum / maximum / average IMU measurements per image frame`：每帧图像之间应有
  足够的 IMU 样本；`frames without sufficient IMU` 应为 0（或极少）。
- `first / last completed frame timestamp`：和 GT 的时间范围核对，确认没截错段。

这步的结论直接决定调参方向：**如果 IMU 时间轴本身不正常，先修数据/对齐，再谈后端。**

### 4.2 前端 + MSCKF 诊断（跑 `sphere_vio_feature_runner` 时的 stdout）

`sphere_vio_feature_runner` 跑完后，stdout 末尾有几段关键信息：

- **前端诊断**：每个相机的检测/跟踪/描述子统计、跨相机匹配漏斗（raw → absolute →
  ratio → mutual → epipolar → final）、三角化候选统计、地标统计。用来判断视觉链路是否
  健康（匹配数、三角化数是否明显异常）。
- **`Sphere-VIO performance summary`**：`completed frames`、`RTF`、`dropped_frames`
  等，确认没有丢帧、能跑完整条序列。
- **`MSCKF update diagnostics`**：`considered_features` / `accepted_features` /
  `gate_rejected_features` / `landmark_count`。这是最直接的后端健康指标：
  - 如果 `accepted_features` 长期为 0，说明特征要么没进后端、要么全被卡方门控拒掉，
    尺度必然无约束。
  - `gate_rejected` 过高，往往是时间对齐或像素噪声/门控概率设得不合理。

建议把每次运行的 stdout 重定向保存，方便回溯。在 PowerShell 里可以这样追加记录：

```powershell
docker run --rm -v "${repo}:/repo" -v "${data}:/data" sphere_vio:noetic `
  rosrun sphere_vio sphere_vio_feature_runner --config /repo/config/offline.yaml `
  --frontend-config /repo/config/system.yaml `
  --cameras /repo/config/cameras_d2slam.yaml `
  --bag /data/quadcam_7inch_n3_2023_1_14/eight_noyaw_3_short-sync.bag `
  --msckf --output-dir /repo/output/$out *> "$repo/result/run_$out.log"
```

> PowerShell 里 `*>` 会把 stdout 和 stderr 一起写进文件（注意它和 Bash 的 `2>&1` 写法
> 不同）。

---

## 5. 调参顺序（按对“尺度”的影响从大到小）

### 5.1 第一步：时间偏移（最可能的主因，优先扫）

D2SLAM 的官方配置里给出了这组数据的权威时间偏移：

- 文件：`config/d2slam/quadcam_single.yaml`
- 字段：`td: -0.186`
- 语义：**相机时间戳 + td = IMU 时间**，即 `t_imu = t_camera - 0.186 s`。
  也就是说 IMU 时间戳比相机时间戳**早约 0.186 秒**。

这个偏移现在已经接通到 runner 里，位置在 `config/offline.yaml` 的 `synchronization` 下：

```yaml
synchronization:
  camera_to_imu_offset_s: -0.186   # t_imu = t_camera + 该值
```

代码会在 IMU 进入后端前把它换算到相机时间基准
（`imu_to_camera_shift = -camera_to_imu_offset_s = +0.186`），让 IMU 积分窗口、clone、
视觉观测、轨迹输出都处在同一个时钟里。原来 clone≥8 后那次 ±25 ms 的在线时间偏移/外参
搜索（`estimateTimeOffset` / `estimateExtrinsicPerturbation`）也已经去掉，不会再覆盖
这个固定值。

#### 现在要做的：扫描这个固定值

把 `camera_to_imu_offset_s` 当作唯一的时间旋钮，从 `-0.30` 扫到 `-0.05`
（粗扫步长 `0.02`，定位区间后再用 `0.005` 细扫），每次看 `ATE_Sim3 scale` 是否向
`1.0` 靠拢。先用 `-0.186` 跑一版作为参考点。

> 这个字段在 `config/offline.yaml` 里，改完**无需重新 build 镜像**，直接重跑即可
> （见第 2 节“改 YAML 不用重编”）。

#### 判断标准

- `ATE_Sim3 scale` 越接近 `1.0`，说明时间对齐越接近正确。
- 同时 `MSCKF update diagnostics` 里 `accepted_features` 应该明显上升，
  `gate_rejected_features` 下降。

### 5.2 第二步：重力与 IMU 噪声

时间对齐方向确认后，再核对 IMU 模型参数。`config/system.yaml` 的 `backend.msckf` 与
D2SLAM 参考值（`config/d2slam/quadcam_single.yaml`）对比如下：

| 本项目字段（system.yaml） | 当前值 | D2SLAM 参考字段 | D2SLAM 参考值 | 是否一致 |
| --- | ---: | --- | ---: | --- |
| `gravity_magnitude` | 9.805 | `g_norm` | 9.805 | 一致 |
| `accelerometer_noise` | 1.0e-1 | `acc_n` | 0.1 | 一致 |
| `gyroscope_noise` | 1.0e-2 | `gyr_n` | 0.05 | **不一致（差 5 倍）** |
| `accelerometer_bias_noise` | 1.0e-3 | `acc_w` | 0.002 | **不一致（差 2 倍）** |
| `gyroscope_bias_noise` | 1.0e-4 | `gyr_w` | 0.0004 | **不一致（差 4 倍）** |

建议优先把上面不一致的三项对齐到 D2SLAM 参考值试一版：

```yaml
backend:
  msckf:
    gyroscope_noise: 5.0e-2
    accelerometer_bias_noise: 2.0e-3
    gyroscope_bias_noise: 4.0e-4
```

噪声项的意义（便于理解调参方向）：

- `*_noise` 是**测量噪声**标准差，越大表示越不信任该测量，滤波器会更依赖视觉。
- `*_bias_noise` 是**零偏随机游走**，越大表示零偏被允许变化得越快。

尺度问题是“加速度计积分出的位移被错误缩放”，所以 `accelerometer_noise` /
`accelerometer_bias_noise` 和 `gravity_magnitude` 对尺度最敏感，优先动这几个。

### 5.3 第三步：MSCKF 窗口与门控

时间、重力、噪声稳定后，再微调后端：

- `maximum_clones`（当前 20）：滑动窗口里的 clone 数量，越大越能保留更长的特征观测
  弧段，代价是算力和数值稳定性。
- `feature_chi_square_probability`（当前 0.99）：卡方门控的置信概率。太严会拒掉大量
  有效特征（`gate_rejected` 飙升），太松会放过坏特征。
- `pixel_noise`（当前 0.5）：像素测量噪声标准差。若视觉残差普遍偏大，可以适当放宽。
- `initialization_duration` / `minimum_initialization_samples`：静止初始化窗口长度，
  决定初始重力/零偏估计是否充分。

这些也可以用命令行直接覆盖，方便快速扫描，不用改 YAML：

```powershell
docker run --rm -v "${repo}:/repo" -v "${data}:/data" sphere_vio:noetic `
  rosrun sphere_vio sphere_vio_feature_runner --config /repo/config/offline.yaml `
  --frontend-config /repo/config/system.yaml `
  --cameras /repo/config/cameras_d2slam.yaml `
  --bag /data/quadcam_7inch_n3_2023_1_14/eight_noyaw_3_short-sync.bag `
  --msckf `
  --msckf-max-clones 20 `
  --msckf-pixel-noise 0.5 `
  --msckf-chi-square-probability 0.99 `
  --gravity-magnitude 9.805 `
  --output-dir /repo/output/$out
```

### 5.4 第四步：前端特征（放到最后）

`config/system.yaml` 的 `frontend` 下是 FAST/LK/ORB/匹配/三角化参数。它们影响视觉
几何质量，但对“尺度=1e-5”这种量级错误作用次之，应放在前几步都无效之后再动：

- `maximum_features_per_camera`、`fast_threshold`：特征数量与响应阈值。
- `lk_window_size`、`pyramid_levels`、`maximum_forward_backward_error`：LK 跟踪质量。
- `cross_camera.*`：跨相机匹配门控。
- `triangulation_candidates.*`：三角化候选的几何门控。

### 5.5 输出门控保持不变

评测用 `--output-period 0.05`，与 runner 里 50 ms 插值输出的轨迹一致。不要改成别的
值，否则 `max_pose_interval` 的阈值（`2 * output_period`）会对不上，导致 `success`
被误判为失败，而实际算法没坏。

---

## 6. 每轮实验记录模板

调参一定要留痕，否则很快会忘记“某组数字对应哪次改动”。建议每一轮记一张表：

| 实验号 | 时间偏移 | 重力 | 噪声改动 | 其他改动 | ATE_SE3 | ATE_Sim3 | scale | RPE_10m | coverage | success | 备注 |
| --- | ---: | ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| 01（基线） | 0.0 | 9.805 | 默认 | 无 | 85400 | 1.57 | 1e-5 | 50.9 | 0.9998 | False | 尺度错误 |
| 02 | -0.186 | 9.805 | 默认 | 接 td | ? | ? | ? | ? | ? | ? | 待测 |
| 03 | -0.20 | 9.805 | 默认 | 关在线搜索 | ? | ? | ? | ? | ? | ? | 待测 |

补充记录：stdout 日志文件路径、`MSCKF update diagnostics` 的
`accepted/rejected`、以及 `IMU message count` 等关键诊断，方便对比。

---

## 7. PowerShell 常见坑

1. **路径要加双引号**：`$repo`、`$data` 含中文/空格，必须写成 `"${repo}:/repo"`，
   不要写成 `${repo}:/repo` 或单引号包裹（单引号在 PS 里是字面量，不会展开变量）。
2. **反引号是续行符**：`` ` `` 后面不能有任何空格，否则命令会断错。
3. **区分 PS 变量与容器内字符串**：`$repo`、`$data`、`$out` 是 **PowerShell** 变量，
   在 `docker run` 之前就会被展开成真实 Windows 路径，这是预期行为。
4. **改 C++ 后必须重编**：只改 YAML 不用重编，但改了 `src/` 或 `include/` 后一定要
   重新 `docker build -t sphere_vio:noetic .`，否则跑的还是旧二进制。
5. **日志重定向**：PowerShell 用 `*> file` 合并 stdout/stderr，不要照抄 Bash 的
   `2>&1`（那在 PS 里语义不同）。
6. **不确定就写 `.ps1` 脚本**：如果复制粘贴多行命令反复报错，把命令保存成 `run.ps1`
   再执行，避免手工转义问题。

---

## 8. 常见故障排查

- **Docker 连不上**：启动 Docker Desktop，等右下角图标变成“运行中”再重试。
- **改 YAML 没生效**：确认改的是 runner 实际读的 `config/system.yaml`
  （`--frontend-config` 指向它），而不是 D2SLAM 的 `config/d2slam/quadcam_single.yaml`
  （那个目前只是参考值，runner 不直接读）。
- **找不到 bag**：确认 `$data` 挂载的是父目录，bag 路径是
  `/data/quadcam_7inch_n3_2023_1_14/eight_noyaw_3_short-sync.bag`。
- **评测报 `No module named 'numpy'`**：镜像可能没装 numpy。可以在 Dockerfile 的
  `apt-get install` 里加 `python3-numpy`，或在一次性容器里 `pip3 install numpy` 后重试。
- **trajectory.csv 是空/几乎空**：回到第 4 节看 `MSCKF update diagnostics`；如果
  `accepted_features = 0`，先查时间偏移（5.1）和 IMU 诊断（4.1）。
- **`success=False` 但 `has_nan_inf=False`、`coverage` 也高**：多半是
  `max_pose_interval` 超阈值，检查是否用了错误的 `--output-period`（应固定 0.05）。

---

## 9. 小结（一条最小可行的闭环）

1. `docker build -t sphere_vio:noetic .`
2. 跑 `sphere_vio_bag_runner` 看 IMU 时间轴是否正常。
3. 跑 `sphere_vio_feature_runner --msckf` 得到基线 `trajectory.csv`。
4. `evaluate_sphere_vio.py` 看 `ATE_Sim3 scale`，确认是尺度错误。
5. 扫描时间偏移 `synchronization.camera_to_imu_offset_s`（以 `-0.186` 为起点），
   目标是 `scale → 1.0`。
6. 再对齐 IMU 噪声/重力，最后微调 MSCKF 窗口与门控。
7. 每轮记录一张表，盯住 `ATE_Sim3 scale` 和 `MSCKF accepted/gate_rejected`。
