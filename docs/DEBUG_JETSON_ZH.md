# Jetson Orin NX 远程调试指南

本指南用于把 Sphere-VIO 从 Windows 开发机搬到 NVIDIA Jetson Orin NX 上编译、
跑单测、跑 `eight_noyaw_3_short` 门禁序列，并解释每一步「为什么这样做」。

## 0. 先明确目标与顺序

本轮的修复目标按优先级是：

1. **门禁一（当前重点）**：时间轴修复 + 原始 FAST/ORB，跑通
   `eight_noyaw_3_short`，目标是 `ATE_SE3 < 100 m`、`RPE_10m < 20 m`、
   coverage ≥ 90%、全程无 NaN。这一步**只需要 CPU**，先别碰 GPU。
2. 门禁二：`pipeline_mode: rectified` 的 FAST/ORB。
3. 门禁三（可选）：SuperPoint CUDA。它需要 aarch64 版 ONNX Runtime GPU 和
   一个 D2SLAM 兼容的 `.onnx` 模型，本仓库**没有**附带模型。

所以调试顺序一定是「先环境 → 先编译单测 → 先 CPU 基线」，SuperPoint 放在最后。

## 1. 预检：确认 Jetson 的系统版本

Jetson Orin NX 是 **ARM64（aarch64）**，跑的是 JetPack。本项目用 ROS 1 Noetic，
它只支持 **Ubuntu 20.04**，也就是 **JetPack 5.x（L4T R35.x）**。

SSH 上去后先看四项：

```bash
# 1) 发行版（重点看是不是 Ubuntu 20.04）
lsb_release -a

# 2) JetPack / L4T 版本
cat /etc/nv_tegra_release

# 3) 架构（应该是 aarch64，不是 x86_64）
uname -m

# 4) CUDA 版本（JetPack 5 通常是 11.4，JetPack 6 是 12.x）
nvcc --version
```

判读：

- `Ubuntu 20.04 + R35.x`：可以原生安装 ROS Noetic，直接看第 3 节。
- `Ubuntu 22.04/24.04 + R36/R39`（JetPack 6）：没有 ROS Noetic 的 apt 包。CPU
  部分用第 1.1 节的 aarch64 容器调试，GPU 部分要另想办法。

> 注意：仓库里 `docker/Dockerfile.cuda` 是给 **x86 的 CUDA 11.8** 写的，在
> Jetson 上**不要用**。Jetson 走原生编译 + aarch64 ONNX Runtime（见第 7 节）。

## 1.1 你的机器是 JetPack 6 / Ubuntu 24.04

你贴出的信息是 `Ubuntu 24.04.4 + L4T R39 + aarch64`，也就是 **JetPack 6.x**。
结论很明确：**不能原生装 ROS Noetic**（Noetic 只支持 Ubuntu 20.04），而且
`nvcc` 没找到说明 CUDA 编译工具链也没装（GPU 部分后面再说）。

所以门禁一/二的正确路径是：在 Jetson 上用 **aarch64 版 ROS Noetic 容器**编译和
跑。CPU 计算不需要 GPU，容器里是 Ubuntu 20.04 用户态、复用 24.04 的内核，完全
可用。

先确认 Docker 可用：

```bash
docker --version
docker run --rm hello-world
```

如果没装 Docker，装一下（JetPack 6 一般自带，或执行）：

```bash
sudo apt update && sudo apt install -y docker.io
sudo usermod -aG docker "$USER"
# 重新登录让用户组生效
```

仓库里已经给你写好了 aarch64 镜像文件
[docker/Dockerfile.jetson](../docker/Dockerfile.jetson)，它就是标准
`Dockerfile` 换了个 `arm64v8/ros:noetic-ros-base-focal` 基础镜像，依赖完全一样。

把代码传上去后（见第 2 节），在 Jetson 上构建：

```bash
cd ~/sphere_vio
docker build -f docker/Dockerfile.jetson -t sphere_vio:jetson .
```

构建完先跑单测（`--entrypoint` 绕过默认入口直接给命令）：

```bash
docker run --rm -v "$HOME/sphere_vio:/repo" \
  --entrypoint bash sphere_vio:jetson -lc \
  "source /opt/ros/noetic/setup.bash; source /root/catkin_ws/devel/setup.bash; \
   cd /root/catkin_ws; \
   catkin_make run_tests -DCATKIN_WHITELIST_PACKAGES=sphere_vio && \
   catkin_test_results"
```

> 说明：`docker build` 会把当前目录 `COPY` 进镜像再编译。改代码后要重新
> `docker build`；调试时更快的做法是只挂载目录、在容器里手动 `catkin_make`，
> 见第 6 节。第 3 节的「原生 apt 安装 ROS」只适用于 JetPack 5，你的机器跳过。

同时确认磁盘和内存够用（bag 文件可能有几个 GB）：

```bash
df -h ~
free -h
nvidia-smi   # 或 tegrastats，看 GPU/显存
```

## 2. 把代码搬到 Jetson

本机工作区有**未提交的改动**和**新增文件**（例如 `src/frontend/superpoint_extractor.cpp`），
所以最省事、最不会漏文件的方式是打包整个工作树（排除 `.git`、`output`、`data`）。

在 **Windows PowerShell** 里执行：

```powershell
# 进入仓库根目录后打包（Windows 自带的 tar.exe 即可）
tar -czf sphere_vio.tar.gz --exclude=.git --exclude=output --exclude=data .

# 传到 Jetson（把 user 和 ip 换成你的）
scp sphere_vio.tar.gz user@192.168.x.x:~/
```

在 Jetson 上解压：

```bash
mkdir -p ~/sphere_vio
tar -xzf ~/sphere_vio.tar.gz -C ~/sphere_vio
```

> 想保留 git 历史的话，也可以先在 Windows 上 `git add -A && git commit`，再用
> `git bundle create sphere_vio.bundle --all` 打包，`scp` 后 `git clone`。两种都行，
> 打包工作树最简单。

## 3. 原生安装依赖（仅 JetPack 5 / Ubuntu 20.04）

> 你的机器是 JetPack 6 / Ubuntu 24.04，**跳过本节**，直接用第 1.1 节的
> `docker/Dockerfile.jetson` 容器。本节保留给恰好用 JetPack 5 的机器。

### 3.1 ROS Noetic + 本项目用到的 catkin 包

```bash
sudo apt update
sudo apt install -y curl gnupg2 lsb-release

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
  > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo apt update

sudo apt install -y \
  build-essential cmake git python3-pip \
  ros-noetic-ros-base \
  ros-noetic-catkin \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-message-filters \
  ros-noetic-rosbag \
  ros-noetic-tf2 \
  ros-noetic-tf2-ros \
  ros-noetic-geometry-msgs \
  ros-noetic-nav-msgs \
  ros-noetic-sensor-msgs \
  ros-noetic-std-msgs \
  ros-noetic-roscpp
```

> 这些 ROS 包对应 `package.xml` 和 `CMakeLists.txt` 里的 `find_package(catkin ...)`
> 组件。少了任何一个，`catkin_make` 会在 configure 阶段直接报找不到包。

### 3.2 Eigen / OpenCV / yaml-cpp

```bash
sudo apt install -y libeigen3-dev libopencv-dev libyaml-cpp-dev
```

> **Jetson 的 OpenCV 坑**：JetPack 自带一套 CUDA 版 OpenCV，而 ROS apt 的
> `cv_bridge` 是照着 Ubuntu 的 OpenCV 4.2 编译的，两者 ABI 可能不一致。CPU 门禁
> 阶段建议统一用 apt 的 `libopencv-dev`（4.2），`find_package` 能找到它即可；等
> SuperPoint 阶段 ONNX Runtime 不依赖 OpenCV，所以这套 4.2 依然够用。

### 3.3 评测脚本依赖

```bash
sudo apt install -y python3-numpy   # 或 pip3 install numpy
```

## 4. 编译 + 跑单元测试

用 catkin workspace 包住仓库（软链接即可，改动实时可见）：

```bash
mkdir -p ~/catkin_ws/src
ln -s ~/sphere_vio ~/catkin_ws/src/sphere_vio

cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=sphere_vio
```

编译通过后跑全部 gtest：

```bash
cd ~/catkin_ws
catkin_make run_tests -DCATKIN_WHITELIST_PACKAGES=sphere_vio
catkin_test_results
```

重点看这几个新增/修改的用例是否全绿：

- `ImuIntervalBufferTest.*`：区间边界与插值（时间轴修复的核心）。
- `MsckfFeatureAccumulatorTest.*`：失跟 drain、边缘化 segment 只消费一次。
- `MsckfTest.*`：静止初始化门控。
- `SuperPointExtractorTest.*`：默认构建下必须明确 unavailable（不会静默降级）。
- `OmniRectifierTest.*`：校正映射往返误差。
- `CrossCameraMatcherTest.*`：L2 描述子匹配与混合格式拒绝，原 ORB 测试不回归。

如果某条挂了，先把 `catkin_test_results` 的失败文件名和断言输出发回来。

### 4.1 容器里的增量编译循环（JetPack 6）

镜像 `docker build` 每次都会全量重编，改几个文件也慢。调试期更快的做法是把源码
和 build/devel 目录都挂到宿主机，让 `catkin_make` 做增量编译：

```bash
# 第一次：在 Jetson 上建好用于持久化的编译目录
mkdir -p ~/sphere_vio_build/build ~/sphere_vio_build/devel
```

之后每次改完代码都执行：

```bash
docker run --rm \
  -v "$HOME/sphere_vio:/root/catkin_ws/src/sphere_vio" \
  -v "$HOME/sphere_vio_build/build:/root/catkin_ws/build" \
  -v "$HOME/sphere_vio_build/devel:/root/catkin_ws/devel" \
  --entrypoint bash sphere_vio:jetson -lc \
  "source /opt/ros/noetic/setup.bash; \
   cd /root/catkin_ws; \
   catkin_make -DCATKIN_WHITELIST_PACKAGES=sphere_vio"
```

跑单测同理，在后面拼上：

```bash
   catkin_make run_tests -DCATKIN_WHITELIST_PACKAGES=sphere_vio && \
   catkin_test_results
```

这样 `build`/`devel` 留在宿主机，第二次起就是增量编译。

## 5. 准备数据

只需要 `eight_noyaw_3_short` 一个序列的 bag 和 GT。在 Windows 上：

```powershell
scp `
  "C:\HHQ_STUDY\RM_QY\Drone\oiv_and_slam\D2dataset\quadcam_7inch_n3_2023_1_14\quadcam_7inch_n3_2023_1_14\eight_noyaw_3_short-sync.bag" `
  user@192.168.x.x:~/D2dataset/

scp `
  "C:\HHQ_STUDY\RM_QY\Drone\oiv_and_slam\D2dataset\quadcam_7inch_n3_2023_1_14\eight_noyaw_3_short-groundtruth.txt" `
  user@192.168.x.x:~/D2dataset/
```

在 Jetson 上确认 bag 可读：

```bash
source /opt/ros/noetic/setup.bash
rosbag info ~/D2dataset/eight_noyaw_3_short-sync.bag
```

（JetPack 6 容器里等价命令：`docker run --rm -v "$HOME/D2dataset:/data" --entrypoint bash sphere_vio:jetson -lc "source /opt/ros/noetic/setup.bash; rosbag info /data/eight_noyaw_3_short-sync.bag"`。）

（`eight_yaw_1` 才需要 reindex，`eight_noyaw_3_short` 一般不需要。）

## 6. 门禁一：时间修复 FAST/ORB 基线

先确认 `~/sphere_vio/config/system.yaml` 里这些值是对的（本轮已经改好，别手滑覆盖）：

```yaml
synchronization:
  camera_to_imu_offset_s: -0.186
offline:
  output_period: 0.05
frontend:
  pipeline_mode: legacy_per_camera
backend:
  msckf:
    stationary_initialization_gate: true
    time_offset_cam_imu: 0.0
```

运行：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

rosrun sphere_vio sphere_vio_feature_runner \
  --config ~/sphere_vio/config/offline.yaml \
  --frontend-config ~/sphere_vio/config/system.yaml \
  --cameras ~/sphere_vio/config/cameras_d2slam.yaml \
  --bag ~/D2dataset/eight_noyaw_3_short-sync.bag \
  --msckf \
  --output-dir ~/sphere_vio/output/msckf_eight_noyaw_3_short
```

> JetPack 6 / Ubuntu 24.04 用容器跑（把 `$HOME/sphere_vio` 和 `$HOME/D2dataset`
> 挂进容器，路径与上面等价）：
>
> ```bash
> docker run --rm \
>   -v "$HOME/sphere_vio:/repo" \
>   -v "$HOME/D2dataset:/data" \
>   --entrypoint bash sphere_vio:jetson -lc \
>   "source /opt/ros/noetic/setup.bash; source /root/catkin_ws/devel/setup.bash; \
>    rosrun sphere_vio sphere_vio_feature_runner \
>      --config /repo/config/offline.yaml \
>      --frontend-config /repo/config/system.yaml \
>      --cameras /repo/config/cameras_d2slam.yaml \
>      --bag /data/eight_noyaw_3_short-sync.bag \
>      --msckf \
>      --output-dir /repo/output/msckf_eight_noyaw_3_short"
> ```
>
> 但注意：容器里 `/root/catkin_ws/devel` 是 `docker build` 时编译的**快照**。
> 你刚改了代码的话，要么重新 `docker build`，要么用第 4 节「挂目录手动编译」的
> 方式，别用旧 devel 跑新代码。

日志里重点核对（对应本次改动要验证的点）：

- `IMU/clock diagnostics` 区：`camera_to_imu_offset_s = -0.186`；
  `boundary_interpolations / boundary_holds` 合理，不是全 0。
- `MSCKF update diagnostics` 区：`accepted_features` 与 `considered_features`
  的接受率，`landmark_count`（本轮关闭持久地标后应为 0 或很小）。
- 全程没有 `nan`、`inf`，退出码是 0。

评测：

```bash
python3 ~/sphere_vio/scripts/evaluate_sphere_vio.py \
  --groundtruth ~/D2dataset/eight_noyaw_3_short-groundtruth.txt \
  --trajectory ~/sphere_vio/output/msckf_eight_noyaw_3_short/trajectory.csv \
  --output-period 0.05 \
  --output-report ~/sphere_vio/result/report_jetson_eight_noyaw_3_short.md
```

看 `ATE_SE3 / RPE_10m / coverage / max_pose_interval / has_nan_inf`。

## 7. 门禁二：rectified FAST/ORB

只改 `config/system.yaml` 一行：

```yaml
frontend:
  pipeline_mode: rectified
```

换一个输出目录重跑即可（几何仍然用原始鱼眼像素，只有检测/LK 在 200°×100°
校正图上做）：

```bash
rosrun sphere_vio sphere_vio_feature_runner \
  --config ~/sphere_vio/config/offline.yaml \
  --frontend-config ~/sphere_vio/config/system.yaml \
  --cameras ~/sphere_vio/config/cameras_d2slam.yaml \
  --bag ~/D2dataset/eight_noyaw_3_short-sync.bag \
  --msckf \
  --output-dir ~/sphere_vio/output/msckf_rectified_eight_noyaw_3_short
```

## 8. 门禁三（可选）：SuperPoint CUDA

> **JetPack 6 的重要限制**：ROS Noetic 是 Ubuntu 20.04，而 JetPack 6 的
> CUDA/cuDNN/TensorRT 用户态库是给 Ubuntu 22.04/24.04 的 glibc 编译的，装不进
> Ubuntu 20.04 容器（glibc/ABI 不兼容）。所以在这台 JetPack 6 机器上，很难把
> 「ROS Noetic 的完整 VIO」和「Jetson GPU」放进同一个进程。要端到端跑 GPU
> SuperPoint，现实选项是：把 Jetson 刷成 JetPack 5.1.x（Ubuntu 20.04，ROS
> Noetic 与 CUDA 11.4 原生共存），或在宿主 Ubuntu 24.04 上单独做
> 「读帧 → SuperPoint 推理」的 GPU benchmark。详见聊天说明。

前提：你已经拿到 D2SLAM 兼容的 `superpoint_v1_sim_int32.onnx`，并在
`config/system.yaml` 的 `frontend.superpoint.model_path` 指向它。

Jetson 是 aarch64，**不要用**仓库里的 `docker/Dockerfile.cuda`。原生构建步骤：

1. 安装 aarch64 版 ONNX Runtime GPU（版本要和你的 JetPack / L4T 匹配）：

   ```bash
   # JetPack 5 常见做法：从 NVIDIA 的 Jetson 索引安装
   pip3 install -U pip wheel
   pip3 install onnxruntime-gpu -f https://pypi.nvidia.com
   ```

   具体可用版本按你的 `cat /etc/nv_tegra_release` 现场确认；JetPack 6 用
   CUDA 12 / TensorRT 8.6，JetPack 5 用 CUDA 11.4 / TensorRT 8.4。

2. pip 的 wheel 只带 `.so`，不带 C++ 头文件。取匹配版本源码里的
   `include/onnxruntime` 放到 `/usr/local/include`，并把 `.so` 归集到
   `/usr/local/onnxruntime/lib`（`find_library` 需要 `libonnxruntime.so` 这个
   不带版本号的符号链接）。

3. 用同样参数打开编译开关重新 catkin 编译：

   ```bash
   cd ~/catkin_ws
   source /opt/ros/noetic/setup.bash
   catkin_make -DCATKIN_WHITELIST_PACKAGES=sphere_vio \
     -DSPHERE_VIO_WITH_ONNXRUNTIME_CUDA=ON \
     -DONNXRUNTIME_DIR=/usr/local/onnxruntime
   ```

4. 跑的时候加 `--cross-camera-matching`（`superpoint_cuda` 必须开它）：

   ```bash
   rosrun sphere_vio sphere_vio_feature_runner \
     --config ~/sphere_vio/config/offline.yaml \
     --frontend-config ~/sphere_vio/config/system.yaml \
     --cameras ~/sphere_vio/config/cameras_d2slam.yaml \
     --bag ~/D2dataset/eight_noyaw_3_short-sync.bag \
     --msckf --cross-camera-matching \
     --output-dir ~/sphere_vio/output/msckf_superpoint_eight_noyaw_3_short
   ```

如果 CUDA Provider 没起来、模型路径不对或没有该模型，进程会以退出码 9 明确报错，
**不会**悄悄退回 ORB。这一步目前大概率还缺模型和 aarch64 ORT，不要卡在这里，
先确认门禁一/二达标。

## 9. 常见问题

- **`catkin_make` 找不到 `cv_bridge` / 某个 ROS 包**：对应第 3.1 节的组件没装全，
  用 `apt-cache search ros-noetic-<name>` 补装。
- **OpenCV 报 ABI 错误 / cv_bridge 崩**：多半是 JetPack OpenCV 和 apt OpenCV 4.2
  混用。门禁一/二用 apt 的 `libopencv-dev` 重新编译整个 workspace 试一次。
- **内存不够编译**：Orin NX 8GB 版编译大项目可能触发 OOM，加 swap：
  `sudo fallocate -l 8G /swapfile && sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile`。
- **bag 读不动 / 时间戳报错**：先 `rosbag info` 看话题是否包含
  `/arducam/image/compressed` 和 `/dji_sdk_1/dji_sdk/imu`；个别序列要先
  `rosbag reindex`。
- **跑出来 `max_pose_interval` 还是超 0.1s**：确认 `offline.output_period=0.05`
  且后端已初始化才写轨迹（本轮已处理），再查日志里第一帧初始化时间点。

## 10. 每轮调参的固定动作

改代码 → `catkin_make` → `run_tests` → 跑 `eight_noyaw_3_short` → 记录
`ATE_SE3 / RPE_10m / coverage / max_pose_interval / has_nan_inf / RTF` →
把 stdout 日志和报告存进 `result/`，并简要写清「改了什么、参数是什么、指标变化」。
