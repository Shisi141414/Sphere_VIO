# Sphere-VIO 中文使用说明与预研运行手册

本文面向四目鱼眼相机 + IMU 无人机的 VIO 算法预研，覆盖仓库现状、ROS 版本判断、
ROS2 迁移方案（仅说明，暂不实施）、Docker 构建、Xlaunch 可视化、结果保存和需要
重点关注的诊断指标。

## 1. 仓库现状与可执行程序

`Sphere_VIO` 是一个 ROS 1 Noetic 的 `catkin` 包，目标平台是四路全向鱼眼相机
与一路 IMU。当前代码是“可重复诊断基线”，不是已经完成闭环的完整 VIO：

```text
当前已经实现
  - 四路 image + IMU 的 rosbag 直接读取与帧装配
  - FAST 网格检测、金字塔 LK 同相机时序跟踪
  - ORB 描述子、跨相机候选匹配与球面极线过滤
  - 当前帧两视图三角化几何诊断
  - LandmarkTrack 观测关联与生命周期管理
  - 有限半径 USPM 全景逆重映射及首帧保存
  - 初版 15 状态 ESKF/IMU 后端、地标地图、CSV 与 ROS 输出
  - MSCKF 滑动窗口、clone 增广/边缘化、omni-radtan 球面重投影更新

当前尚未实现
  - 跨帧逆深度滤波
  - MSCKF 滑动窗口 clone 与严格球面重投影雅可比
  - HOFA 多层半直接对齐
```

因此本次“在数据库上检验效果”主要检验的是：传感器数据是否完整、时间同步是否正常、
特征跟踪是否稳定、跨相机匹配/三角化几何是否自洽、USPM 全景是否可生成。它不会
直接输出可与 ground truth 对齐的 VIO 轨迹。

主要可执行文件：

| 可执行文件 | 作用 |
| --- | --- |
| `sphere_vio_bag_runner` | 统计四路图像与 IMU 的完整性、同步和耗时 |
| `sphere_vio_feature_runner` | 跑时序前端、跨相机匹配、三角化诊断、LandmarkTrack |
| `sphere_vio_panorama_runner` | 跑 USPM 全景逆重映射，可保存首帧 PNG |
| `sphere_vio_bag_visualizer` | 交互/离屏显示四路图像与几何诊断 |
| `sensor_inspector_node` | 实时 ROS 传感器输入检查节点 |

## 2. 当前基于 ROS 还是 ROS2

当前基于 **ROS 1 Noetic**，不是 ROS2。判断依据：

- `package.xml` 使用 `<buildtool_depend>catkin</buildtool_depend>`，并依赖
  `roscpp`、`rosbag`、`message_filters`、`cv_bridge` 等 ROS1 包。
- `CMakeLists.txt` 使用 `find_package(catkin ...)`、`catkin_package()`、
  `add_dependencies(... ${catkin_EXPORTED_TARGETS})`。
- 源码使用 `ros::init()`、`ros::NodeHandle`、`rosbag::Bag`、`ros::Time`、
  `sensor_msgs/Image.h` 等 ROS1 API。
- `launch/*.launch` 是 ROS1 XML launch 文件。

## 3. 迁移到 ROS2 的修改点（暂不实施）

如果后续要迁移到 ROS2，建议保持核心算法 `sphere_vio_camera/geometry/panorama/
frontend` 不变，重点重写 `src/ros/` 与构建/launch 层。需要修改的内容包括：

1. 构建系统：把 `catkin` 换成 `ament_cmake`。`package.xml` 改为
   `<buildtool_depend>ament_cmake</buildtool_depend>`，依赖改为 `rclcpp`、
   `sensor_msgs`、`cv_bridge`、`image_transport`、`message_filters`、`tf2_ros`
   等 ROS2 包；`CMakeLists.txt` 改用 `find_package(ament_cmake ...)`、
   `ament_target_dependencies()`、`install(...)`、`ament_package()`。
2. 节点入口：`ros::init/ros::NodeHandle` 改为 `rclcpp::init()` 和
   `rclcpp::Node::make_shared()`；`Publisher/Subscriber` 改为 `rclcpp` 版本；
   回调改为 `std::function` 接口。日志 `ROS_INFO/ROS_WARN` 改为
   `RCLCPP_INFO/RCLCPP_WARN`。
3. 时间与消息：`ros::Time`/`ros::Duration` 改为 `rclcpp::Time`/
   `rclcpp::Duration`；消息头仍然使用 `std_msgs/Header`、`sensor_msgs/Image`、
   `sensor_msgs/Imu`，但 ROS2 生成头文件路径和消息对象生命周期不同。
4. rosbag：ROS1 的 `rosbag::Bag/rosbag::View` 改为 `rosbag2_cpp` /
   `rosbag2_storage`，离线 runner 与 visualizer 的读取层需要重写。也可以用
   `rosbag2_py` 做预处理，但当前 C++ 离线工具更倾向于直接改 C++ 层。
5. 同步：`message_filters::Synchronizer` 在 ROS2 中仍然存在，但策略头文件和
   模板参数有变化；不把 IMU 放入五路同步器的设计原则保持不变。
6. TF：`tf2_ros::TransformBroadcaster`、`tf2_ros::Buffer` 的命名空间基本接近，
   但 ROS2 下节点生命周期、时钟接口和 QoS 设置不同。
7. 参数：ROS1 的 `NodeHandle::getParam` / `rosparam` 改为
   `declare_parameter()` + YAML 加载；launch XML 改为 ROS2 Python launch 或
   XML launch 的新标签。
8. 测试与安装：`catkin_add_gtest()` 改为 `ament_add_gtest()`，
   `CATKIN_ENABLE_TESTING` 改为 `BUILD_TESTING`；安装目录变量改为
   `AMENT_PREFIX_PATH` 对应的目标。

建议分三步迁移：先把纯算法库编译为不依赖 ROS 的静态/共享库，再把
`sensor_inspector_node` 作为第一个 ROS2 节点迁移，最后迁移三个离线 runner
和 visualizer。

## 4. Docker 构建

仓库已补充 `Dockerfile`、`.dockerignore` 和 `docker/entrypoint.sh`。在 Windows
上使用 Docker Desktop 的 PowerShell 构建：

```powershell
cd C:\HHQ_STUDY\RM_QY\Drone\oiv_and_slam\Sphere_VIO
docker build -t sphere_vio:noetic .
```

镜像会安装 ROS Noetic、Eigen、OpenCV、yaml-cpp 与 OpenCV GUI 运行库，并把仓库
放入 `/root/catkin_ws/src/sphere_vio` 后执行 `catkin_make`。

构建完成后可以先跑一遍单元测试，确认几何、相机模型、前端和 ROS 装配层在当前
镜像中可用：

```bash
docker run --rm sphere_vio:noetic \
  catkin_make run_tests -DCATKIN_WHITELIST_PACKAGES=sphere_vio
```

该命令会编译并运行仓库中的 17 个 gtest 可执行文件；返回码为 0 表示全部通过。

## 5. Xlaunch 可视化准备

可视化依赖 X11，Windows 上使用 VcXsrv 的 `Xlaunch`：

1. 启动 `Xlaunch`。
2. Display number 选择 `-1`(自动)。
3. 选择 `Multiple windows`。
4. 选择 `Start no client`。
5. **在 Extra settings 中勾选 `Disable access control`，否则容器内程序连接会被拒绝。**
6. 如 Windows 防火墙提示，允许 VcXsrv 在专用网络通信。

然后进入容器时把 `DISPLAY` 指向 Windows 主机：

```powershell
docker run --rm -it `
  -e DISPLAY=host.docker.internal:0.0 `
  sphere_vio:noetic
```

容器内可以运行：

```bash
rosrun sphere_vio sphere_vio_bag_visualizer --help
```

如果 OpenCV 窗口仍不能显示，再补加
`-e QT_X11_NO_MITSHM=1 -e LIBGL_ALWAYS_INDIRECT=0`，但大多数 VcXsrv 场景下
不是必须的。

## 6. 数据目录与结果保存

建议在 Windows 仓库根目录准备两个目录：

```text
Sphere_VIO/
├── data/          # 放入四目 + IMU 的 rosbag，例如 sphere_algorithm_test.bag
└── output/        # 保存日志、首帧全景 PNG、后续分析结果
```

启动容器时挂载这两个目录：

```powershell
$repo = "C:\HHQ_STUDY\RM_QY\Drone\oiv_and_slam\Sphere_VIO"
docker run --rm -it `
  -e DISPLAY=host.docker.internal:0.0 `
  -v "$repo\data:/data" `
  -v "$repo\output:/output" `
  sphere_vio:noetic
```

容器内三个配置文件的默认路径为：

```text
/root/catkin_ws/src/sphere_vio/config/offline.yaml
/root/catkin_ws/src/sphere_vio/config/system.yaml
/root/catkin_ws/src/sphere_vio/config/cameras.yaml
```

### 6.1 保存统计日志

三个离线 runner 目前都把统计结果打印到 `stdout`，所以保存结果最直接的方法是
重定向日志，例如：

```bash
rosrun sphere_vio sphere_vio_bag_runner \
  --config /root/catkin_ws/src/sphere_vio/config/offline.yaml \
  --bag /data/sphere_algorithm_test.bag \
  2>&1 | tee /output/bag_runner.log
```

同理可保存 feature runner 和 panorama runner 的日志：

```bash
rosrun sphere_vio sphere_vio_feature_runner \
  --config /root/catkin_ws/src/sphere_vio/config/offline.yaml \
  --bag /data/sphere_algorithm_test.bag \
  --cross-camera-matching \
  --triangulation-candidates \
  --landmark-tracks \
  2>&1 | tee /output/feature_runner.log
```

### 6.2 保存首帧全景 PNG

`sphere_vio_panorama_runner` 提供 `--save-first-frame DIR`，会写出一组 2048x1024
的灰度诊断图：

```bash
mkdir -p /output/panorama
rosrun sphere_vio sphere_vio_panorama_runner \
  --config /root/catkin_ws/src/sphere_vio/config/offline.yaml \
  --system-config /root/catkin_ws/src/sphere_vio/config/system.yaml \
  --cameras /root/catkin_ws/src/sphere_vio/config/cameras.yaml \
  --bag /data/sphere_algorithm_test.bag \
  --parallel-cameras \
  --save-first-frame /output/panorama \
  2>&1 | tee /output/panorama_runner.log
```

生成文件：

```text
frame0000_c0.png ... frame0000_c3.png   # 每路相机的 USPM 重映射层
frame0000_coverage.png                  # 每个全景像素被 0..4 路覆盖的强度图
frame0000_owner.png                     # owner/source 相机编号图
frame0000_composite.png                 # owner 选择的诊断合成全景
```

### 6.3 无真实 rosbag 时的合成冒烟测试

如果暂时没有四目鱼眼加 IMU 的数据库，可以用仓库里的
`scripts/make_synthetic_bag.py` 生成一个很小的确定性 rosbag，用来先验证整套
离线 runner、可视化渲染和输出目录挂载。这个合成数据只用于冒烟测试，不能代替
真实数据库的精度评估。

```powershell
$repo = "C:\HHQ_STUDY\RM_QY\Drone\oiv_and_slam\Sphere_VIO"
docker run --rm `
  -v "$repo:/repo" `
  -v "$repo\output:/output" `
  sphere_vio:noetic `
  python3 /repo/scripts/make_synthetic_bag.py --out /output/synthetic.bag
```

之后把第 7 节命令中的 `/data/sphere_algorithm_test.bag` 换成
`/output/synthetic.bag` 即可。生成物会写到 Windows 的 `output/`：

```text
synthetic.bag
synthetic_bag_runner.log
synthetic_feature_runner.log
synthetic_panorama_runner.log
synthetic_visualizer_headless.log
panorama_synthetic/frame0000_*.png
```

## 7. 完整运行命令

### 7.1 数据完整性检查

```bash
rosrun sphere_vio sphere_vio_bag_runner \
  --config /root/catkin_ws/src/sphere_vio/config/offline.yaml \
  --bag /data/sphere_algorithm_test.bag \
  2>&1 | tee /output/bag_runner.log
```

该命令不依赖 `roscore`，不播放传感器，只直接读 rosbag。

### 7.2 特征前端与几何诊断

按需要逐层打开开关，建议首次预研直接打开到 landmark track：

```bash
rosrun sphere_vio sphere_vio_feature_runner \
  --config /root/catkin_ws/src/sphere_vio/config/offline.yaml \
  --bag /data/sphere_algorithm_test.bag \
  --cross-camera-matching \
  --triangulation-candidates \
  --triangulation-threshold-sweep \
  --landmark-tracks \
  2>&1 | tee /output/feature_runner.log
```

开关含义：

```text
--cross-camera-matching        启用四对重叠相机的 ORB 跨相机匹配
--triangulation-candidates     启用当前帧两视图三角化几何门控
--triangulation-threshold-sweep 对关键阈值做单变量扫描
--landmark-tracks              启用跨相机观测关联与生命周期统计
```

### 7.3 USPM 全景

```bash
mkdir -p /output/panorama
rosrun sphere_vio sphere_vio_panorama_runner \
  --config /root/catkin_ws/src/sphere_vio/config/offline.yaml \
  --system-config /root/catkin_ws/src/sphere_vio/config/system.yaml \
  --cameras /root/catkin_ws/src/sphere_vio/config/cameras.yaml \
  --bag /data/sphere_algorithm_test.bag \
  --parallel-cameras \
  --save-first-frame /output/panorama \
  2>&1 | tee /output/panorama_runner.log
```

### 7.4 Xlaunch 交互可视化

基础四路图像：

```bash
rosrun sphere_vio sphere_vio_bag_visualizer \
  --config /root/catkin_ws/src/sphere_vio/config/offline.yaml \
  --bag /data/sphere_algorithm_test.bag \
  --rate 1.0
```

叠加球面覆盖、极线、时序特征、跨相机匹配、三角化候选、LandmarkTrack：

```bash
rosrun sphere_vio sphere_vio_bag_visualizer \
  --config /root/catkin_ws/src/sphere_vio/config/offline.yaml \
  --bag /data/sphere_algorithm_test.bag \
  --rate 1.0 \
  --show-spherical-coverage \
  --show-temporal-features \
  --show-cross-camera-matches \
  --match-camera-1 2 \
  --match-camera-2 3 \
  --show-triangulation-candidates \
  --show-landmark-tracks
```

USPM 全景显示：

```bash
rosrun sphere_vio sphere_vio_bag_visualizer \
  --config /root/catkin_ws/src/sphere_vio/config/offline.yaml \
  --bag /data/sphere_algorithm_test.bag \
  --show-uspm-panorama \
  --show-uspm-layers \
  --show-uspm-owner
```

交互键：`Space` 暂停/继续，`N` 暂停时前进一帧，`Q` 或 `Esc` 退出。

### 7.5 ESKF / IMU 后端

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

加上 `--publish-ros` 可发布 odometry、path、TF 和 landmark 点云，前提是容器外已经启动 `roscore`。ESKF 状态与地标 CSV 会写入 `--output-dir`。更完整的后端说明见
`docs/BACKEND_ESKF_ZH.md`。

使用完整 MSCKF 滑动窗口时，把 `--esfk` 替换成 `--msckf`，其余参数相同。MSCKF
会在每个同步图像帧增广 pose clone、用 `OmniRadtan` 球面投影构造视觉残差，并在
超过窗口长度后边缘化最旧 clone。实现说明见 `docs/MSCKF_ZH.md`。

**时间轴约定**：`system.yaml` 的 `synchronization.camera_to_imu_offset_s` 采用
D2SLAM 权威标定值 `-0.186`（即 `t_imu = t_camera - 0.186 s`）。IMU 区间提取、
MSCKF 传播、clone 与观测关联都换算到 IMU 时钟；`trajectory.csv` 仍用原始相机
时间戳输出，以保证与 groundtruth 对齐。旧的 ±25 ms 在线时间偏移/外参搜索已禁用。

**前端模式**：`frontend.pipeline_mode` 可选 `legacy_per_camera`（默认，原始鱼眼
图 FAST+LK）、`rectified`（200°×100°、800×400 局部球面校正图上检测跟踪，几何
仍用原始鱼眼像素）与 `superpoint_cuda`（SuperPoint 批推理替代 ORB 描述子，需
ONNX Runtime CUDA 构建与 D2SLAM 兼容 ONNX 模型，见 `README.md`）。`rectified`
与 `superpoint_cuda` 之外不改变 MSCKF 的后端接口。

## 8. 重点指标及含义

### 8.1 数据完整性与同步（bag runner）

| 指标 | 含义与判断 |
| --- | --- |
| `camera0..3 image count` | 四路图像总数，应先确认四路数量接近，数量差异说明丢帧或话题配置错误 |
| `completed four-camera frame count` | 真正完成四路同步装配的帧数；只有这些帧会进入算法 |
| `dropped image count` | 因超时/乱序被丢弃的图像；应接近 0 |
| `timestamp mismatch count` | 四路图像时间戳不满足最大时间差或严格同步要求；应接近 0 |
| `minimum/maximum/average IMU interval` | IMU 相邻采样间隔，理想值约 `1/IMU频率`；过大说明丢 IMU |
| `non-monotonic / duplicate IMU timestamp` | IMU 时间戳乱序或重复，应接近 0 |
| `abnormally large IMU interval count` | 超过阈值的 IMU 空洞，直接影响后续 IMU 积分 |
| `minimum/maximum/average IMU per image frame` | 每两帧图像之间应有足够 IMU；过少表示 IMU 与图像频率不匹配或丢包 |
| `frames without sufficient IMU` | 没有 IMU 覆盖的图像帧数；应接近 0 |
| `average processing time per completed frame` | 离线处理吞吐，便于估计实时运行是否达标 |

### 8.2 特征前端质量（feature runner）

每路相机 `C0..C3`：

| 指标 | 含义 |
| --- | --- |
| `active avg/min/max` | 当前活跃特征数量，过低会导致后续匹配过少，过高增加耗时 |
| `new / tracked / rejected` | 新检测、成功跟踪、被拒绝数量；`tracked` 过低说明 LK 跟踪不稳定 |
| `age avg/max` | 特征连续存活帧数，反映跟踪持续性 |
| `FB avg/max` | LK 前向-反向误差，越小说明光流一致性越好 |
| `model-domain rejected` | 因相机模型域无效而拒绝的数量，过大可能是标定/分辨率问题 |

跨相机匹配漏斗：

| 指标 | 含义 |
| --- | --- |
| `raw -> absolute -> ratio -> mutual -> epipolar -> final` | 从候选到最终匹配的逐级筛选数量；若某一级急剧下降，定位是哪类约束过严 |
| `descriptor distance avg/max` | ORB 描述子距离，越小越相似；过高说明纹理弱或匹配错误 |
| `epipolar error avg/max rad` | 球面极线角度误差，越小说明两路相机几何越一致 |

三角化候选：

| 指标 | 含义 |
| --- | --- |
| `input / triangulation successes / admitted` | 输入匹配、有正深度几何解、通过所有门控的数量 |
| `status counts` | 各拒绝原因计数，用于定位负深度、视差角不足、深度超限、最近距离过大、重投影超差等 |
| `ray angle rad` | 两条观测射线的夹角，过小表示基线退化，三角化不可靠 |
| `minimum/maximum depth m` | 当前帧恢复的深度范围，应落在合理场景尺度内 |
| `closest distance m` | 两条射线最短距离，接近 0 表示几何交点良好 |
| `angular reprojection error rad` | 两路角重投影误差，越小几何一致性越好 |

LandmarkTrack 生命周期：

| 指标 | 含义 |
| --- | --- |
| `admitted candidate inputs / created tracks` | 进入关联的候选与真正新建假设数量；创建过少通常说明匹配/门控太严 |
| `association conflicts` | 关联冲突数量，同一相机已有成员冲突过多说明跨相机 ID 关联仍有问题 |
| `active / stale / retired` | 假设生命周期分布，过多 stale/retired 说明跟踪无法持续 |
| `tracks ever reaching active` | 真正达到 active 的假设数量；当前 active 仍只是观测关联假设，不是确认 landmark |

### 8.3 USPM 全景（panorama runner）

| 指标 | 含义 |
| --- | --- |
| `C0..C3 valid (%)` | 每路相机在全景网格中的有效像素比例，检查镜头朝向和标定 |
| `coverage[0..4]` | 全景像素被 0..4 路相机覆盖的比例；空洞过多说明相机配置/外参异常 |
| `overlap Cx-Cy` | 配置相机对的重叠像素数，用于判断重叠区是否与 `camera_pairs` 匹配 |
| `owner[C0..C3]` | owner 策略选择的来源分布，判断全景合成是否被某一路垄断 |
| `remap success / failure` | 每帧重映射成功/失败数，应无失败 |
| `remap ms average/max` | 单帧重映射耗时，是后续 HOFA/全景特征可行性的重要性能指标 |

## 9. 建议的预研执行顺序

1. 先跑 `sphere_vio_bag_runner`，确认 rosbag 内话题、四路图像、IMU 频率与时间戳。
2. 再跑 `sphere_vio_feature_runner`，先只开 `--cross-camera-matching`，观察特征与匹配漏斗。
3. 打开 `--triangulation-candidates` 和 `--landmark-tracks`，检查几何/关联统计。
4. 跑 `sphere_vio_panorama_runner` 保存首帧，检查 USPM 覆盖与合成图。
5. 最后用 Xlaunch 打开 `sphere_vio_bag_visualizer`，人工核对四路相机顺序、时间同步、
   特征点位置、匹配线和三角化候选是否正确。

## 10. 当前局限与下一步

当前代码没有输出 `nav_msgs/Odometry`、`nav_msgs/Path`、`sensor_msgs/PointCloud2`
或 TF，因此不能用 `evo` 直接评估轨迹精度。预研结论应聚焦在数据完整性、特征前端
稳定性、跨相机几何自洽性和全景重映射可行性；在得到稳定的 LandmarkTrack 和
几何基线后，再进入逆深度滤波、ESKF 后端与真实轨迹评估。

## 11. D2SLAM 全量数据集运行方法

本节给出在新 D2SLAM 四合一压缩图数据集上跑完整 MSCKF 的具体步骤。离线 runner
已经支持直接读取原始 `-sync.bag`，不需要再运行 `convert_d2slam_quad.py` 生成中间
全量 Sphere-VIO bag。

完整数据集结构、序列规模和话题说明见：

```text
docs/FULL_DATASET_ZH.md
```

### 11.1 数据目录

```text
C:\HHQ_STUDY\RM_QY\Drone\oiv_and_slam\D2dataset\quadcam_7inch_n3_2023_1_14
```

全量测试使用的 5 个序列：

```text
eight_noyaw_1
eight_noyaw_3_short
eight_noyaw_4
eight_noyaw_5
eight_yaw_1
```

每个序列的 rosbag 位于：

```text
quadcam_7inch_n3_2023_1_14\eight_<seq>-sync.bag
```

groundtruth 位于数据集根目录：

```text
eight_<seq>-groundtruth.txt
```

groundtruth 格式为 `timestamp x y z qx qy qz qw`。

### 11.2 准备工作

构建或使用最新镜像：

```powershell
cd C:\HHQ_STUDY\RM_QY\Drone\oiv_and_slam\Sphere_VIO
docker build -t sphere_vio:noetic .
```

`eight_yaw_1-sync.bag` 原始文件可能未建立索引，首次运行前需要 reindex：

```powershell
docker run --rm -v `
  "C:\HHQ_STUDY\RM_QY\Drone\oiv_and_slam\D2dataset\quadcam_7inch_n3_2023_1_14:/data" `
  sphere_vio:noetic bash -lc `
  "source /opt/ros/noetic/setup.bash; rosbag reindex /data/quadcam_7inch_n3_2023_1_14/eight_yaw_1-sync.bag"
```

### 11.3 单序列运行

以 `eight_noyaw_3_short` 为例：

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

关键说明：

- `--config` 使用 `config/offline.yaml`，其中已经配置原始 D2SLAM 四合一压缩图话题
  `/arducam/image/compressed` 和 DJI IMU 话题 `/dji_sdk_1/dji_sdk/imu`。
- `--cameras` 使用 D2SLAM 7-inch-n3 标定转换后的 `config/cameras_d2slam.yaml`。
- `--msckf` 会自动启用跨相机匹配、三角化候选、LandmarkTrack 和 MSCKF 后端。
- 输出目录包含：

```text
odometry.csv
trajectory.csv
landmarks.csv
```

其中 `trajectory.csv` 是严格 TUM 顺序：

```text
timestamp,x,y,z,qx,qy,qz,qw
```

### 11.4 批量运行全部序列

```powershell
$repo = "C:\HHQ_STUDY\RM_QY\Drone\oiv_and_slam\Sphere_VIO"
$data = "C:\HHQ_STUDY\RM_QY\Drone\oiv_and_slam\D2dataset\quadcam_7inch_n3_2023_1_14"

$sequences = @(
  "eight_noyaw_1",
  "eight_noyaw_3_short",
  "eight_noyaw_4",
  "eight_noyaw_5",
  "eight_yaw_1"
)

foreach ($seq in $sequences) {
  $cmd = "source /opt/ros/noetic/setup.bash; " +
         "source /root/catkin_ws/devel/setup.bash; " +
         "rosrun sphere_vio sphere_vio_feature_runner " +
         "--config /repo/config/offline.yaml " +
         "--frontend-config /repo/config/system.yaml " +
         "--cameras /repo/config/cameras_d2slam.yaml " +
         "--bag /data/quadcam_7inch_n3_2023_1_14/${seq}-sync.bag " +
         "--msckf " +
         "--output-dir /repo/output/msckf_${seq}"

  docker run --rm -v "${repo}:/repo" -v "${data}:/data" `
    sphere_vio:noetic bash -lc $cmd
}
```

### 11.5 评测

使用仓库内评测脚本：

```powershell
docker run --rm -v "${repo}:/repo" -v "${data}:/data" `
  sphere_vio:noetic python3 `
  /repo/scripts/evaluate_sphere_vio.py `
    --groundtruth /data/eight_noyaw_3_short-groundtruth.txt `
    --trajectory /repo/output/msckf_eight_noyaw_3_short/trajectory.csv `
    --output-period 0.05 `
    --output-report /repo/result/report_eight_noyaw_3_short.md
```

评测脚本会输出 `ATE_SE3`、`ATE_Sim3`、`RPE_1m/5m/10m`、`coverage`、
`max_pose_interval`、`has_nan_inf` 和 `success`。完整指标定义见
`docs/INDEX_LIST.md`。

批量评测全部
```powershell
$sequences = @(
  "eight_noyaw_1",
  "eight_noyaw_3_short",
  "eight_noyaw_4",
  "eight_noyaw_5",
  "eight_yaw_1"
)

foreach ($seq in $sequences) {
  docker run --rm -v "${repo}:/repo" -v "${data}:/data" `
    sphere_vio:noetic python3 `
    /repo/scripts/evaluate_sphere_vio.py `
      --groundtruth /data/${seq}-groundtruth.txt `
      --trajectory /repo/output/msckf_${seq}/trajectory.csv `
      --output-period 0.05 `
      --output-report /repo/result/report_${seq}.md
}
```

### 11.6 可视化轨迹

在 Xlaunch 可用时，可以用 OpenCV 轨迹窗口检查 GT 与估计轨迹：

```powershell
docker run --rm -it `
  -e DISPLAY=host.docker.internal:0.0 `
  -v "$repo:/repo" `
  -v "$data:/data" `
  sphere_vio:noetic python3 `
  /repo/scripts/visualize_trajectory.py `
    --groundtruth /data/eight_noyaw_3_short-groundtruth.txt `
    --trajectory /repo/output/msckf_eight_noyaw_3_short/trajectory.csv
```

## 12. 仓库目录与文件说明

### 12.1 顶层文件

| 文件 | 功能与特点 |
| --- | --- |
| `CMakeLists.txt` | catkin/CMake 构建入口，定义 `sphere_vio_core/camera/geometry/panorama/frontend/backend/ros` 库、可执行程序和 gtest 目标 |
| `package.xml` | ROS1 catkin 包元数据，声明 `cv_bridge`、`rosbag`、`message_filters`、`tf2` 等依赖 |
| `Dockerfile` | 构建 `sphere_vio:noetic` 镜像，安装 ROS Noetic、OpenCV、Eigen、yaml-cpp，并执行 `catkin_make` |
| `docker/entrypoint.sh` | 容器入口，自动 source ROS 和 catkin workspace 后再执行用户命令 |
| `.dockerignore` | 避免把 `.git`、`data`、`output`、`.bag` 等大文件带入镜像 |
| `.devcontainer/`、`.vscode/`、`.clangd` | 开发容器和 IDE/编辑器配置 |

### 12.2 `config/`

| 文件 | 功能与特点 |
| --- | --- |
| `offline.yaml` | 离线 runner 的 bag 路径、相机/IMU 话题、同步参数、起止时间和进度参数 |
| `system.yaml` | 前端参数、跨相机匹配、三角化门控、LandmarkTrack、runtime governor、后端 MSCKF 参数 |
| `cameras.yaml` | 通用四目标定，非 D2SLAM 数据集场景使用 |
| `cameras_d2slam.yaml` | D2SLAM 7-inch-n3 标定转换结果，全量数据集使用 |
| `d2slam/quad_cam_calib-camchain-imucam-7-inch-n3.yaml` | D2SLAM 原始标定 |
| `d2slam/quadcam_multi.yaml`、`quadcam_single.yaml` | D2VINS/D2SLAM 实验编排参考文件 |

### 12.3 `include/` 与 `src/` 模块

| 模块 | 主要文件 | 功能与特点 |
| --- | --- | --- |
| 相机模型 | `camera/camera_model.hpp`、`camera/omni_radtan.*`、`camera/kannala_brandt.*` | 统一相机投影/反投影接口，实现 Omni+Radtan 和 Kannala-Brandt 鱼眼模型 |
| 四目标定 | `camera/camera_rig.hpp`、`common/camera_rig_loader.*` | 多相机外参链、pixel/bearing/body 转换和 YAML 加载 |
| 几何 | `geometry/spherical_geometry.*`、`epipolar_geometry.*`、`triangulation.*` | 球面坐标、极线约束、两视图三角化 |
| 前端检测跟踪 | `frontend/feature_detector.*`、`feature_tracker.*`、`temporal_frontend.*` | FAST 网格检测、LK 金字塔跟踪、四路时序前端 |
| 前端匹配 | `frontend/orb_descriptor_extractor.*`、`cross_camera_matcher.*` | ORB 描述子、跨相机双向匹配和球面极线过滤 |
| 前端候选与关联 | `frontend/triangulation_candidate_evaluator.*`、`landmark_track_manager.*`、`landmark_track.hpp` | 当前帧三角化门控、LandmarkTrack 生命周期 |
| 后端 | `backend/eskf.*`、`backend/msckf.*`、`backend/landmark_map.*` | ESKF、滑窗 MSCKF、持久 landmark 协方差状态 |
| ROS 层 | `ros/frame_assembler.*`、`ros/d2slam_stitched.*`、`ros/offline_feature_runner.*`、`ros/offline_bag_runner.*`、`ros/offline_panorama_runner.*`、`ros/offline_bag_visualizer.*`、`ros/output_recorder.*`、`ros/ros_output.*`、`ros/ros_conversions.*`、`ros/sensor_inspector_node.*` | 离线 bag 读取、四合一图拆分、前端/后端 runner、全景 runner、可视化、CSV 输出、ROS 输出 |
| IMU 缓冲 | `imu_interval_buffer.*` | IMU 时间戳排序、区间抽取和统计 |
| 全景 | `panorama/uspm.*`、`panorama/panorama_remapper.*`、`panorama/panorama_spec.hpp` | 有限半径 USPM、全景重映射与覆盖 mask |

### 12.4 `scripts/`

| 文件 | 功能与特点 |
| --- | --- |
| `convert_d2slam_quad.py` | 把 D2SLAM 四合一 bag 转换成四路 Sphere-VIO bag |
| `convert_d2slam_calib.py` | 把 D2SLAM Kalibr 标定转换成 `cameras_d2slam.yaml` |
| `evaluate_d2slam.py` | 早期 D2SLAM 轨迹评测脚本 |
| `evaluate_sphere_vio.py` | 全量评测脚本，输出 ATE_SE3/ATE_Sim3/RPE/coverage/success |
| `visualize_trajectory.py` | 用 OpenCV 三视图显示 GT 与估计轨迹 |
| `make_synthetic_bag.py` | 生成小型合成 rosbag 用于 smoke test |

### 12.5 `test/`

包含各模块的 gtest 单元测试，以及 `test_main.cpp` 的 GoogleTest 入口和
`frontend_test_utils.hpp` 的测试工具。新增的 `test_d2slam_stitched.cpp` 验证四合一
图像拆分，`test_msckf.cpp` 验证 IMU 初始化、特征累加器、持久 landmark 协方差增广。

### 12.6 `docs/`

| 文件 | 内容 |
| --- | --- |
| `FULL_DATASET_ZH.md` | 全量数据集结构和运行说明 |
| `USAGE_ZH.md` | 总使用手册、Docker/Xlaunch/运行命令 |
| `INDEX_LIST.md` | 评测指标定义 |
| `SPHERE_VIO_IMPLEMENTATION_GUIDE.md` | 详细实现指南 |
| `MSCKF_ZH.md` | MSCKF 后端说明 |
| `BACKEND_ESKF_ZH.md` | ESKF 后端说明 |
| `D2SLAM_DATASET_SHORT_ZH.md` | D2SLAM 早期小数据集接入说明 |

### 12.7 `data/`、`output/`、`result/`

| 目录 | 用途 |
| --- | --- |
| `data/` | 本地 smoke bag 和转换后的 Sphere-VIO bag，不进入 Git |
| `output/` | 各次运行生成的 `odometry.csv`、`trajectory.csv`、`landmarks.csv`、日志和全景图 |
| `result/` | 全量评测报告、改进日志、运行日志和汇总表 |
