# Sphere-VIO 复现项目：代码架构与 ROS 接入实施指南

> 本文档用于指导 Codex 在 VS Code 中持续开发 `sphere_vio`。  
> 项目目标：在四目 360°鱼眼相机与 IMU 平台上，复现 Sphere-VIO 的统一球面表示、多相机特征关联、逆深度估计和滤波式视觉惯性里程计。  
> 文档日期：2026-07-14

## 1. 已确认的开发环境

### x86 开发电脑 `chloe`

```text
Ubuntu 20.04.6 LTS
x86_64
ROS 1 Noetic
catkin_make
GCC/G++ 9.4.0
CMake 3.16.3
OpenCV C++ 4.2.0
Eigen 3.3.7
yaml-cpp 0.6.2
Python venv: /root/catkin_ws/.venv
Python OpenCV: 4.13.0
Workspace: /root/catkin_ws
Package: /root/catkin_ws/src/sphere_vio
```

### ARM 部署设备 `seeker`

```text
Ubuntu 20.04.6 LTS
aarch64 / ARM64
ROS 1 Noetic
Linux 5.10.160
8 logical CPUs
ARM Cortex-A55
Maximum frequency: 2.304 GHz
NEON available: asimd
Workspace: /home/vslam/catkin_ws_vins
```

开发原则：在 `chloe` 上完成编码、单元测试、rosbag 回放和精度分析；只同步源码到 `seeker`，在 ARM 上重新编译并测试实时性能。不得复制 `build/` 和 `devel/`。

---

## 2. 构建与版本管理

Git 仓库根目录应为：

```text
/root/catkin_ws/src/sphere_vio
```

不要在整个 `/root/catkin_ws` 上新建仓库，因为该工作空间还包含其他 ROS 包、虚拟环境和构建输出。

只编译本包：

```bash
cd /root/catkin_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="sphere_vio"
source devel/setup.bash
```

打开新终端时推荐按以下顺序加载：

```bash
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
source /root/catkin_ws/.venv/bin/activate
```

Catkin 可以继续使用系统 `/usr/bin/python3`；项目 Python 工具使用 `.venv`。不要把 Python `cv2` 的路径写入 CMake，C++ 核心应链接系统 OpenCV 4.2.0。

---

## 3. 系统边界

系统分为两部分：

1. `sphere_vio_core`：纯 C++ 核心算法，不依赖 ROS 消息和 ROS 通信；
2. `sphere_vio_ros`：订阅、同步、消息转换、参数读取、结果发布和 TF。

依赖方向必须保持：

```text
ROS node
  -> SphereVioSystem
       -> Frontend
            -> Geometry
                 -> CameraModel
       -> ESKF
```

核心算法中禁止出现：

```cpp
ros::NodeHandle
ros::Publisher
ros::Subscriber
sensor_msgs::Image
sensor_msgs::Imu
nav_msgs::Odometry
```

这些类型只能出现在 `src/ros/` 和 `include/sphere_vio/ros/`。

---

## 4. 目标目录结构

```text
sphere_vio/
├── CMakeLists.txt
├── package.xml
├── README.md
├── docs/
│   └── SPHERE_VIO_IMPLEMENTATION_GUIDE.md
├── config/
│   ├── cameras.yaml
│   ├── imu.yaml
│   └── system.yaml
├── launch/
│   ├── sensor_inspector.launch
│   ├── run_offline.launch
│   └── run_realtime.launch
├── include/sphere_vio/
│   ├── common/
│   │   ├── types.hpp
│   │   ├── transform.hpp
│   │   └── parameters.hpp
│   ├── camera/
│   │   ├── camera_model.hpp
│   │   ├── kannala_brandt.hpp
│   │   └── camera_rig.hpp
│   ├── geometry/
│   │   ├── spherical_geometry.hpp
│   │   ├── epipolar_geometry.hpp
│   │   └── triangulation.hpp
│   ├── frontend/
│   │   ├── feature.hpp
│   │   ├── feature_detector.hpp
│   │   ├── feature_tracker.hpp
│   │   ├── spherical_matcher.hpp
│   │   ├── depth_filter.hpp
│   │   └── frontend.hpp
│   ├── backend/
│   │   ├── imu_types.hpp
│   │   ├── eskf_state.hpp
│   │   ├── imu_propagator.hpp
│   │   └── eskf.hpp
│   ├── system/
│   │   └── sphere_vio_system.hpp
│   └── ros/
│       ├── ros_conversions.hpp
│       ├── sensor_inspector_node.hpp
│       └── sphere_vio_node.hpp
├── src/
│   ├── camera/
│   │   ├── kannala_brandt.cpp
│   │   └── camera_rig.cpp
│   ├── geometry/
│   │   ├── spherical_geometry.cpp
│   │   ├── epipolar_geometry.cpp
│   │   └── triangulation.cpp
│   ├── frontend/
│   │   ├── feature_detector.cpp
│   │   ├── feature_tracker.cpp
│   │   ├── spherical_matcher.cpp
│   │   ├── depth_filter.cpp
│   │   └── frontend.cpp
│   ├── backend/
│   │   ├── imu_propagator.cpp
│   │   └── eskf.cpp
│   ├── system/
│   │   └── sphere_vio_system.cpp
│   └── ros/
│       ├── ros_conversions.cpp
│       ├── sensor_inspector_node.cpp
│       ├── sensor_inspector_node_main.cpp
│       ├── sphere_vio_node.cpp
│       └── sphere_vio_node_main.cpp
└── test/
    ├── test_kannala_brandt.cpp
    ├── test_camera_rig.cpp
    ├── test_spherical_geometry.cpp
    ├── test_epipolar_geometry.cpp
    ├── test_triangulation.cpp
    ├── test_depth_filter.cpp
    └── test_imu_propagation.cpp
```

不要一次创建所有空文件。按照实施阶段逐步增加文件，每一阶段必须能单独编译和测试。

---

## 5. 坐标系与命名规范

统一使用：

```text
W: world
B: body / IMU
C0, C1, C2, C3: four cameras
S: shared spherical representation, normally aligned with B
```

变换约定：

\[
\mathbf p_A=\mathbf R_{AB}\mathbf p_B+\mathbf t_{AB}
\]

因此：

```text
R_a_b: rotates a vector from frame B into frame A
t_a_b: origin of B expressed in frame A
```

变量必须表达坐标系：

```cpp
R_b_c0
t_b_c0
R_w_b
t_w_b
bearing_c
bearing_b
point_w
```

禁止在较大作用域中使用含义不清楚的 `R`、`T`、`pose`、`point`、`extrinsic`。

---

## 6. 公共数据类型

`common/types.hpp` 只保存基础类型，不保存算法。

```cpp
namespace sphere_vio {

using Timestamp = double;
using CameraId = std::uint32_t;
using FeatureId = std::uint64_t;

struct ImageFrame {
    Timestamp timestamp = 0.0;
    CameraId camera_id = 0;
    cv::Mat image;
};

struct MultiCameraFrame {
    Timestamp timestamp = 0.0;
    std::vector<ImageFrame> images;
};

struct ImuMeasurement {
    Timestamp timestamp = 0.0;
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

}  // namespace sphere_vio
```

内部时间统一使用秒和 `double`。进入算法前必须由 ROS 时间戳转换，不得在核心算法中调用 `ros::Time::now()`。

---

## 7. 相机模型层

### 7.1 `CameraModel`

所有相机模型必须实现统一接口：

```cpp
class CameraModel {
public:
    using Ptr = std::shared_ptr<CameraModel>;
    virtual ~CameraModel() = default;

    virtual bool project(
        const Eigen::Vector3d& point_c,
        Eigen::Vector2d* pixel) const = 0;

    virtual bool unproject(
        const Eigen::Vector2d& pixel,
        Eigen::Vector3d* bearing_c) const = 0;

    virtual bool isPixelValid(
        const Eigen::Vector2d& pixel) const = 0;

    virtual int width() const = 0;
    virtual int height() const = 0;
    virtual std::string modelName() const = 0;
};
```

核心关系：

```text
project: 3D point in camera frame -> image pixel
unproject: image pixel -> unit bearing in camera frame
```

所有三维几何应尽早从鱼眼像素转换到单位观测射线 `bearing`。

### 7.2 `KannalaBrandt`

第一版实现 KB4 参数：

```text
width, height
fx, fy, cx, cy
k1, k2, k3, k4
```

公共函数只保留 `project()`、`unproject()` 和有效性检查；角度畸变及牛顿迭代求逆应作为私有函数。

### 7.3 `CameraRig`

管理四个独立相机模型及外参：

```cpp
struct RigCamera {
    CameraModel::Ptr model;
    Eigen::Matrix3d R_b_c;
    Eigen::Vector3d t_b_c;
};
```

核心函数：

```cpp
bool pixelToBodyBearing(
    CameraId camera_id,
    const Eigen::Vector2d& pixel,
    Eigen::Vector3d* bearing_b) const;
```

不得假设四个镜头拥有相同的内参、分辨率或鱼眼模型。

---

## 8. 球面几何层

本层只处理单位射线、旋转和平移，不关心射线来自针孔还是鱼眼相机。

### 8.1 球面与 ERP

核心函数：

```cpp
Eigen::Vector2d bearingToLongitudeLatitude(
    const Eigen::Vector3d& bearing);

Eigen::Vector3d longitudeLatitudeToBearing(
    const Eigen::Vector2d& lon_lat);

Eigen::Vector2d bearingToEquirectangular(
    const Eigen::Vector3d& bearing,
    int width,
    int height);

Eigen::Vector3d equirectangularToBearing(
    const Eigen::Vector2d& pixel,
    int width,
    int height);
```

球面是几何对象，ERP 只是二维参数化。三维计算优先使用 bearing，不得把 ERP 像素距离直接当作均匀角度距离。

### 8.2 球面极线约束

核心公式：

\[
r=\mathbf b_2^T[\mathbf t]_{\times}\mathbf R\mathbf b_1
\]

核心函数：

```cpp
double epipolarAngularError(
    const Eigen::Vector3d& bearing_1,
    const Eigen::Vector3d& bearing_2,
    const Eigen::Matrix3d& R_2_1,
    const Eigen::Vector3d& t_2_1);
```

实际筛选匹配应使用归一化角度误差，不应直接用未归一化代数残差。

### 8.3 三角化

`TriangulationResult` 至少保存：

```cpp
struct TriangulationResult {
    bool valid = false;
    Eigen::Vector3d point_w = Eigen::Vector3d::Zero();
    double depth_1 = 0.0;
    double depth_2 = 0.0;
    double ray_angle = 0.0;
    double closest_ray_distance = 0.0;
    double reprojection_error = 0.0;
};
```

三角化后必须检查：正深度、射线夹角、最近距离和重投影误差。

---

## 9. 视觉前端

### 9.1 特征数据

```cpp
struct FeatureObservation {
    Timestamp timestamp = 0.0;
    CameraId camera_id = 0;
    Eigen::Vector2d pixel = Eigen::Vector2d::Zero();
    Eigen::Vector3d bearing = Eigen::Vector3d::Zero();
};

struct FeatureTrack {
    FeatureId id = 0;
    std::vector<FeatureObservation> observations;
    double inverse_depth = 0.0;
    double inverse_depth_variance = 0.0;
    bool depth_initialized = false;
};
```

不得只保存像素而丢失相机编号、时间戳和 bearing。

### 9.2 第一版视觉流程

```text
grid-based FAST/ORB detection
 -> same-camera temporal LK tracking
 -> cross-camera descriptor candidate matching
 -> spherical epipolar filtering
 -> bidirectional consistency
 -> triangulation
 -> inverse-depth update
```

先建立可解释的稀疏几何基线，再实现论文 HOFA 的多层半直接对齐。

### 9.3 逆深度滤波

每个地标至少保存：

```text
inverse depth mean
inverse depth variance
observation count
last reprojection error
confidence
```

错误匹配、遮挡和射线夹角过小的观测不得进入滤波更新。

---

## 10. ESKF 后端

名义状态至少包含：

```cpp
struct EskfState {
    Timestamp timestamp = 0.0;
    Eigen::Vector3d position_w_b;
    Eigen::Vector3d velocity_w_b;
    Eigen::Quaterniond rotation_w_b;
    Eigen::Vector3d accel_bias;
    Eigen::Vector3d gyro_bias;
    Eigen::Vector3d gravity_w;
    Eigen::Matrix<double, 15, 15> covariance;
};
```

实现顺序：

1. 静止状态初始化；
2. IMU中值积分；
3. 协方差传播；
4. 球面 bearing 视觉残差；
5. 测量雅可比；
6. Kalman 更新；
7. 误差注入；
8. 协方差重置。

不得使用 Ceres 或 GTSAM 替代论文的 ESKF 核心。它们可以用于离线验证或对比，但不能改变主算法结构。

---

## 11. ROS 接入设计

### 11.1 输入话题

默认话题名仅为占位符，必须从 YAML/ROS 参数读取：

```text
/camera0/image_raw   sensor_msgs/Image
/camera1/image_raw   sensor_msgs/Image
/camera2/image_raw   sensor_msgs/Image
/camera3/image_raw   sensor_msgs/Image
/imu/data_raw        sensor_msgs/Imu
```

### 11.2 同步原则

只同步四路图像，不把 IMU 放进五路同步器。

```text
IMU callback -> append to timestamp-ordered IMU buffer

four-image callback
 -> validate four timestamps
 -> obtain image reference time
 -> extract IMU interval from previous image time to current image time
 -> propagate ESKF to image time
 -> process visual frame
 -> visual update
```

硬件同步且时间戳完全一致时使用 `ExactTime`；否则先使用 `ApproximateTime`，同时主动检查四路最大时间差。同步器成功产生回调不等于数据时间差一定可接受。

### 11.3 图像生命周期

`cv_bridge::toCvShare()` 不复制图像。如果图像需要在回调结束后进入工作队列或异步线程，必须执行 `clone()`；否则 `cv::Mat` 可能引用已经释放的 ROS 消息内存。

### 11.4 IMU 缓冲

核心系统维护：

```cpp
std::deque<ImuMeasurement> imu_buffer_;
```

必须检查：

- 时间戳严格递增；
- 图像区间内有足够的 IMU；
- IMU 是否有异常大间隔；
- 是否需要插值到图像曝光时刻；
- 缓冲区是否无限增长。

### 11.5 ROS 输出

```text
/sphere_vio/odometry       nav_msgs/Odometry
/sphere_vio/path           nav_msgs/Path
/sphere_vio/landmarks      sensor_msgs/PointCloud2
/sphere_vio/debug/features sensor_msgs/Image
TF: world -> body
TF static: body -> camera0...camera3
```

调试图仅在配置开启时发布，ARM 实时运行时默认关闭。

---

## 12. ROS 第一阶段：Sensor Inspector

不要首先编写完整 VIO 节点。第一个 ROS 可执行程序为 `sensor_inspector_node`，只负责验证数据输入。

必须统计和打印：

```text
each image topic name
image width and height
image encoding
per-camera frame rate
four-camera maximum timestamp difference
IMU frequency
IMU timestamp monotonicity
dropped or delayed messages
```

可选发布一张缩小后的四目拼接图，用于检查相机顺序、曝光和同步。该节点不实现投影、匹配、三角化或 ESKF。

Sensor Inspector 验收标准：

- 四路相机顺序正确；
- 图像编码已明确；
- 图像分辨率稳定；
- 帧率符合设备规格；
- 时间戳来自曝光/硬件时钟而非回调到达时间；
- 四路时间差在可接受范围；
- IMU时间戳递增且频率稳定。

---

## 13. 参数文件组织

### `config/system.yaml`

```yaml
topics:
  camera0: /camera0/image_raw
  camera1: /camera1/image_raw
  camera2: /camera2/image_raw
  camera3: /camera3/image_raw
  imu: /imu/data_raw

frames:
  world: world
  body: body
  cameras: [camera0, camera1, camera2, camera3]

synchronization:
  image_queue_size: 10
  maximum_image_time_difference: 0.003
  imu_queue_size: 2000

frontend:
  maximum_features_per_camera: 250
  minimum_feature_distance: 15
  pyramid_levels: 4
  enable_debug_image: true

geometry:
  maximum_epipolar_angle: 0.002
  minimum_triangulation_angle: 0.01
  maximum_reprojection_error: 2.0

output:
  publish_tf: true
  publish_path: true
  publish_landmarks: true
```

完整相机内外参放入 `config/cameras.yaml`，不要写进 launch 文件，也不要假定四个镜头相同。

---

## 14. CMake 目标组织

不要把所有 CPP 直接编入 ROS 节点。建议最终目标为：

```text
sphere_vio_camera
sphere_vio_geometry
sphere_vio_frontend
sphere_vio_backend
sphere_vio_system
sensor_inspector_node
sphere_vio_node
```

依赖方向：

```text
camera -> geometry -> frontend -> system
backend -----------------------> system
system ------------------------> ROS nodes
```

初期至少使用：

```cmake
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O2 -Wall -Wextra)
```

禁止全局加入：

```text
-march=native
-mavx
-mavx2
-msse*
```

这些会破坏 ARM64 兼容性。第一阶段依赖 Eigen、OpenCV 和编译器自动向量化，后期再单独评估 NEON 优化。

---

## 15. 分阶段实施计划

### Phase 0：工程与输入检查

- 建立 Git 仓库；
- 整理 `.gitignore` 和 README；
- 建立分层目录；
- 实现 `sensor_inspector_node`；
- 确认真实话题、编码、帧率和时间戳。

验收：本包独立编译；ROS 能找到两个环境中的包；输入统计可靠。

建议提交：

```text
chore: initialize sphere_vio ROS package
feat: add multi-camera and IMU sensor inspector
```

### Phase 1：相机模型

- `CameraModel` 接口；
- Kannala–Brandt `project/unproject`；
- 有效像素判断；
- 投影闭环单元测试。

验收：有效像素 `pixel -> bearing -> pixel` 误差达到设定精度；覆盖中心、中间、边缘和随机样本。

建议提交：

```text
feat: add camera model abstraction and KB4 projection
test: add camera projection round-trip tests
```

### Phase 2：四目相机架与球面表示

- `CameraRig`；
- 外参方向统一；
- camera bearing 转 body bearing；
- bearing、经纬度和 ERP 双向映射；
- 初始化查找表。

验收：相邻相机中的同一标定点转换到机身坐标后方向一致；双向球面映射测试通过。

### Phase 3：球面极线与三角化

- 球面极线角度误差；
- 合成对应点测试；
- 两视图射线三角化；
- 正深度、射线角和重投影检查。

验收：无噪声合成数据恢复正确三维点；增加像素噪声后误差趋势合理；退化基线被拒绝。

### Phase 4：稀疏视觉前端

- 网格化特征检测；
- LK跨帧跟踪；
- 跨相机候选匹配；
- 极线、唯一性和双向一致性筛选；
- 逆深度初始化与更新。

验收：静态已知场景产生稳定稀疏点云；误匹配明显被剔除；输出置信度。

### Phase 5：IMU 与 ESKF

- IMU初始化；
- 状态和协方差传播；
- 球面视觉残差；
- 误差状态更新；
- ROS odometry 和 TF 输出。

验收：静止数据无明显发散；合成运动测试通过；rosbag轨迹可用 evo 评估。

### Phase 6：HOFA 与性能优化

- 图像金字塔；
- 深度范围引导；
- 半直接光度对齐；
- 跨相机联合深度观测；
- 四相机并行；
- ARM性能分析。

验收：精度优于基础ORB方案；ARM设备达到目标帧率且内存稳定。

---

## 16. 单元测试要求

每个几何模块必须先使用合成数据测试，再接入真实相机。

必须包含：

```text
camera projection round trip
invalid pixel and invalid ray handling
camera-to-body bearing transform
bearing <-> longitude/latitude round trip
bearing <-> ERP round trip with seam cases
known-correspondence epipolar residual
valid two-view triangulation
near-parallel-ray rejection
negative-depth rejection
inverse-depth update convergence
stationary IMU propagation
```

函数遇到无效输入时不得静默产生 NaN。应返回 `false`、无效结果结构或明确异常，并在单元测试中覆盖。

---

## 17. Codex 开发规则

Codex在修改本项目时必须遵守：

1. 先阅读本文档和当前 `README.md`；
2. 修改前检查工作树，保留用户现有更改；
3. 一次只实现一个可测试阶段；
4. 核心算法不得依赖ROS消息；
5. 不得默认鱼眼极线为水平直线；
6. 不得默认四相机拥有相同模型和参数；
7. 不得混淆 `R_a_b` 与 `R_b_a`；
8. 不得只输出深度而不保存有效性和置信度；
9. 不得添加 x86 专属编译选项；
10. 不得引入 CUDA 依赖；
11. 不得为了快速通过测试而降低几何检查标准；
12. 新模块必须有单元测试和简短文档；
13. 编译时只白名单 `sphere_vio`，避免影响工作空间其他包；
14. 不得提交 rosbag、数据集、构建目录、虚拟环境或生成二进制；
15. 完成修改后报告编译命令、测试结果和仍未验证的假设。

---

## 18. 当前下一任务

当前进度：

Phase 0 已完成：
- sensor_inspector_node
- 真实四目与 IMU 话题确认
- 离线 rosbag runner
- FrameAssembler
- IMU 区间提取
- 合成 rosbag 确定性测试

Phase 1：CameraModel 和 KB4 已完成合成数据验证。
真实相机标定尚未接入。

当前优先级为：

```text
1. 确认 Git 仓库边界位于 /root/catkin_ws/src/sphere_vio
2. 创建 docs/ 并保存本文档
3. 实现 sensor_inspector_node
4. 获取真实四目图像和 IMU 话题名称
5. 验证四路时间同步及 IMU 频率
6. 再实现 CameraModel 和 Kannala-Brandt
```

在真实输入话题、图像编码和标定模型尚未确认前，不要编写完整 `sphere_vio_node`，也不要假造相机参数。

---

## 19. 参考目标

本项目参考 Sphere-VIO 的总体思想，但当前公开论文未提供官方实现。复现时应把论文结论与本项目自行设计清楚区分。项目首先建立正确、可测试、跨架构的工程基线，再逐步逼近论文的 USPM、HOFA 和球面 bearing ESKF。

论文：

```text
Sphere-VIO: Fast and Robust Visual-Inertial Odometry via
Unified Spherical Representation for Heterogeneous Multi-Camera Systems
arXiv:2606.29910
https://arxiv.org/abs/2606.29910
```
