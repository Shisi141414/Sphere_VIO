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
    TriangulationStatus status = TriangulationStatus::kInvalidInput;
    Eigen::Vector3d point_common = Eigen::Vector3d::Zero();
    double depth_1 = 0.0;
    double depth_2 = 0.0;
    double ray_angle = 0.0;
    double closest_ray_distance = 0.0;
    double maximum_angular_reprojection_error = 0.0;
};
```

`point_common` 的坐标系由接口明确指定，不能在没有世界位姿时命名为
`point_w`。三角化后必须检查：正深度、射线夹角、最近距离和重投影误差。

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

Phase 1.5：真实四目投影模型采用 Kalibr `omni` + `radtan`。权威标定来源为：

```text
/root/catkin_ws/src/seeker_utils/config/seeker1/kalibr_cam_chain.yaml
```

Kalibr 中的 `/fisheye/*/image_raw/compressed` 与真实 rosbag 中的
`/fisheye/*/image_raw` 是经 seeker launch/relay 映射的同一基础图像流；
相机核心模型不处理 ROS 话题或 image transport。

项目 CameraId 与 Kalibr 相机编号必须显式映射：

```text
project C0 left   <- Kalibr cam0 left
project C1 right  <- Kalibr cam1 right
project C2 bleft  <- Kalibr cam3 bleft
project C3 bright <- Kalibr cam2 bright
```

不得按数组下标直接复制 Kalibr 相机。真实参数按上述项目顺序保存在
`config/cameras.yaml`。Kalibr cam2 和 cam3 未提供 `timeshift_cam_imu`，
对应项目 C3 和 C2 使用 `null` 与 `timeshift_available: false` 表示不可用，
不得默认为零。时间偏移不参与本阶段的 `project()` 或 `unproject()`，应在
后续 VIO 时间对齐阶段单独处理。

Phase 2：`CameraRig`、真实外参与统一 Body bearing 已完成。当前约定 B 与
IMU 坐标系对齐。`config/cameras.yaml` 保存权威 Kalibr 文件中的原始
`T_cam_imu`，其语义为：

```text
p_c = R_c_b * p_b + t_c_b
```

加载时显式求逆并在 `CameraRig` 中保存：

```text
R_b_c = R_c_b.transpose()
t_b_c = -R_b_c * t_c_b
```

方向变换只使用旋转并显式归一化：

```text
bearing_b = normalize(R_b_c * bearing_c)
```

三维点变换则使用旋转和平移。项目与 Kalibr 相机编号仍严格采用
`C0/C1/C2/C3 <- cam0/cam1/cam3/cam2`。`T_cn_cnm1` 不用于构建 Rig，
只用于验证 `T_cam_imu` 推导出的相邻相机变换。缺失的时间偏移继续保持
unavailable，尚未参与时间对齐。本阶段没有开始球面极线、特征、深度估计、
三角化或 ESKF。

Phase 2B：基础球面表示与 ERP 参数化已实现。所有输入均为 Body 坐标系
bearing，采用：

```text
longitude = atan2(y, x)                         in [-pi, pi)
latitude  = atan2(z, sqrt(x*x + y*y))           in [-pi/2, pi/2]
```

longitude 是绕 z 轴的方位角，latitude 是相对 x-y 平面的纬度角；这里不额外
假定某一经度对应车辆正前方。正负极点的 longitude 数学上不唯一，转换时固定
为 0，确保结果确定且有限。

ERP 使用连续几何坐标：

```text
u = width  * (longitude + pi) / (2*pi)          in [0, width)
v = height * (pi/2 - latitude) / pi             in [0, height]
```

水平方向周期化，`u=width` 与 `u=0` 等价；没有使用 `width-1` 或
`height-1`。ERP 坐标不是可直接访问 `cv::Mat` 的整数索引，调用方仍需明确
处理水平 wrap、垂直边界和像素中心约定。三维几何继续直接使用单位 bearing，
经纬角和 ERP 目前只用于参数化、索引与调试可视化。

离线 visualizer 的可选 `--show-spherical-coverage` 面板预计算四相机稀疏
`pixel -> bearing_c -> bearing_b -> ERP` 覆盖点，只显示缝合线、赤道与相机
中心方向；不进行图像拼接、重叠混合或深度估计。Phase 2B 当时尚未实现
球面极线、三角化或深度。

Phase 3A：球面极线几何与合成对应验证已实现。相对位姿继续使用：

```text
p_2 = R_2_1 * p_1 + t_2_1
R_2_1 = R_b_c2.transpose() * R_b_c1
t_2_1 = R_b_c2.transpose() * (t_b_c1 - t_b_c2)
```

因此 `t_2_1` 是源相机中心相对目标相机中心的位置，并表达在目标相机 C2
坐标系中。球面代数约束和目标相机中的极线平面法向量分别为：

```text
r = bearing_2.transpose() * skew(t_2_1) * R_2_1 * bearing_1
n_hat = normalize(t_2_1.cross(R_2_1 * bearing_1))
```

代数残差随基线尺度变化，仅用于调试。用于未来筛选的归一化角度误差为
`abs(asin(clamp(bearing_2.dot(n_hat), -1, 1)))`，单位为 radian；对称误差分别
用正向位姿及 `R_1_2=R_2_1.transpose()`、`t_1_2=-R_1_2*t_2_1` 计算正反
角度，并明确保存 maximum 和 average。

极线是单位球面与过两相机中心及源射线的平面之交，即一条大圆。实现通过
稳定选择辅助坐标轴构造平面内正交基，并在 `[0,2*pi)` 采样单位 bearing，
不重复首尾点。零或极小基线、非法旋转，以及基线与旋转后源射线近似平行时
极线平面不唯一，接口显式失败。

当前验证只使用解析合成点以及真实四目内外参生成的合成可见对应点，尚未使用
真实图像特征对应。visualizer 的可选 `--show-epipolar-curve` 模式只把标定
计算的大圆有效部分投影到目标相机，不寻找对应点。Phase 3A 当时尚未实现
自动匹配、三角化、深度或逆深度。

Phase 3B：两视图球面三角化已实现。通用接口在调用方指定的公共坐标系中表示
两条射线：

```text
p_1(lambda_1) = origin_1 + lambda_1 * direction_1
p_2(lambda_2) = origin_2 + lambda_2 * direction_2
```

实现构造 `A=[direction_1,-direction_2]` 和 `b=origin_2-origin_1`，使用
`Eigen::ColPivHouseholderQR` 求最小二乘深度，不显式计算法方程逆矩阵。两条
最近射线点的中点作为 `point_common`。相对位姿包装输出目标相机 C2 坐标，
CameraRig 包装把相机 bearing 旋转到 Body 并输出 `point_b`（保存在明确标记为
Body 的 `point_common`）。

`depth_1` 和 `depth_2` 是沿归一化 bearing 的射线参数，正深度检查直接比较
两个 lambda，不能用点的 z 坐标代替，因为全向相机的有效射线可以具有负 z。
`ray_angle=acos(clamp(direction_1.dot(direction_2),-1,1))`；接近平行或反平行
时两射线深度病态，由 `minimum_ray_angle` 拒绝。深距离通常对应更小视差角，
相同角噪声会产生更大的深度误差。

噪声射线一般不严格相交，`closest_ray_distance` 是两个最接近射线点之间的
欧氏距离。恢复中点分别指回两个相机中心后，与输入 bearing 计算球面角距离，
得到两路 angular reprojection error。最近距离、最大角重投影误差、最小
baseline、最小 ray angle 和最小 depth 都由 `TriangulationOptions` 配置，
当前没有固定最终真实匹配阈值。

全向 OmniRadtan 不满足已校正水平针孔双目的假设，因此没有使用
`focal_length*baseline/disparity`，也不把横向像素差当作视差。当前只通过
解析交点和真实四目内外参生成的合成对应验证；尚未接入自动真实图像匹配、
逆深度滤波、多帧优化或 VIO。

Phase 4A：单相机网格特征检测与同相机时序跟踪已实现。纯算法库
`sphere_vio_frontend` 使用分网格 FAST，并对候选点执行边界、全图最小距离和
相机模型域检查。第一帧只检测；后续帧先用金字塔 LK 做同一相机的前向与反向
光流，再按 LK error、边界和 forward-backward error 分类过滤，轨迹数量低于
配置比例时才补充检测。

每个新检测点和成功跟踪点都执行：

```text
pixel -> bearing_c -> bearing_b
```

四路相机分别保存上一帧图像、时间戳和 active tracks，互不共享时序状态。
`FeatureId` 在一个前端实例内全局递增，当前仅表示某一相机中的时序轨迹，不能
解释为跨相机 landmark。每条轨迹只保存 current 与 previous 两个观测，不保留
无限历史。当前没有建立跨相机特征关联，也没有对真实图像点执行三角化、深度或
逆深度估计。`config/system.yaml` 中的 Phase 4A 参数仍是初始调试参数，尚未
针对最终运动与光照条件完成调优。

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
