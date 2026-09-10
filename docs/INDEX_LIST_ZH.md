# 测试指标列表
> 此表中是在数据集上进行测试需要输出的指标。
> 在数据集上测试过程中，建result文件夹，每次测试后写报告文件。
> 文件内容：测试各指标数据，算法改动点，关键参数设置。

## 一、定位精度指标（必须项）

| 指标名 | 定义/计算方式 | 备注 |
|--------|--------------|------|
| `ATE_SE3` | 估计轨迹与GT经SE(3)对齐（尺度固定=1）后的绝对轨迹误差RMSE | 主指标 |
| `ATE_Sim3` | 仅作参考，不用于主评判 | 需同时报告但注明非主指标 |
| `RPE_1m` | 间隔1m的相对位姿误差（平移+旋转） | |
| `RPE_5m` | 间隔5m的相对位姿误差 | |
| `RPE_10m` | 间隔10m的相对位姿误差 | |
| `RPE_R` | 若GT无姿态，填 `N/A` | 明确输出 |

**数据输出格式要求**：  
每帧估计结果必须输出为：
```
timestamp, x, y, z, qx, qy, qz, qw
```
（所有序列统一）

**时间戳处理规则**：
- GT插值到估计时间戳（禁止最近帧匹配）

---

## 二、鲁棒性与有效性指标（全序列统计，不截断）

| 指标名 | 计算方式 | 合格门槛 |
|--------|----------|----------|
| `success` | 程序是否无崩溃/死锁（整个序列） | 是/否 |
| `coverage` | 有效输出时间 / GT总时间 | >= 90% |
| `max_pose_interval` | 相邻有效位姿的最大时间间隔 | <= 2 × 输出周期 |
| `has_nan_inf` | 是否存在NaN/Inf或非法四元数 | 必须为 `False` |
| `failure_time` | 若中途失败，记录 `FAIL@时间点` | 用于汇总 |

**最终汇总只报告**：
```
成功率（所有序列） + 平均覆盖率 + ATE_SE3 + RPE_1m/5m/10m
```
**不得**仅对成功序列求平均。

---

## 三、性能指标（本机，仅作为参考）

| 指标名 | 定义 |
|--------|------|
| `RTF` | 数据时长 / 实际运行时长，RTF>=1表示实时 |
| `cpu_core_hours_per_frame` | CPU核时 / 输入帧数（比CPU%更稳定） |
| `avg_frame_time_ms` | 平均每帧处理耗时 |
| `frame_step` | 后端降频步长（当前=2，不记为丢帧） |
| `dropped_frames` | 真正未进入算法处理的输入帧数（不含frame_step） |

**当前实时预算**：16Hz输入 => 单帧预算 **62.5 ms**。

---

## 四、RK3588 部署性能指标（最终板上实测）

| 指标名 | 说明 |
|--------|------|
| `features_per_frame` | 每帧提取的特征数 |
| `match_candidates` | 匹配候选数 |
| `triangulation_candidates` | 三角化候选数 |
| `effective_landmarks` | 有效landmark数 |
| `backend_residuals` | 后端残差数量 |
| `optimization_iterations` | 优化迭代次数 |
| `sliding_window_size` | 滑动窗口大小 |
| `rk3588_avg_ms_per_frame` | RK3588上平均每帧耗时 |
| `rk3588_rtf` | 基于RK3588实际运行时长计算的RTF |

---

## 五、最终汇总表（仅SphereVIO）

**表1 – 定位精度（本机）**  
列：`序列名, ATE_SE3, ATE_Sim3(参考), RPE_1m, RPE_5m, RPE_10m, RPE_R, coverage, success, failure_time`

**表2 – 运行性能（本机）**  
列：`序列名, RTF, cpu_core_hours_per_frame, avg_frame_time_ms, frame_step, dropped_frames`

**表3 – RK3588实测**  
列：`序列名, features_per_frame, match_candidates, triangulation_candidates, effective_landmarks, backend_residuals, optimization_iterations, sliding_window_size, rk3588_avg_ms_per_frame, rk3588_rtf`
