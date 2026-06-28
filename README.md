# tof_ros

基于 TMF8829 dToF 传感器 + 光流模块的低成本室内定位 ROS1 包。

## 系统概览

**目标**：TMF8829 点云 GICP 定位 + 光流里程计融合，实现无 IMU 的低成本室内 2D 定位。

**运行环境**

- ROS 1 Noetic + Ubuntu 20.04
- C++14，依赖 PCL、Eigen

**硬件**

| 设备 | 接口 |
|------|------|
| Upixels UP-T1-001 光流模块 | USB-TTL → `/dev/ttyUSB0`，115200 8N1 |
| AMS TMF8829 dToF 传感器 | UDP → 端口 55320 |

---

## 节点架构

```
/dev/ttyUSB0 ──► [oflow_odometry]  ──► /oflow/pose (PoseStamped)
                        ▲
                        │ 修正积分基点 + 朝向
                        │
UDP:55320    ──► [tof_receiver]    ──► /tof/points (PointCloud2)
                                   └── /tof/depth  (Image 32FC1)
                        │
                        ▼
              [tof_localizer]      ──► /tof/pose   (PoseStamped)
                                   └── /tof/aligned (PointCloud2)
                                   └── TF: map → tof_frame
```

---

## 编译

```bash
cd ~/tof_ws
catkin_make
source devel/setup.bash
```

**依赖**

```bash
sudo apt install ros-noetic-pcl-ros ros-noetic-pcl-conversions \
                 ros-noetic-tf2-ros ros-noetic-sensor-msgs
```

**串口权限**（光流模块）

```bash
sudo usermod -aG dialout $USER   # 重新登录后生效
# 或临时授权：
sudo chmod 666 /dev/ttyUSB0
```

---

## 节点说明

### tof_receiver

接收 TMF8829 UDP 数据帧，解析 48×32 深度图，发布点云和深度图。

**订阅**：无

**发布**

| Topic | 类型 | 说明 |
|-------|------|------|
| `/tof/points` | `sensor_msgs/PointCloud2` | 过滤 + SOR 去噪后的点云 |
| `/tof/depth` | `sensor_msgs/Image` (32FC1) | 原始深度图（米），无效点为 NaN |

**参数**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `conf_threshold` | 5 | 置信度阈值（0-255），低于此值丢弃 |
| `dist_min_m` | 0.10 | 最小有效距离（米），过滤近处遮挡 |
| `dist_max_m` | 5.7 | 最大有效距离（米） |
| `sor_neighbors` | 10 | SOR 滤波 k 近邻数 |
| `sor_std_ratio` | 2.0 | SOR 标准差倍数阈值，越小过滤越激进 |

**UDP 帧格式**（版本 2）

```
TofUdpHeader (28 字节):
  magic        : 0x544F4600 ("TOF\0")
  version      : 2
  mode         : 0x04 (48×32)
  frame_id     : uint32，单调递增（用于检测丢帧）
  timestamp_us : uint64，CLOCK_MONOTONIC 微秒
  cols / rows  : 48 / 32
  payload_bytes: cols × rows × 3

TofZone (3 字节 × 1536):
  dist_mm : uint16，单位 0.25mm/LSB（0 = 无效）
  conf    : uint8（0 = 无效）

每帧总大小：28 + 4608 = 4636 字节（IP 层自动分 4 片传输）
```

---

### tof_localizer

订阅 `/tof/points`，与先验 PCD 地图做 GICP/ICP 匹配，输出传感器在地图中的位姿。

**订阅**

| Topic | 类型 |
|-------|------|
| `/tof/points` | `sensor_msgs/PointCloud2` |

**发布**

| Topic | 类型 | 说明 |
|-------|------|------|
| `/tof/pose` | `geometry_msgs/PoseStamped` | GICP 定位结果 |
| `/tof/aligned` | `sensor_msgs/PointCloud2` | 对齐后的当前帧点云（调试用） |
| TF `map → tof_frame` | - | 位姿变换 |

**参数**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map_path` | `/home/enhe/tof_ws/scans_2.pcd` | 先验地图文件路径 |
| `voxel_size` | 0.10 | 地图体素下采样边长（米） |
| `init_x/y/z` | 0.0 | 传感器初始位置（米），放在建图原点时均为 0 |
| `init_heading_deg` | 0.0 | 传感器前向（z 轴）在地图中的初始朝向角，+x=0°，逆时针为正 |
| `accumulate_frames` | 1 | 多帧累积数量，增大可改善点云稀疏场景的匹配质量 |
| `max_corr_dist` | 1.5 | GICP 最大匹配点对距离（米） |
| `max_fitness` | 0.02 | fitness score 上限，超过则拒绝本帧更新 |
| `max_jump` | 0.5 | 帧间最大允许位移（米），超过视为跳变拒绝 |
| `matcher_type` | `gicp` | 匹配算法：`gicp` 或 `icp_ptp`（Point-to-Plane） |

**坐标系约定**

- `tof_frame`：z 轴朝前（光轴方向），x 轴朝右，y 轴朝下
- `map`：FAST-LIO2 建图坐标系（x 前，y 左，z 上）
- 节点内部 `R_mount` 自动完成两个坐标系的转换

**注意**：不要为 `g_T` 添加基于速度的运动预测。GICP 小噪声叠加会导致静置漂移。改善收敛请优先调大 `accumulate_frames` 或减小 `voxel_size`。

---

### oflow_odometry

读取光流模块串口数据（50Hz），积分计算 XY 位移，发布里程计位姿。订阅 `/tof/pose` 时自动修正积分基点和朝向。

**订阅**

| Topic | 类型 | 说明 |
|-------|------|------|
| `/tof/pose` | `geometry_msgs/PoseStamped` | ToF 定位成功时修正积分基点 + 更新 heading |

**发布**

| Topic | 类型 |
|-------|------|
| `/oflow/pose` | `geometry_msgs/PoseStamped` |
| `/oflow/path` | `nav_msgs/Path` |

**参数**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `port` | `/dev/ttyUSB0` | 串口设备路径 |
| `height_override` | -1.0 | >0：强制使用固定高度（米）；<0：使用模块自带激光测距 |
| `init_heading_deg` | 0.0 | 无 ToF 数据时的初始朝向假设 |
| `mount_yaw_deg` | 0.0 | 光流模块机头（Y 轴）相对于 ToF 前向的安装偏角（度） |
| `flow_sign_x` | 1 | 光流 X 方向符号，实测方向相反时改为 -1 |
| `flow_sign_y` | 1 | 光流 Y 方向符号 |
| `min_laser_conf` | 10 | 激光测距最低置信度，低于此值丢弃该帧 |

**串口帧格式**（Upixels 出厂协议，14 字节）

```
0xFE | 0x0A | flow_x(int16) | flow_y(int16) | dt_us(uint16) |
dist_mm(uint16) | valid(uint8) | lconf(uint8) | XOR | 0x55

flow_x/y 单位：弧度 × 10000（一帧累积角位移）
valid = 0xF5 表示有效；上电约 3 秒后才输出有效数据
```

**位移计算**

```
dx_sensor = flow_x / 10000 × height_m
dy_sensor = flow_y / 10000 × height_m
θ = heading + mount_yaw_rad
dx_map =  dx_s·sin(θ) + dy_s·cos(θ)
dy_map = -dx_s·cos(θ) + dy_s·sin(θ)
```

---

## Launch 文件

### 仅查看 TMF8829 点云

```bash
roslaunch tof_ros tof_receiver.launch
```

### TMF8829 GICP 定位（含 RViz）

```bash
roslaunch tof_ros tof_localizer.launch \
  map_path:=/home/enhe/tof_ws/scans_2.pcd \
  init_heading_deg:=0.0
```

### 仅光流里程计（含地图可视化 + RViz）

```bash
roslaunch tof_ros oflow_test.launch
```

---

## 工具脚本

### tof_dist_check.py — 测距精度验证

将传感器对准已知距离的平面，运行此脚本观察实时统计。

```bash
# 先启动接收节点
roslaunch tof_ros tof_receiver.launch

# 另一个终端运行脚本
python3 src/tof_ros/scripts/tof_dist_check.py
```

输出示例：
```
有效= 384/1536  min=0.912m  mean=0.998m  median=1.001m  max=1.043m  std=0.018m
```

---

## 带宽估算（无线传输参考）

| 数据流 | 帧率 | 每帧大小 | 带宽 |
|--------|------|----------|------|
| TMF8829 | 5 Hz | 4636 B | ~189 kbps |
| 光流模块 | 50 Hz | 14 B | ~17 kbps |
| **合计** | - | - | **≈ 206 kbps** |

注意：每帧 4636 字节超过以太网 MTU 1500，IP 层会自动分 4 片。无线传输时任一分片丢失则整帧丢弃，建议发送端实现应用层分包。

---

## 地图文件

使用 FAST-LIO2 建图生成的 PCD 文件，默认路径：

```
/home/enhe/tof_ws/scans_2.pcd
```

`scans.pcd` 为旧版地图，请勿使用。

---

## 待实现

- [ ] 节点 C（pose_arbiter）：融合 `/tof/pose` 和 `/oflow/pose`，输出 `/robot/pose`
- [ ] tof_localizer 增加 `/tof/reset_pose` 订阅，支持位姿重置
- [ ] 光流方向校准（实测调整 `flow_sign_x/y`、`mount_yaw_deg`）
