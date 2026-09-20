# romujoco LiDAR 组件完整设计方案

## 1. 设计目标与总体架构

### 1.1 目标

重构 `romujoco` 当前 LiDAR 实现，解决现有设计必须依赖大量 MuJoCo `<rangefinder>` sensor 的问题，并建立能够长期支持常见 2D / 3D LiDAR 的统一模型。

设计目标：

* 支持 2D 平面激光雷达；
* 支持常见 16 / 32 / 64 / 128 线 3D LiDAR；
* 支持非均匀垂直通道角；
* 支持每个通道独立 azimuth offset；
* 支持 180°、270°、360° 和 partial-FOV 扫描；
* 支持雷达任意安装姿态；
* MJCF 只负责描述传感器安装位置和方向，不展开 beam；
* 由 `romujoco` 使用 MuJoCo ray casting 完成实际扫描；
* 2D 输出语义参考 ROS 2 `sensor_msgs/msg/LaserScan`；
* 3D 输出语义参考 ROS 2 `sensor_msgs/msg/PointCloud2` / `PointField`；
* `romujoco` 不依赖 ROS 2；
* 不在公共接口引入 `std::variant`；
* 不建立 `Lidar2DComponent` / `Lidar3DComponent` 两套重复实现。

ROS 2 的 `LaserScan` 本身就是面向 planar laser range finder 的消息，定义了扫描角度、时间、量程、ranges 和 intensities，并规定零角沿 +X、正角绕 +Z 逆时针；设备不提供 intensity 时数组应为空。

ROS 2 对 3D 数据没有定义专门的 `Lidar3D` 消息，而是使用 `PointCloud2` 表示 N 维点集合，其布局由 `PointField[]` 描述，并同时支持 organized 和 unordered point cloud。

### 1.2 总体架构

```text
MJCF
 └── site
      │
      │ position / orientation
      ▼
LidarComponent
 ├── LidarInfo
 │    ├── scan geometry
 │    ├── channels
 │    ├── range
 │    └── output
 │
 ├── compile ray pattern
 │
 │    azimuth × channel
 │           ↓
 │    local_directions_
 │
 ├── site pose
 │           ↓
 │    world_directions_
 │
 ├── mj_multiRay()
 │           ↓
 │       distances_
 │
 └── formatter
      ├── LaserScan
      └── PointCloud2
```

核心原则：

> `LidarComponent` 本身不知道“2D 算法”和“3D 算法”。

它只处理：

```text
传感器坐标系
+
Ray Pattern
+
Ray Cast
```

2D 是只有一个 elevation=0 通道的特殊情况；3D 则是多个 elevation channel。

### 1.3 V1 能力边界

| 能力                               |  V1 |
| -------------------------------- | --: |
| 2D LiDAR                         |  支持 |
| 多线旋转式 3D LiDAR                   |  支持 |
| 非均匀 vertical channels            |  支持 |
| channel azimuth correction       |  支持 |
| partial FOV                      |  支持 |
| 固定规则 solid-state pattern         |  支持 |
| Organized PointCloud2            |  支持 |
| 强度 intensity                     | 不支持 |
| Multi-echo                       | 不支持 |
| Beam divergence                  | 不支持 |
| 噪声模型                             | 不支持 |
| 每通道独立 ray origin                 | 不支持 |
| Livox 式非重复动态 pattern             | 不支持 |
| Rolling scan / motion distortion | 不支持 |

本设计中的“雷达”特指 LiDAR，不包含毫米波 Radar。

---

## 2. 公共数据模型

### 2.1 LiDAR 配置模型

建议删除当前：

```cpp
std::string sensor_prefix;

double angle_min;
double angle_max;
double angle_increment;
```

改为：

```cpp
using LidarId = std::size_t;

enum class LidarOutput {
    LaserScan,
    PointCloud2,
};

struct LidarChannel {
    double elevation{0.0};
    double azimuth_offset{0.0};
};

struct LidarInfo {
    LidarId id{0};

    std::string name;
    std::string frame_id;
    std::string site_name;

    LidarOutput output{LidarOutput::LaserScan};

    double period{0.1};

    double azimuth_start{0.0};
    double azimuth_increment{0.0};
    std::uint32_t azimuth_samples{0};

    std::vector<LidarChannel> channels;

    double range_min{0.0};
    double range_max{0.0};

    std::uint32_t geom_group_mask{0};
    bool exclude_parent_body{true};
};
```

其中：

### `site_name`

对应 MuJoCo：

```xml
<site name="lidar_scan_frame"/>
```

表示 ray origin 和 LiDAR 坐标系。

### `frame_id`

表示对外发布数据对应的逻辑坐标系。

通常：

```text
site_name == frame_id
```

但二者职责不同，因此不应合并。

### `azimuth_start`

第一列 ray 的水平角。

坐标约定：

```text
0 rad → +X
+angle → 绕 +Z 逆时针
```

与 ROS `LaserScan` 保持一致。

### `azimuth_increment`

相邻水平采样之间的角度差。

允许正值或负值，但：

```text
azimuth_increment != 0
```

### `azimuth_samples`

显式保存采样数量，而不是继续使用：

```text
angle_min + angle_max + increment
```

反推数量。

这样可以避免 360°：

```text
-π
...
+π
```

重复首尾 ray。

完整 360° 应表达为：

```cpp
azimuth_start = -pi;
azimuth_increment = 2.0 * pi / samples;
azimuth_samples = samples;
```

### `LidarChannel`

每个通道：

```cpp
struct LidarChannel {
    double elevation;
    double azimuth_offset;
};
```

其中：

```text
elevation
```

表示相对 XY 平面的垂直发射角。

```text
azimuth_offset
```

表示该 channel 相对于基础 azimuth 的水平标定偏移。

2D：

```cpp
channels = {
    {0.0, 0.0}
};
```

3D：

```cpp
channels = {
    {-0.26, 0.0},
    {-0.21, 0.0},
    ...
    { 0.26, 0.0},
};
```

不使用：

```cpp
vertical_min
vertical_max
vertical_increment
```

因为真实多线 LiDAR 的 elevation 通常不保证均匀。

---

### 2.2 2D 输出：LaserScan

定义：

```cpp
struct LaserScan {
    std::uint64_t timestamp{0};
    std::string frame_id;

    float angle_min{0.0F};
    float angle_max{0.0F};
    float angle_increment{0.0F};

    float time_increment{0.0F};
    float scan_time{0.0F};

    float range_min{0.0F};
    float range_max{0.0F};

    std::vector<float> ranges;
    std::vector<float> intensities;
};
```

字段类型和语义尽量对应：

```text
sensor_msgs/msg/LaserScan
```

ROS 标准本身使用 `float32`，因此输出数据使用 `float`；配置与内部 ray 数学仍然使用 `double/mjtNum`。

时间戳使用：

```cpp
std::uint64_t
```

表示 simulation timestamp，单位固定为纳秒。

其语义与 ROS Header stamp 一致：

> 一帧扫描的采集时间。

当前 V1 为瞬时扫描，因此所有 ray 对应同一个 simulation snapshot。

所以：

```cpp
time_increment = 0.0F;
scan_time = static_cast<float>(info.period);
```

当前不模拟 intensity：

```cpp
intensities.clear();
```

不能填充：

```cpp
{0, 0, 0, ...}
```

因为 ROS 明确定义设备没有 intensity 时应保留为空数组。

状态包装：

```cpp
struct LaserScanState {
    LidarId id{0};
    std::uint64_t sequence{0};
    LaserScan scan;
};
```

---

### 2.3 3D 输出：PointCloud2

首先对齐 ROS 2 Humble `PointField`：

```cpp
enum class PointFieldType : std::uint8_t {
    Int8 = 1,
    UInt8 = 2,
    Int16 = 3,
    UInt16 = 4,
    Int32 = 5,
    UInt32 = 6,
    Float32 = 7,
    Float64 = 8,
};

struct PointField {
    std::string name;
    std::uint32_t offset{0};
    PointFieldType datatype{PointFieldType::Float32};
    std::uint32_t count{1};
};
```

这些数值与 ROS 2 Humble `sensor_msgs/msg/PointField` 保持一致。

PointCloud：

```cpp
struct PointCloud2 {
    std::uint64_t timestamp{0};
    std::string frame_id;

    std::uint32_t height{0};
    std::uint32_t width{0};

    std::vector<PointField> fields;

    bool is_bigendian{false};

    std::uint32_t point_step{0};
    std::uint32_t row_step{0};

    std::vector<std::uint8_t> data;

    bool is_dense{false};
};
```

这与 ROS `PointCloud2` 的结构保持一致：`height/width` 描述点云组织结构，`fields` 描述二进制点布局，`point_step` 和 `row_step` 描述字节跨度。

状态：

```cpp
struct PointCloudState {
    LidarId id{0};
    std::uint64_t sequence{0};
    PointCloud2 cloud;
};
```

### V1 PointField

固定为：

```cpp
fields = {
    {"x", 0, PointFieldType::Float32, 1},
    {"y", 4, PointFieldType::Float32, 1},
    {"z", 8, PointFieldType::Float32, 1},
};

point_step = 12;
```

ROS 2 PointCloud2 工具同样把 `x/y/z` 作为标准的 FLOAT32 point fields 使用。

暂不加入：

```text
intensity
ring
time
reflectivity
ambient
range
```

其中 intensity 当前没有真实模拟能力；ring 则可以通过 organized cloud 的 row 得到。

3D LiDAR 输出：

```cpp
cloud.height =
    static_cast<std::uint32_t>(info.channels.size());

cloud.width =
    info.azimuth_samples;
```

因此：

```text
row    = channel
column = azimuth sample
```

符合 `PointCloud2` 对 organized cloud 的标准能力。

---

### 2.4 不建立统一 LidarState

不采用：

```cpp
struct LidarState {
    LaserScan scan;
    PointCloud2 cloud;
};
```

也不采用：

```cpp
std::variant<LaserScanState, PointCloudState>
```

而是：

```text
LidarComponent
    ↓
根据 LidarOutput
    ↓
LaserScanState
或
PointCloudState
```

`RobotState` 相应维护：

```cpp
std::vector<LaserScanState> laser_scans;
std::vector<PointCloudState> point_clouds;
```

同一个 `LidarId` 命名空间仍然属于所有 LiDAR，不因输出类型重新编号。

这样：

* 不暴露 `variant`；
* 不存在一半字段永远无效的“大状态结构”；
* 数据语义与 ROS 保持清晰对应。

---

## 3. 配置与验证模型

### 3.1 2D LiDAR XML

示例，仅表示格式，不代表 NanoScan3 的最终标定参数：

```xml
<lidar
    id="0"
    name="lidar_front"
    frame_id="lidar_front_scan_frame"
    site="lidar_front_scan_frame"
    output="laser_scan"
    period="0.05"
    range_min="0.05"
    range_max="40.0">

  <scan
      azimuth_start="-2.35619449"
      azimuth_increment="0.004363323"
      azimuth_samples="1081"/>

  <channel
      elevation="0.0"
      azimuth_offset="0.0"/>

</lidar>
```

MFR3Duo 可以直接绑定已经存在的：

```text
lidar_front_scan_frame
lidar_rear_scan_frame
```

不再要求 MJCF 中存在：

```text
lidar_front-0
lidar_front-1
...
```

等大量 `rangefinder` sensor。

---

### 3.2 3D LiDAR XML

```xml
<lidar
    id="2"
    name="lidar_top"
    frame_id="lidar_top_scan_frame"
    site="lidar_top_scan_frame"
    output="point_cloud2"
    period="0.1"
    range_min="0.5"
    range_max="120.0">

  <scan
      azimuth_start="-3.141592654"
      azimuth_increment="0.003067962"
      azimuth_samples="2048"/>

  <channel elevation="-0.261799" azimuth_offset="0.0"/>
  <channel elevation="-0.174533" azimuth_offset="0.0"/>
  <channel elevation="-0.087266" azimuth_offset="0.0"/>
  <channel elevation="0.000000" azimuth_offset="0.0"/>
  <channel elevation="0.087266" azimuth_offset="0.0"/>
  <channel elevation="0.174533" azimuth_offset="0.0"/>
  <channel elevation="0.261799" azimuth_offset="0.0"/>

</lidar>
```

不增加：

```xml
type="2d"
type="3d"
```

而使用：

```text
output="laser_scan"
output="point_cloud2"
```

因为设备的核心扫描模型是统一的，区别在于公开数据契约。

---

### 3.3 Geometry filtering

默认：

```cpp
geom_group_mask == 0
```

定义为：

> 不进行 geom group filtering，扫描全部 MuJoCo geom groups。

如果配置过滤，例如只扫描 group 3：

```xml
<raycast
    geom_groups="3"
    exclude_parent_body="true"/>
```

Parser 转换成内部 bit mask。

`exclude_parent_body=true` 时：

```cpp
bodyexclude = model.site_bodyid[site_id];
```

防止 LiDAR 直接命中自己所在 body 的 housing。

但不默认排除整个机器人，因为机械臂、车体、支架对雷达形成遮挡本身可能是真实现象。

---

### 3.4 配置验证

初始化 MuJoCo model 前可以完成：

```text
name 非空
frame_id 非空
site_name 非空

period finite && period > 0

azimuth_start finite
azimuth_increment finite
azimuth_increment != 0
azimuth_samples > 0

channels 非空

channel.elevation finite
channel.azimuth_offset finite
elevation ∈ [-π/2, π/2]

range_min finite
range_max finite
0 <= range_min < range_max
```

对于：

```text
output = LaserScan
```

额外要求：

```text
channels.size() == 1
channel.elevation == 0
```

`azimuth_offset` 可以非零，最终直接合并进 LaserScan 的起始角。

对于：

```text
output = PointCloud2
```

允许任意正数量 channel。

Ray 数量：

```cpp
ray_count =
    channels.size() * azimuth_samples;
```

必须：

* 无整数溢出；
* `ray_count <= INT_MAX`，因为 MuJoCo `mj_multiRay()` 的 `nray` 使用 `int`；
* `3 * ray_count` 的 direction buffer 大小必须可表示。

不设置人为的 65536 等 beam 数量限制，因为 128 × 2048 本身就已经超过这一数值。

加载 MuJoCo model 后再验证：

```text
site 存在
site 类型正确
site body id 有效
geom group 配置合法
```

---

## 4. LidarComponent 实现

### 4.1 内部状态

```cpp
class LidarComponent : public SimulationComponent {
public:
    explicit LidarComponent(LidarInfo info);

    bool init(const SimulationContext& context) override;
    bool reset(const SimulationContext& context) override;
    bool advance(const SimulationContext& context) override;
    bool update(const SimulationContext& context) override;

    bool read_laser_scan(
        std::shared_ptr<const LaserScanState>& state) const;

    bool read_point_cloud(
        std::shared_ptr<const PointCloudState>& state) const;

    const LidarInfo& info() const noexcept;

private:
    LidarInfo info_;

    int site_id_{-1};
    int body_id_{-1};

    std::vector<mjtNum> local_directions_;
    std::vector<mjtNum> world_directions_;
    std::vector<mjtNum> distances_;

    std::shared_ptr<const LaserScanState> laser_scan_state_;
    std::shared_ptr<const PointCloudState> point_cloud_state_;

    std::uint64_t sequence_{0};
    bool initialized_{false};
};
```

不再存在：

```cpp
std::vector<int> beam_addresses_;
```

也不再访问：

```cpp
mjData::sensordata
```

获取 LiDAR beam。

---

### 4.2 Ray Pattern 编译

所有 sin/cos 在 `init()` 完成。

定义：

```text
α = azimuth
β = elevation
```

对于：

```text
channel = r
column  = c
```

有：

```cpp
const double azimuth =
    info_.azimuth_start +
    static_cast<double>(column) *
        info_.azimuth_increment +
    channel.azimuth_offset;

const double elevation =
    channel.elevation;
```

局部方向：

```cpp
x = cos(elevation) * cos(azimuth);
y = cos(elevation) * sin(azimuth);
z = sin(elevation);
```

并按照：

```cpp
index =
    channel_index * azimuth_samples +
    azimuth_index;
```

存入：

```cpp
local_directions_[index * 3 + 0];
local_directions_[index * 3 + 1];
local_directions_[index * 3 + 2];
```

这样运行阶段不再执行：

```text
sin()
cos()
```

---

### 4.3 Runtime Ray Cast

每次 LiDAR 到达自己的 update period：

获取：

```cpp
origin =
    data->site_xpos + 3 * site_id_;

rotation =
    data->site_xmat + 9 * site_id_;
```

把：

```text
local_directions_
```

转换成：

```text
world_directions_
```

即：

```text
direction_world =
    R_world_site * direction_local
```

然后调用一次：

```text
mj_multiRay()
```

而不是每根 beam 分别调用 `mj_ray()`。

MuJoCo 官方明确提供 `mj_multiRay()` 用于从同一个 origin 批量发射多根 ray，并支持 geom group filtering、body exclusion 和 cutoff；没有交点时 distance 为负值。

调用语义：

```text
origin         = site_xpos
directions     = world_directions_
geomgroup      = configured groups / nullptr
flg_static     = true
bodyexclude    = configured parent body / -1
geomid         = nullptr
dist           = distances_
normal         = nullptr
nray           = ray_count
cutoff         = range_max
```

当前不需要：

```text
geomid
surface normal
```

因此不额外分配这些结果。

项目固定 MuJoCo 3.12.0；ray-cast API 在较新 MuJoCo 已包含可选 surface-normal 参数，因此实现时以项目 vendored 3.12.0 header 的实际签名为唯一编译基准。MuJoCo 的 changelog 记录了这项 ray API 变化。

---

### 4.4 Kinematics 时序

`LidarComponent` 内部禁止：

```cpp
mj_forward(...)
mj_kinematics(...)
```

MuJoCo ray-cast 依赖已经计算好的 kinematics，因此调用 ray cast 前必须保证 `mjData` 中 site / geom pose 有效。

这个保证应该由：

```text
SimulationRuntime
+
Simulation scheduler
```

统一提供。

不能让：

```text
Lidar
Camera
IMU
```

各自改变 MuJoCo pipeline。

如果后续确认当前 `mj_step()` 后的 sampling boundary 需要重新计算 post-integration kinematics，应在 Runtime 层统一解决，而不是在 LiDAR 中特殊调用 `mj_forward()`。

---

### 4.5 LaserScan formatter

对于唯一 channel：

```cpp
angle_min =
    azimuth_start +
    channel.azimuth_offset;

angle_increment =
    azimuth_increment;

angle_max =
    angle_min +
    (azimuth_samples - 1) *
        azimuth_increment;
```

Range：

```text
有效 hit 且 range_min <= distance <= range_max
    → distance

没有命中
    → +Inf

命中过近
    → 无效距离表示
```

至少保证所有超出：

```text
[range_min, range_max]
```

的数据不会被当成正常有效测量；这与 ROS `LaserScan` 对有效 range 的定义保持一致。

输出：

```cpp
scan.ranges.size() == azimuth_samples;
scan.intensities.empty();
scan.time_increment = 0.0F;
scan.scan_time = period;
```

---

### 4.6 PointCloud2 formatter

Ray cast 使用的是：

```text
world_directions_
```

但是 PointCloud2 中的 xyz 必须表达在：

```text
frame_id
```

对应的 LiDAR site frame 中。

因此不能：

```cpp
point = distance * world_direction;
```

而应使用：

```cpp
point =
    distance * local_direction;
```

即：

```cpp
x = range * local_direction.x;
y = range * local_direction.y;
z = range * local_direction.z;
```

这样输出点自然位于 LiDAR frame。

数据布局固定：

```text
offset 0 : float32 x
offset 4 : float32 y
offset 8 : float32 z

point_step = 12
row_step   = width * 12
```

PointCloud2：

```cpp
height = channels.size();
width = azimuth_samples;
```

无效 ray：

```text
x = NaN
y = NaN
z = NaN
```

这样不会伪造：

```text
range_max 处存在一个障碍物
```

只要存在无效点：

```cpp
is_dense = false;
```

所有 ray 均有效时：

```cpp
is_dense = true;
```

`PointCloud2` 本身就用 `is_dense` 表示点云中是否存在 invalid points。

二进制数据固定使用 little-endian 编码：

```cpp
is_bigendian = false;
```

不能仅仅假设 host endian；写入 helper 应确保生成的数据确实是 little-endian。

---

### 4.7 内存与性能

初始化时预分配：

```text
local_directions_ : ray_count × 3
world_directions_: ray_count × 3
distances_       : ray_count
```

运行阶段不重新计算扫描几何。

例如：

```text
128 × 2048
= 262144 rays
```

PointCloud XYZ32：

```text
262144 × 12
≈ 3 MiB / frame
```

10 Hz 大约产生：

```text
≈ 30 MiB/s
```

的 point-cloud payload 写入量，因此不应该额外保存：

```text
range + xyz
```

两套重复公开数据。

内部只保存 ray distance；PointCloud formatter 直接生成最终 xyz。

LiDAR 更新周期由：

```cpp
LidarInfo::period
```

控制，不需要每个 physics step 都进行 ray cast。

---

## 5. ComponentManager、测试与实施计划

### 5.1 ComponentManager

仍然只有：

```text
LidarComponent
```

一种组件。

初始化：

```text
LidarInfo
   ↓
LidarComponent
```

根据：

```cpp
info.output
```

产生：

```text
LaserScanState
```

或者：

```text
PointCloudState
```

`ComponentManager` 最终写入：

```cpp
RobotState::laser_scans
RobotState::point_clouds
```

不引入：

```cpp
std::variant
std::optional
```

也不建立：

```text
Lidar2DComponent
Lidar3DComponent
```

---

### 5.2 测试要求

配置测试必须覆盖：

```text
空 site
非法 period
非法 range
samples == 0
increment == 0
空 channels
非法 elevation
ray_count overflow
LaserScan 多 channel
LaserScan 非零 elevation
非法 geom group
```

Ray Pattern 测试覆盖：

```text
azimuth = 0
azimuth = ±π/2
azimuth = π
positive increment
negative increment
channel elevation
channel azimuth_offset
360° 首尾不重复
```

2D synthetic MJCF：

```text
scan site
    |
    +-------- box
```

验证：

```text
正确距离
无 hit
过近目标
site translation
site rotation
parent body exclusion
geom group filtering
```

3D synthetic MJCF：

```text
多个 elevation channel
+
多个已知高度/方向目标
```

验证：

```text
height == channel count
width == azimuth samples
PointField x/y/z
point_step == 12
row_step == width * 12
data.size() == row_step * height
XYZ 正确
invalid point == NaN
is_dense 正确
```

ROS 兼容性测试不需要链接 ROS。

只验证生成结构满足：

```text
LaserScan field semantics

PointCloud2:
data.size() == row_step * height
field offsets/type/count 正确
```

后续在 `ros2_mujoco` 单独测试：

```text
romujoco::LaserScan
    →
sensor_msgs::msg::LaserScan

romujoco::PointCloud2
    →
sensor_msgs::msg::PointCloud2
```

转换层应基本只做 Header/time 与字段复制，不重新解释扫描几何。

MFR3Duo 集成测试至少覆盖：

```text
lidar_front_scan_frame
lidar_rear_scan_frame

初始化成功
两路 LiDAR 独立
beam 数量正确
frame_id 正确
reset 正确
移动底盘后扫描坐标正确
```

---

### 5.3 文件结构

保持现有结构，不创建新的通用 Sensor Framework：

```text
romujoco/
├── include/romujoco/component/
│   └── lidar.hpp
│
├── src/component/lidar/
│   ├── lidar_component.hpp
│   └── lidar_component.cpp
│
├── src/config/
│   ├── simulation_config_data.hpp
│   ├── simulation_config_parser.cpp
│   └── simulation_config_validator.cpp
│
└── tests/unit/
    ├── lidar_config_test.cpp
    ├── lidar_component_test.cpp
    └── lidar_mfr3duo_integration_test.cpp
```

暂时不增加：

```text
RaycastBackend
LidarBackend
SensorBackend
RaycastService
ScanPattern interface
```

内部只需要把配置编译成：

```cpp
local_directions_
```

未来如果加入非规则 solid-state / Livox pattern，只扩展：

```text
配置 → local ray pattern
```

这一阶段。

后面的：

```text
site transform
→
mj_multiRay
→
distance
→
output formatter
```

全部保持不变。

---

### 5.4 实施阶段

整个重构压缩为四个阶段。

**阶段 1：公共模型和配置重构**

完成：

```text
sensor_prefix 删除
LaserScan / PointCloud2 / PointField 定义
LidarInfo 通用化
XML scan/channel/output
Parser
Validator
RobotState typed outputs
```

**阶段 2：统一 Ray Cast 实现**

完成：

```text
site binding
ray pattern compile
site transform
mj_multiRay
range filtering
LaserScan formatter
PointCloud2 formatter
reset/read state
```

**阶段 3：测试闭环**

完成：

```text
config validation
ray mathematics
2D synthetic model
3D synthetic model
filter/self exclusion
message layout
reset/lifecycle
```

**阶段 4：MFR3Duo 与 ROS2 集成**

完成：

```text
front NanoScan3
rear NanoScan3
完整 simulation config

romujoco LaserScan
    → ROS LaserScan

romujoco PointCloud2
    → ROS PointCloud2
```

---

## 最终设计结论

LiDAR 的长期稳定边界应该是：

```text
                    LidarInfo
                        │
                        ▼
              Generic Ray Pattern
                        │
                        ▼
                   Site Pose
                        │
                        ▼
                  mj_multiRay
                        │
                        ▼
                   Distances
                    /       \
                   /         \
          LaserScan         PointCloud2
             2D                 3D
```

其中：

* **MJCF 描述“雷达安装在哪里”**；
* **LidarInfo 描述“雷达如何扫描”**；
* **LidarComponent 描述“如何与 MuJoCo 世界求交”**；
* **LaserScan / PointCloud2 描述“扫描结果如何对外表达”**；
* **ros2_mujoco 只做 ROS message 适配，不重新实现雷达语义**。

这使当前 MFR3Duo 的 NanoScan3 不再需要展开几百或几千个 `<rangefinder>`，同时以后增加 Velodyne、Ouster、Hesai 等常见多线 3D LiDAR 时，也不需要再次修改 LiDAR 核心架构。
