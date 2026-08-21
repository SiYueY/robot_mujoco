# ros2_mujoco 设计方案

## 1. 概述

`ros2_mujoco` 是 ROS2 与 `mujoco_simulation` 之间的适配层，用于将 MuJoCo 仿真接入 ROS2 生态。

设计目标：

* 支持 ROS2 Control；
* 支持 MoveIt2、Nav2 等标准 ROS2 应用；
* 支持 MuJoCo 中已有硬件抽象：

  * Joint
  * MobileBase
  * Imu
  * Camera
  * Lidar
* 保持 `mujoco_simulation` 为纯 C++ 仿真核心，不引入 ROS2 依赖。

整体关系：

```
robot_mujoco

├── mujoco_simulation
│
│   MuJoCo Runtime
│
│   ├── Joint
│   ├── MobileBase
│   ├── Imu
│   ├── Camera
│   └── Lidar
│
└── ros2_mujoco
    |
    ROS2 Adapter
```

依赖方向：

```
ros2_mujoco
      |
      ↓
mujoco_simulation
      |
      ↓
    MuJoCo
```

禁止：

```
mujoco_simulation
      |
      ↓
     ROS2
```

---

# 2. 设计原则

## 2.1 仿真核心与 ROS2 解耦

`mujoco_simulation` 只负责：

* MuJoCo 生命周期；
* Physics Step；
* 模型管理；
* 设备状态；
* 设备控制。

`ros2_mujoco` 负责：

* ROS2 Control 接入；
* ROS2 Topic 发布；
* ROS2 消息转换。

---

## 2.2 不统一所有硬件接口

虽然 Joint、MobileBase、Imu、Camera、Lidar 都属于硬件抽象，但它们生命周期不同：

| 硬件         | 频率         | 接口特点    |
| ---------- | ---------- | ------- |
| Joint      | 500Hz~1kHz | 控制接口    |
| MobileBase | 50~100Hz   | 运动控制    |
| Imu        | 100~500Hz  | 状态发布    |
| Camera     | 10~60Hz    | 图像发布    |
| Lidar      | 5~20Hz     | 点云/扫描发布 |

因此不设计：

```cpp
class Hardware
{
    virtual update();
};
```

避免产生过度抽象。

---

# 3. 软件结构

最终目录：

```
ros2_mujoco

├── include/

│   └── ros2_mujoco/

│       ├── mujoco_hardware.hpp
│       │
│       └── hardware/
│
│           ├── joint.hpp
│           ├── mobile_base.hpp
│           ├── imu.hpp
│           ├── camera.hpp
│           └── lidar.hpp
│


├── src/

│   ├── mujoco_hardware.cpp
│   │
│   └── hardware/
│
│       ├── joint.cpp
│       ├── mobile_base.cpp
│       ├── imu.cpp
│       ├── camera.cpp
│       └── lidar.cpp


├── config/

│   └── hardware.yaml


├── launch/

│   └── simulation.launch.py


├── plugin_description.xml

├── package.xml

└── CMakeLists.txt
```

---

# 4. MujocoHardware

文件：

```
include/ros2_mujoco/mujoco_hardware.hpp
```

## 4.1 职责

`MujocoHardware` 是 ROS2 Control Hardware Plugin。

负责：

* 加载 MuJoCo Simulation；
* 管理 Joint；
* 管理 MobileBase；
* 提供 command/state interface；
* 与 controller_manager 通信。

继承：

```cpp
hardware_interface::SystemInterface
```

---

## 4.2 类结构

```cpp
namespace ros2_mujoco
{

class MujocoHardware
    : public hardware_interface::SystemInterface
{

public:

    CallbackReturn on_init(
        const hardware_interface::HardwareInfo& info) override;


    CallbackReturn on_configure(
        const rclcpp_lifecycle::State&) override;


    CallbackReturn on_activate(
        const rclcpp_lifecycle::State&) override;


    CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State&) override;


    std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;


    std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;


    hardware_interface::return_type read(
        const rclcpp::Time&,
        const rclcpp::Duration&) override;


    hardware_interface::return_type write(
        const rclcpp::Time&,
        const rclcpp::Duration&) override;


private:

    std::shared_ptr<
        mujoco_simulation::Simulation
    > simulation_;


    std::vector<hardware::Joint>
    joints_;


    std::vector<hardware::MobileBase>
    mobile_bases_;

};

}
```

---

# 5. Joint

文件：

```
hardware/joint.hpp
```

## 5.1 职责

表示 ROS2 Control 中的机器人关节。

支持：

* position；
* velocity；
* effort。

---

## 5.2 接口

```cpp
class Joint
{

public:

    bool initialize(
        mujoco_simulation::Joint* joint);


    double position() const;


    double velocity() const;


    double effort() const;


    void set_position_command(
        double value);


    void set_velocity_command(
        double value);


    void set_effort_command(
        double value);

};
```

---

## 5.3 数据流

读取：

```
MuJoCo

 ↓

mujoco_simulation::Joint

 ↓

ros2_mujoco::hardware::Joint

 ↓

ROS2 state_interface
```

写入：

```
controller

 ↓

command_interface

 ↓

Joint

 ↓

MuJoCo actuator
```

---

# 6. MobileBase

文件：

```
hardware/mobile_base.hpp
```

## 6.1 职责

负责移动底盘适配。

支持：

* differential drive；
* mecanum；
* omnidirectional。

---

## 6.2 接口

```cpp
class MobileBase
{

public:

    bool initialize(
        mujoco_simulation::MobileBase* base);


    void set_velocity(
        double vx,
        double vy,
        double wz);


    MobileBaseState state();

};
```

---

## 6.3 数据流

```
cmd_vel

 ↓

MobileBase

 ↓

运动学模型

 ↓

MuJoCo
```

---

# 7. Imu

文件：

```
hardware/imu.hpp
```

## 7.1 职责

负责 MuJoCo IMU 数据转换。

输出：

```
sensor_msgs/msg/Imu
```

---

## 7.2 接口

```cpp
class Imu
{

public:

    bool initialize(
        mujoco_simulation::Imu* imu);


    ImuState data();

};
```

---

ROS Topic：

```
/imu/data
```

---

# 8. Camera

文件：

```
hardware/camera.hpp
```

## 8.1 职责

负责相机数据转换。

支持：

* RGB；
* Depth；
* CameraInfo。

---

## 8.2 接口

```cpp
class Camera
{

public:

    bool initialize(
        mujoco_simulation::Camera* camera);


    ImageFrame image();


    CameraInfo info();

};
```

---

ROS Topic：

```
/camera/image_raw

/camera/camera_info
```

---

# 9. Lidar

文件：

```
hardware/lidar.hpp
```

## 9.1 职责

负责激光雷达数据转换。

输出：

```
sensor_msgs/msg/LaserScan
```

---

## 9.2 接口

```cpp
class Lidar
{

public:

    bool initialize(
        mujoco_simulation::Lidar* lidar);


    LaserScan scan();

};
```

---

ROS Topic：

```
/scan
```

---

# 10. ROS2 Control 接入

URDF/Xacro：

```xml
<ros2_control
    name="mujoco"
    type="system">


    <hardware>

        <plugin>
            ros2_mujoco/MujocoHardware
        </plugin>

    </hardware>


</ros2_control>
```

---

控制链：

```
MoveIt2

 ↓

JointTrajectoryController

 ↓

controller_manager

 ↓

MujocoHardware

 ↓

mujoco_simulation

 ↓

MuJoCo
```

---

# 11. 配置文件

统一配置：

```
config/hardware.yaml
```

不拆：

```
ros2_control.yaml
sensors.yaml
```

原因：

机器人硬件描述应该作为一个整体管理。

---

示例：

```yaml
hardware:

  joints:

    - name: fr3_joint1

      command_interfaces:
        - position

      state_interfaces:
        - position
        - velocity
        - effort



  mobile_base:

    name: base

    type: mecanum



  sensors:


    imu:

      - name: imu_link

        topic: /imu/data



    camera:

      - name: head_camera

        topic: /camera/image_raw

        width: 1920

        height: 1080



    lidar:

      - name: front_lidar

        topic: /scan
```

---

# 12. 生命周期

启动流程：

```
simulation.launch.py

        |

        |

ros2_mujoco

        |

        |

MujocoHardware

        |

        |

controller_manager

        |

        |

Controllers
```

---

生命周期：

```
unconfigured

      ↓

configured

      ↓

activated

      ↓

running
```

---

# 13. 与真实机器人统一

仿真：

```
MoveIt2

 ↓

ros2_control

 ↓

MujocoHardware

 ↓

mujoco_simulation

 ↓

MuJoCo
```

---

真实机器人：

```
MoveIt2

 ↓

ros2_control

 ↓

H10WHardware

 ↓

LowCmd / LowState

 ↓

Robot
```

二者共享：

* ROS2 Control；
* Controller；
* MoveIt2；
* Nav2。

---

# 14. 最终设计总结

`ros2_mujoco` 采用：

* 一个 ROS2 Control 入口：`MujocoHardware`；
* 一个 hardware 目录管理设备适配；
* 不引入统一 Hardware 基类；
* 不引入 HardwareManager；
* Joint / MobileBase 负责控制；
* Imu / Camera / Lidar 负责传感器数据转换；
* 配置统一为 `hardware.yaml`。

最终架构：

```
ros2_mujoco

├── MujocoHardware

└── hardware

    ├── Joint

    ├── MobileBase

    ├── Imu

    ├── Camera

    └── Lidar
```

该设计能够同时支持：

* FR3 / Mobile FR3 Duo；
* H10W 双臂机器人；
* MoveIt2；
* Nav2；
* ROS2 Control；
* MuJoCo 仿真。
