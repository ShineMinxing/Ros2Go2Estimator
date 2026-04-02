# CAPO-LeggedRobotOdometry 🦾  [![License](https://img.shields.io/badge/License-See%20LICENSE-blue.svg)](LICENSE)

**Language / 语言：** **English** | [中文](#capo-leggedrobotodometry--中文)

**CAPO-LeggedRobotOdometry** is a ROS 2 (**Humble**) proprioceptive odometry / state-estimation repository for **biped / quadruped / wheel-legged** robots on **Ubuntu 22.04**.

It provides high-accuracy odometry using only **IMU + joint encoders + foot contact/force signals**, without requiring cameras or LiDAR.

In 3D closed-loop trials (a **200 m** horizontal and **15 m** vertical loop), Astrall point-foot robot A achieves **0.1638 m** horizontal error and **0.219 m** vertical error; for wheel-legged robot B, the corresponding errors are **0.2264 m** and **0.199 m**.

At the repository level, **`fusion_estimator_node.cpp`** is mainly the **ROS2 wrapper** (topics, parameters, message conversion, publishing), while the actual fusion-estimation algorithm lives in **`FusionEstimator/`** as a **portable, pure C++ implementation**. This makes it straightforward to port the estimator to **ROS1**, **non-ROS2 applications**, or **embedded platforms**.

In addition, the **`Matlab/`** folder provides a **MATLAB + MEX** example that compiles and calls the same C++ estimator core for **offline validation**, **algorithm analysis**, and **cross-platform reuse**.

---

## 📄 Paper

**Contact-Anchored Proprioceptive Odometry for Quadruped Robots** (arXiv:2602.17393)

* Paper: https://arxiv.org/abs/2602.17393

If you use this repository in research, please consider citing the paper.

---

## 📦 Data Sharing (Go2-EDU ROS bags)

To help readers quickly validate the pipeline, we provide **two Go2-EDU trial datasets**, including **real-world videos** and the corresponding **ROS bag topics/messages** required by this node, enabling fast reproduction and sanity checks.

* Download (Google Drive): https://drive.google.com/drive/folders/1FfVO69rfmUu6B9crPhZCfKf9wFnV4L7n?usp=sharing

> Note: the IMU on this Go2-EDU platform exhibits **noticeable yaw drift**, so the odometry accuracy is generally **worse than** the results reported for Astrall robots A and B in the paper.

---

## ✨ Key Features

| Category | Description |
| --- | --- |
| **Biped / Quadruped / Wheel-Legged Unified** | Online contact-set switching; stance legs are detected automatically, supporting fast transitions between standing and walking. |
| **Full 3D & Planar 2D** | Publishes both 6DoF odometry (`SMX/Odom`) and a gravity-flattened 2D odometry (`SMX/Odom_2D`). |
| **No Exteroception Required** | Works without cameras or LiDAR; only IMU, joint encoders, and foot contact/force signals are required. |
| **Portable Pure C++ Core** | The estimator core is isolated in `FusionEstimator/`, making it easier to reuse outside ROS2. |
| **MATLAB / MEX Validation** | The `Matlab/` folder demonstrates how to compile and invoke the same C++ core from MATLAB. |
| **Runtime Tuning** | Key parameters can be adjusted through `config.yaml`, and platform-dependent thresholds can be tuned for different robots. |

---

## 🏗️ Related Repositories

| Scope | Repository | Summary |
| --- | --- | --- |
| **Low-level / Drivers** | https://github.com/ShineMinxing/Ros2Go2Base | DDS bridge, Unitree SDK2 control, pointcloud→LaserScan, TF utilities |
| **Odometry** | **CAPO-LeggedRobotOdometry (this repo)** | Pure proprioceptive fusion, publishes `SMX/Odom` / `SMX/Odom_2D` |
| **SLAM / Mapping** | https://github.com/ShineMinxing/Ros2SLAM | Integrations for Cartographer 3D, KISS-ICP, FAST-LIO2, Point-LIO |
| **Voice / LLM** | https://github.com/ShineMinxing/Ros2Chat | Offline ASR + OpenAI Chat + TTS |
| **Vision** | https://github.com/ShineMinxing/Ros2ImageProcess | Camera pipelines, spot / face / drone detection |
| **Gimbal Tracking** | https://github.com/ShineMinxing/Ros2AmovG1 | Amov G1 gimbal control and tracking |
| **Tools** | https://github.com/ShineMinxing/Ros2Tools | Bluetooth IMU, joystick mapping, gimbal loop, data logging |

> ⚠️ **Clone as needed.** If you only need state estimation, this repository is sufficient. For mapping, it is natural to pair it with `Ros2SLAM` and `Ros2Go2Base`.

---

## 📂 Repository Layout

```text
CAPO-LeggedRobotOdometry/
├── CMakeLists.txt
├── package.xml
├── config.yaml
├── fusion_estimator_node.cpp        # ROS2 node wrapper: params / topics / odom publishing
├── FusionEstimator/                 # pure C++ fusion-estimation core (portable)
│   ├── Estimators/
│   ├── fusion_estimator.h
│   ├── LowlevelState.h
│   ├── SensorBase.cpp
│   ├── SensorBase.h
│   ├── Sensor_IMU.cpp
│   ├── Sensor_IMU.h
│   ├── Sensor_Legs.cpp
│   ├── Sensor_Legs.h
│   └── Readme.md
├── Matlab/                          # MATLAB + MEX example for calling the C++ core
│   ├── build_mex.m
│   ├── fusion_estimator.m
│   ├── fusion_estimator_mex.cpp
│   ├── MPXY150Z10.csv
│   └── MWXY150Z10.csv
├── Plotjuggler.xml                  # PlotJuggler layout / visualization helper
└── Readme.md
```

---

## 🧩 Architecture Notes

This repository is intentionally split into two layers:

### 1. ROS2 wrapper layer
`fusion_estimator_node.cpp` is responsible for:

* reading parameters from `config.yaml`
* subscribing to ROS2 topics
* converting ROS messages into estimator inputs
* invoking the fusion-estimation core
* publishing `nav_msgs/Odometry`

### 2. Portable estimator core
`FusionEstimator/` contains the actual estimation logic:

* IMU handling
* leg kinematics / leg sensors
* contact-anchored fusion logic
* estimator state update
* reusable C++ interfaces

Because this part is **pure C++**, it can be reused in:

* **ROS1**
* **non-ROS Linux applications**
* **embedded / custom runtime systems**
* **MATLAB via MEX**

### 3. MATLAB / MEX bridge
`Matlab/` is not required for ROS2 runtime, but it is useful when you want to:

* validate the estimator offline
* replay data in MATLAB
* compare C++ results with MATLAB scripts
* reuse the same `FusionEstimator/` core without ROS2

---

## ⚙️ Configuration (`config.yaml`)

The table below lists the main parameters used by the ROS2 node. See `config.yaml` for the full file and comments.

| Parameter | Typical Value | Meaning |
| --- | ---: | --- |
| `sub_imu_topic` | `SMX/Go2IMU` | IMU topic |
| `sub_joint_topic` | `SMX/Go2Joint` | joint state topic |
| `sub_mode_topic` | `SMX/SportCmd` | reset / mode topic |
| `pub_odom_topic` | `SMX/Odom` | 6DoF odometry output |
| `pub_odom2d_topic` | `SMX/Odom_2D` | planar odometry output |
| `odom_frame` | `odom` | odometry frame |
| `base_frame` | `base_link` | body frame |
| `base_frame_2d` | `base_link_2D` | planar body frame |
| `imu_data_enable` | `true` | enable IMU input |
| `leg_pos_enable` | `true` | enable leg-position kinematics |
| `leg_vel_enable` | `true` | enable leg-velocity kinematics |
| `leg_ori_enable` | `false` | enable kinematics-based yaw correction |
| `leg_ori_init_weight` | `0.001` | initial weight of orientation correction |
| `leg_ori_time_wight` | `1000.0` | time-related growth weight for orientation correction |
| `leg_velcke_enable` | `false` | enable IKVel-CKF / CAPO-CKE |
| `foot_force_threshold` | `20.0` | contact threshold |
| `min_stair_height` | `0.08` | minimum stair-height hypothesis used by the estimator |

> Notes:
> * The exact values are platform-dependent.
> * Some fields in `config.yaml` may be experimental, legacy, or intended for specific branches / workflows.
> * If you port only the pure C++ core, you usually need to keep only the estimator-relevant fields and can drop ROS2-specific naming.

---

## ⚙️ Build & Run

### 1. Clone the repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ShineMinxing/CAPO-LeggedRobotOdometry.git
```

### 2. Install ROS2 dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build

```bash
cd ~/ros2_ws
colcon build --packages-select fusion_estimator
```

### 4. Source the workspace

```bash
cd ~/ros2_ws
source install/setup.bash
```

### 5. Run the node

```bash
ros2 run fusion_estimator fusion_estimator_node
```

### 6. Optional: pull large files / sample data

If you want to use optional sample CSV files or large assets, you may need Git LFS:

```bash
sudo apt update
sudo apt install git-lfs
git lfs install
cd ~/ros2_ws/src/CAPO-LeggedRobotOdometry
git lfs pull
```

> Note: in some versions of this repository, the node loads `config.yaml` through a hard-coded `--params-file` path inside `fusion_estimator_node.cpp`. If your local workspace path differs, update that path and rebuild.

---

## 📑 ROS 2 Interfaces

```text
/fusion_estimator_node (rclcpp)
├─ Publishes
│   • SMX/Odom         nav_msgs/Odometry (odom → base_link)
│   • SMX/Odom_2D      nav_msgs/Odometry (odom → base_link_2D)
├─ Subscribes
│   • SMX/Go2IMU       sensor_msgs/Imu
│   • SMX/Go2Joint     std_msgs/Float64MultiArray
│       layout:
│         data[0..11]   = 12 joint positions q
│         data[12..23]  = 12 joint velocities dq
│         data[24..27]  = 4 foot-force / contact-related values
│   • SMX/SportCmd     std_msgs/Float64MultiArray
│       reset example:
│         data[0] == 25140000  → estimator reset
└─ TF is typically published by another node / utility if needed
```

---

## 🧠 Algorithm Overview (High Level)

1. **Contact detection**  
   Detect stance legs from force / threshold logic.

2. **Forward kinematics**  
   Compute foot-end position / velocity in the body frame from joint measurements.

3. **Contact anchoring**  
   Record touchdown footfall points and use them as intermittent world-frame constraints during stance.

4. **Height stabilization**  
   Use support-plane height logic and confidence decay to reduce long-horizon elevation drift.

5. **Optional IKVel-CKF (CAPO-CKE)**  
   Suppress encoder-induced velocity spikes and obtain smoother leg-end velocity estimates.

6. **Optional yaw stabilization**  
   Use multi-contact geometric consistency to reduce IMU yaw drift during prolonged standing.

7. **2D projection**  
   Publish a planar odometry (`SMX/Odom_2D`) by flattening roll / pitch while preserving yaw and planar motion.

For full derivations, please refer to the paper.

---

## 🔧 Reusing `FusionEstimator/` Outside ROS2

If you do **not** need ROS2, the reusable part is mainly under `FusionEstimator/`.

A typical migration path is:

1. keep the estimator core in `FusionEstimator/`
2. replace ROS2 message subscriptions with your own sensor interface
3. prepare IMU, joint position / velocity, and contact / force inputs
4. call the estimator core from your own loop
5. export odometry to your target middleware / runtime

This design is useful for:

* ROS1 migration
* embedded deployment
* offline replay tools
* MATLAB / simulation analysis
* custom robotics middleware

---

## 🧪 MATLAB / MEX (Optional)

The `Matlab/` folder provides an additional example for compiling the C++ estimator core into a MATLAB MEX module.

Typical workflow:

```matlab
cd Matlab
build_mex
fusion_estimator
```

Files in this folder:

* `build_mex.m` — build script for MEX compilation
* `fusion_estimator_mex.cpp` — MEX bridge wrapping the C++ estimator core
* `fusion_estimator.m` — MATLAB-side usage / validation example
* `MPXY150Z10.csv`, `MWXY150Z10.csv` — sample data for offline tests

This is especially useful when you want to:

* verify C++ estimator behavior in MATLAB
* compare offline results quickly
* reuse the same `FusionEstimator/` implementation without ROS2
* debug algorithm changes before reintegrating them into the ROS2 node

---

## 📈 Visualization

A `Plotjuggler.xml` file is included in the repository for convenient visualization and debugging with PlotJuggler.

You can use it to inspect:

* odometry outputs
* IMU-related signals
* joint / force-related channels
* estimator behavior during walking, standing, or reset events

---

## 🎥 Videos

| Topic | Link |
| --- | --- |
| Odometry-only mapping (morphology switching) | [![img](https://i1.hdslb.com/bfs/archive/4f60453cb37ce5e4f593f03084dbecd0fdddc27e.jpg)](https://www.bilibili.com/video/BV1UtQfYJExu) |
| Indoor walking (0.5%–1% error) | [![img](https://i1.hdslb.com/bfs/archive/10e501bc7a93c77c1c3f41f163526b630b0afa3f.jpg)](https://www.bilibili.com/video/BV18Q9JYEEdn/) |
| Stair climbing (height error < 5 cm) | [![img](https://i0.hdslb.com/bfs/archive/c469a3dd37522f6b7dcdbdbb2c135be599eefa7b.jpg)](https://www.bilibili.com/video/BV1VV9ZYZEcH/) |
| Outdoor walking (380 m, 3.3% error) | [![img](https://i0.hdslb.com/bfs/archive/481731d2db755bbe087f44aeb3f48db29c159ada.jpg)](https://www.bilibili.com/video/BV1BhRAYDEsV/) |
| Voice interaction + navigation | [![img](https://i2.hdslb.com/bfs/archive/5b95c6eda3b6c9c8e0ba4124c1af9f3da10f39d2.jpg)](https://www.bilibili.com/video/BV1HCQBYUEvk/) |
| Face tracking + laser spot tracking | [![img](https://i0.hdslb.com/bfs/archive/5496e9d0b40915c62b69701fd1e23af7d6ffe7de.jpg)](https://www.bilibili.com/video/BV1faG1z3EFF/) |
| AR glasses head-following | [![img](https://i1.hdslb.com/bfs/archive/9e0462e12bf77dd9bbe8085d0d809f233256fdbd.jpg)](https://www.bilibili.com/video/BV1pXEdzFECW) |
| YOLO drone tracking | [![img](https://i1.hdslb.com/bfs/archive/a5ac45ec76ccb7c3fb18de9c6b8df48e8abe2b54.jpg)](https://www.bilibili.com/video/BV18v8xzJE4G) |
| Gimbal + fixed camera collaboration | [![img](https://i2.hdslb.com/bfs/archive/07ac6082b7efdc2e2d200e18fc8074eec1d9cfba.jpg)](https://www.bilibili.com/video/BV1fTY7z7E5T) |
| Multiple SLAM integrations | [![img](https://i1.hdslb.com/bfs/archive/f299bafc7486f71e061eb31f9f00347063e1e621.jpg)](https://www.bilibili.com/video/BV1ytMizEEdG) |

---

## 📄 Further Reading

* Paper (arXiv): https://arxiv.org/abs/2602.17393
* Technical notes (Notion): https://www.notion.so/Ros2Go2-1e3a3ea29e778044a4c9c35df4c27b22
* ROS1 reference implementation: https://github.com/ShineMinxing/FusionEstimation

---

## 📨 Contact

| Email | Affiliation |
| --- | --- |
| [sunminxing20@mails.ucas.ac.cn](mailto:sunminxing20@mails.ucas.ac.cn) | Institute of Optics and Electronics, CAS |

> This repository is under active development. Issues and PRs are welcome.

---

# CAPO-LeggedRobotOdometry 🦾 (中文)

**语言 / Language：** [English](#capo-leggedrobotodometry-) | **中文**

**CAPO-LeggedRobotOdometry** 是面向 **双足 / 四足 / 轮足** 机器人的 **ROS 2（Humble）** 本体状态估计 / 里程计算法仓库，可在 **Ubuntu 22.04** 上运行。

本仓库仅依赖 **IMU + 关节编码器 + 足端接触/力信号** 即可输出高精度里程计，不依赖相机或激光雷达。

在 3D 闭环运动实验（水平 **200 m**、竖直 **15 m**）中：Astrall 公司的点足机器人 A 的水平与竖直误差分别为 **0.1638 m** 和 **0.219 m**；轮足机器人 B 的对应误差分别为 **0.2264 m** 和 **0.199 m**。

从工程结构上看，**`fusion_estimator_node.cpp`** 主要负责 **ROS2 封装层**（参数、Topic、消息转换、发布），而真正的融合估计算法位于 **`FusionEstimator/`** 中，并且实现为 **纯 C++、可移植** 的核心模块。因此它不仅能用于 ROS2，也便于迁移到：

* **ROS1**
* **非 ROS2 平台**
* **嵌入式 / 自定义运行环境**

另外，仓库中的 **`Matlab/`** 文件夹额外提供了 **MATLAB + MEX 混合编译并调用 `FusionEstimator` 中 C++ 核心代码** 的示例，方便离线验证、算法分析和跨平台复用。

---

## 📄 论文

**Contact-Anchored Proprioceptive Odometry for Quadruped Robots**（arXiv:2602.17393）

* 论文链接：https://arxiv.org/abs/2602.17393

如果本仓库对你的研究有帮助，欢迎引用论文。

---

## 📦 数据共享

为便于大家快速验证本仓库的有效性，我们提供了 **Go2-EDU** 的两段示例数据，包含**实拍录像**以及可直接用于本节点的 **ROS bag topic / message**，用于快速复现与 sanity check。

* 数据下载（Google Drive）：https://drive.google.com/drive/folders/1FfVO69rfmUu6B9crPhZCfKf9wFnV4L7n?usp=sharing

> 说明：该 Go2-EDU 平台的 IMU **yaw 漂移较明显**，因此里程计精度通常会**劣于**论文中 Astrall 机器人 A 与 B 的实验结果。

---

## ✨ 算法特性

| 类别 | 说明 |
| --- | --- |
| **双足 / 四足 / 轮足统一支持** | 运行中自动识别支撑足，支持站立、行走和形态切换。 |
| **全 3D + 平面 2D** | 同时发布 6DoF 里程计 `SMX/Odom` 与压平后的 `SMX/Odom_2D`。 |
| **零依赖外部感知** | 不依赖相机或激光雷达，仅依赖本体传感器。 |
| **纯 C++ 可移植核心** | `FusionEstimator/` 中是独立的纯 C++ 融合估计实现，便于脱离 ROS2 使用。 |
| **MATLAB / MEX 联调** | `Matlab/` 中给出了用 MATLAB 混编调用同一套 C++ 核心的示例。 |
| **参数可调** | `config.yaml` 中的关键参数和阈值可按平台进行调整。 |

---

## 🏗️ 生态仓库一览

| 范畴 | 仓库 | 功能简介 |
| --- | --- | --- |
| **底层驱动** | https://github.com/ShineMinxing/Ros2Go2Base | DDS 桥、Unitree SDK2 控制、点云→Scan、TF 工具 |
| **里程计** | **CAPO-LeggedRobotOdometry（本仓库）** | 纯本体多传感器融合，发布 `SMX/Odom` / `SMX/Odom_2D` |
| **SLAM / 建图** | https://github.com/ShineMinxing/Ros2SLAM | 集成 Cartographer 3D、KISS-ICP、FAST-LIO2、Point-LIO 等 |
| **语音 / LLM** | https://github.com/ShineMinxing/Ros2Chat | 离线 ASR + OpenAI Chat + 语音合成 |
| **图像处理** | https://github.com/ShineMinxing/Ros2ImageProcess | 相机、光点 / 人脸 / 无人机检测 |
| **吊舱跟随** | https://github.com/ShineMinxing/Ros2AmovG1 | Amov G1 吊舱控制与目标跟踪 |
| **工具集** | https://github.com/ShineMinxing/Ros2Tools | 蓝牙 IMU、手柄映射、吊舱闭环、数据记录等 |

> ⚠️ 按需克隆：如果只做状态估计，本仓库即可独立使用；如果要建图，通常可以配合 `Ros2SLAM` 和 `Ros2Go2Base` 一起使用。

---

## 📂 仓库结构

```text
CAPO-LeggedRobotOdometry/
├── CMakeLists.txt
├── package.xml
├── config.yaml
├── fusion_estimator_node.cpp        # ROS2 节点封装：参数 / Topic / 里程计发布
├── FusionEstimator/                 # 纯 C++ 融合估计算法核心（可移植）
│   ├── Estimators/
│   ├── fusion_estimator.h
│   ├── LowlevelState.h
│   ├── SensorBase.cpp
│   ├── SensorBase.h
│   ├── Sensor_IMU.cpp
│   ├── Sensor_IMU.h
│   ├── Sensor_Legs.cpp
│   ├── Sensor_Legs.h
│   └── Readme.md
├── Matlab/                          # MATLAB + MEX 调用 C++ 核心的示例
│   ├── build_mex.m
│   ├── fusion_estimator.m
│   ├── fusion_estimator_mex.cpp
│   ├── MPXY150Z10.csv
│   └── MWXY150Z10.csv
├── Plotjuggler.xml                  # PlotJuggler 可视化配置
└── Readme.md
```

---

## 🧩 架构说明

本仓库有意分成两层：

### 1）ROS2 封装层
`fusion_estimator_node.cpp` 主要负责：

* 从 `config.yaml` 读取参数
* 订阅 ROS2 Topic
* 将 ROS 消息转换为估计器输入
* 调用融合估计算法核心
* 发布 `nav_msgs/Odometry`

### 2）可移植的估计器核心
`FusionEstimator/` 中包含真正的估计逻辑，例如：

* IMU 数据处理
* 腿部运动学 / 腿部传感器建模
* 接触锚定约束
* 融合估计状态更新
* 可复用的 C++ 接口

因为这里是**纯 C++ 实现**，所以它不仅能在 ROS2 中使用，也很适合迁移到：

* **ROS1**
* **非 ROS2 的 Linux 程序**
* **嵌入式 / 自定义系统**
* **MATLAB MEX**
* **离线回放与仿真验证**

### 3）MATLAB / MEX 桥接层
`Matlab/` 文件夹不是 ROS2 运行必须的，但在以下场景里很有用：

* 做离线数据验证
* 在 MATLAB 中回放与分析估计结果
* 快速对比 C++ 结果与 MATLAB 脚本
* 不依赖 ROS2 直接复用同一套 `FusionEstimator/` 核心

---

## ⚙️ 参数配置（`config.yaml`）

下面列出当前 ROS2 节点最常用的参数。完整注释请查看 `config.yaml`。

| 参数名 | 常见取值 | 说明 |
| --- | ---: | --- |
| `sub_imu_topic` | `SMX/Go2IMU` | IMU 订阅 Topic |
| `sub_joint_topic` | `SMX/Go2Joint` | 关节状态订阅 Topic |
| `sub_mode_topic` | `SMX/SportCmd` | 复位 / 模式 Topic |
| `pub_odom_topic` | `SMX/Odom` | 全 3D 里程计输出 |
| `pub_odom2d_topic` | `SMX/Odom_2D` | 平面里程计输出 |
| `odom_frame` | `odom` | 里程计坐标系 |
| `base_frame` | `base_link` | 机体坐标系 |
| `base_frame_2d` | `base_link_2D` | 平面机体坐标系 |
| `imu_data_enable` | `true` | 是否启用 IMU |
| `leg_pos_enable` | `true` | 是否启用腿部位置运动学 |
| `leg_vel_enable` | `true` | 是否启用腿部速度运动学 |
| `leg_ori_enable` | `false` | 是否启用基于运动学的航向修正 |
| `leg_ori_init_weight` | `0.001` | 姿态修正初始权重 |
| `leg_ori_time_wight` | `1000.0` | 姿态修正随时间增长的权重 |
| `leg_velcke_enable` | `false` | 是否启用 IKVel-CKF / CAPO-CKE |
| `foot_force_threshold` | `20.0` | 触地阈值 |
| `min_stair_height` | `0.08` | 最小台阶高度假设 |

> 说明：
> * 具体参数需要根据机器人平台做调整。
> * `config.yaml` 中有些字段可能用于实验、兼容旧版本，或特定分支。
> * 如果你只移植 `FusionEstimator/` 纯 C++ 核心，一般只需要保留估计器真正需要的参数，不必保留 ROS2 命名相关字段。

---

## ⚙️ 编译与运行

### 1）克隆仓库

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ShineMinxing/CAPO-LeggedRobotOdometry.git
```

### 2）安装依赖

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3）编译

```bash
cd ~/ros2_ws
colcon build --packages-select fusion_estimator
```

### 4）加载环境

```bash
cd ~/ros2_ws
source install/setup.bash
```

### 5）运行节点

```bash
ros2 run fusion_estimator fusion_estimator_node
```

### 6）可选：拉取大文件 / 示例数据

如果你需要示例 CSV 或大文件资源，可以安装 Git LFS：

```bash
sudo apt update
sudo apt install git-lfs
git lfs install
cd ~/ros2_ws/src/CAPO-LeggedRobotOdometry
git lfs pull
```

> 说明：某些版本中，`fusion_estimator_node.cpp` 会通过硬编码的 `--params-file` 路径加载 `config.yaml`。如果你的本地工作空间路径不同，需要先修改该路径并重新编译。

---

## 📑 节点接口

```text
/fusion_estimator_node (rclcpp)
├─ 发布
│   • SMX/Odom         nav_msgs/Odometry (odom → base_link)
│   • SMX/Odom_2D      nav_msgs/Odometry (odom → base_link_2D)
├─ 订阅
│   • SMX/Go2IMU       sensor_msgs/Imu
│   • SMX/Go2Joint     std_msgs/Float64MultiArray
│       数据布局：
│         data[0..11]   = 12 个关节位置 q
│         data[12..23]  = 12 个关节速度 dq
│         data[24..27]  = 4 个足端力 / 接触相关数值
│   • SMX/SportCmd     std_msgs/Float64MultiArray
│       复位示例：
│         data[0] == 25140000  → 复位估计器
└─ 若需要 TF，一般由其他节点 / 工具单独发布
```

---

## 🧠 算法概览（简述）

1. **触地检测**  
   基于足端力 / 阈值逻辑判定支撑腿。

2. **前向运动学**  
   根据关节测量计算足端在机身坐标系中的位置与速度。

3. **落足点锚定**  
   在触地时记录落足点，在支撑期将接触点作为世界系间歇约束。

4. **高度稳定**  
   利用支撑平面高度逻辑与置信度衰减来降低长时程高度漂移。

5. **可选 IKVel-CKF（CAPO-CKE）**  
   抑制编码器速度尖峰，使足端速度估计更平滑。

6. **可选航向稳定**  
   利用多接触几何一致性来减小原地站立时的 IMU yaw 漂移。

7. **2D 投影**  
   压平 roll / pitch，保留 yaw 与平面运动，发布 `SMX/Odom_2D`。

完整推导请参考论文。

---

## 🔧 在 ROS2 之外复用 `FusionEstimator/`

如果你**不需要 ROS2**，真正可复用的主体主要就是 `FusionEstimator/`。

一个典型迁移流程是：

1. 保留 `FusionEstimator/` 下的纯 C++ 核心
2. 用你自己的传感器接口替换 ROS2 订阅
3. 准备 IMU、关节位置 / 速度、触地 / 足端力输入
4. 在你的主循环中调用估计器
5. 将输出接到你的中间件 / 控制系统

这对以下场景很有帮助：

* ROS1 迁移
* 嵌入式部署
* 离线数据回放
* MATLAB / 仿真验证
* 自定义机器人软件框架

---

## 🧪 MATLAB / MEX（可选）

`Matlab/` 文件夹提供了一个额外示例，用来把 `FusionEstimator/` 中的纯 C++ 核心编译成 MATLAB 可调用的 MEX 模块。

典型流程如下：

```matlab
cd Matlab
build_mex
fusion_estimator
```

该文件夹中的主要文件：

* `build_mex.m`：MEX 编译脚本
* `fusion_estimator_mex.cpp`：MEX 桥接文件，负责封装 C++ 估计器核心
* `fusion_estimator.m`：MATLAB 侧调用示例
* `MPXY150Z10.csv`、`MWXY150Z10.csv`：离线测试用示例数据

这部分特别适合以下需求：

* 在 MATLAB 中验证 C++ 融合器行为
* 快速做离线结果对比
* 不依赖 ROS2 复用同一套 `FusionEstimator/`
* 在把修改重新合回 ROS2 节点之前，先做算法调试

---

## 📈 可视化

仓库中附带了 `Plotjuggler.xml`，可用于 PlotJuggler 的快速可视化与调试。

你可以用它观察：

* 里程计输出
* IMU 相关信号
* 关节 / 足端力相关量
* 站立、行走、复位过程中的估计器行为

---

## 🎥 视频演示

| 主题 | 链接 |
| --- | --- |
| 纯里程计建图（形态切换） | [![img](https://i1.hdslb.com/bfs/archive/4f60453cb37ce5e4f593f03084dbecd0fdddc27e.jpg)](https://www.bilibili.com/video/BV1UtQfYJExu) |
| 室内行走（误差 0.5%–1%） | [![img](https://i1.hdslb.com/bfs/archive/10e501bc7a93c77c1c3f41f163526b630b0afa3f.jpg)](https://www.bilibili.com/video/BV18Q9JYEEdn/) |
| 爬楼梯（高度误差 < 5 cm） | [![img](https://i0.hdslb.com/bfs/archive/c469a3dd37522f6b7dcdbdbb2c135be599eefa7b.jpg)](https://www.bilibili.com/video/BV1VV9ZYZEcH/) |
| 户外行走（380 m，误差 3.3%） | [![img](https://i0.hdslb.com/bfs/archive/481731d2db755bbe087f44aeb3f48db29c159ada.jpg)](https://www.bilibili.com/video/BV1BhRAYDEsV/) |
| 语音交互 + 地图导航 | [![img](https://i2.hdslb.com/bfs/archive/5b95c6eda3b6c9c8e0ba4124c1af9f3da10f39d2.jpg)](https://www.bilibili.com/video/BV1HCQBYUEvk/) |
| 人脸识别跟踪 + 光点跟踪 | [![img](https://i0.hdslb.com/bfs/archive/5496e9d0b40915c62b69701fd1e23af7d6ffe7de.jpg)](https://www.bilibili.com/video/BV1faG1z3EFF/) |
| AR 眼镜头部运动跟随 | [![img](https://i1.hdslb.com/bfs/archive/9e0462e12bf77dd9bbe8085d0d809f233256fdbd.jpg)](https://www.bilibili.com/video/BV1pXEdzFECW) |
| YOLO 无人机识别与跟随 | [![img](https://i1.hdslb.com/bfs/archive/a5ac45ec76ccb7c3fb18de9c6b8df48e8abe2b54.jpg)](https://www.bilibili.com/video/BV18v8xzJE4G) |
| 吊舱 + 固定相机协同 | [![img](https://i2.hdslb.com/bfs/archive/07ac6082b7efdc2e2d200e18fc8074eec1d9cfba.jpg)](https://www.bilibili.com/video/BV1fTY7z7E5T) |
| 多种 SLAM 方法集成 | [![img](https://i1.hdslb.com/bfs/archive/f299bafc7486f71e061eb31f9f00347063e1e621.jpg)](https://www.bilibili.com/video/BV1ytMizEEdG) |

---

## 📄 延伸阅读

* 论文（arXiv）：https://arxiv.org/abs/2602.17393
* 技术笔记（Notion）：https://www.notion.so/Ros2Go2-1e3a3ea29e778044a4c9c35df4c27b22
* ROS1 参考实现：https://github.com/ShineMinxing/FusionEstimation

---

## 📨 联系方式

| 邮箱 | 单位 |
| --- | --- |
| [sunminxing20@mails.ucas.ac.cn](mailto:sunminxing20@mails.ucas.ac.cn) | 中国科学院光电技术研究所 |

> 本仓库持续开发中，欢迎 Issue / PR 交流与贡献。