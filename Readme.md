# Ros2Go2Estimator 🦾

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

**Language / 语言：** **English** | [中文](#ros2go2estimator--中文)

**Ros2Go2Estimator** is a ROS 2 (**Humble**) state-estimation repository for quadruped / biped robots on **Ubuntu 22.04**.
It provides high-accuracy odometry using **IMU + joint encoders + foot contact/force signals**, without requiring cameras or LiDAR.

---

## 📄 Paper

**Contact-Anchored Proprioceptive Odometry for Quadruped Robots** (arXiv:2602.17393)

* Paper: [https://arxiv.org/abs/2602.17393](https://arxiv.org/abs/2602.17393)

If you use this repository in research, please consider citing the paper.

---

## ✨ Key Features

| Category                      | Description                                                                                                                    |
| ----------------------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| **Biped / Quadruped Unified** | Online contact-set switching (stance legs are detected automatically); supports fast transitions between standing and walking. |
| **Full 3D & Planar 2D**       | Publishes both 6DoF odometry (`SMX/Odom`) and a gravity-flattened 2D odometry (`SMX/Odom_2D`).                                 |
| **No Exteroception Required** | Works without vision/LiDAR. See the paper and videos for representative closed-loop performance.                               |
| **Runtime Tuning**            | Key compensation parameters can be adjusted online via `ros2 param` (see `config.yaml`).                                       |

---

## 🏗️ Related Repositories

| Scope                   | Repository                                                                                           | Summary                                                              |
| ----------------------- | ---------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------- |
| **Low-level / Drivers** | [https://github.com/ShineMinxing/Ros2Go2Base](https://github.com/ShineMinxing/Ros2Go2Base)           | DDS bridge, Unitree SDK2 control, pointcloud→LaserScan, TF utilities |
| **Odometry**            | **Ros2Go2Estimator (this repo)**                                                                     | Pure proprioceptive fusion, publishes `SMX/Odom` / `SMX/Odom_2D`     |
| **SLAM / Mapping**      | [https://github.com/ShineMinxing/Ros2SLAM](https://github.com/ShineMinxing/Ros2SLAM)                 | Integrations for Cartographer 3D, KISS-ICP, FAST-LIO2, Point-LIO     |
| **Voice / LLM**         | [https://github.com/ShineMinxing/Ros2Chat](https://github.com/ShineMinxing/Ros2Chat)                 | Offline ASR + OpenAI Chat + TTS                                      |
| **Vision**              | [https://github.com/ShineMinxing/Ros2ImageProcess](https://github.com/ShineMinxing/Ros2ImageProcess) | Camera pipelines, spot / face / drone detection                      |
| **Gimbal Tracking**     | [https://github.com/ShineMinxing/Ros2AmovG1](https://github.com/ShineMinxing/Ros2AmovG1)             | Amov G1 gimbal control and tracking                                  |
| **Tools**               | [https://github.com/ShineMinxing/Ros2Tools](https://github.com/ShineMinxing/Ros2Tools)               | Bluetooth IMU, joystick mapping, gimbal loop, data logging           |

> ⚠️ **Clone as needed.** If you only need state estimation, this repository is sufficient. For mapping, pair it with `Ros2SLAM` and `Ros2Go2Base`.

---

## 📂 Repository Layout

```text
Ros2Go2Estimator/
├── fusion_estimator/                 # ROS2 package
│   ├── launch/                       # example launch files
│   ├── cfg/                          # URDF / robot description
│   └── src/fusion_estimator_node.cpp # ROS2 interfaces
│   └── src/GO2FusionEstimator        # pure C++ core (portable)
├── config.yaml                       # global parameters
└── Readme.md                         # this file
```

---

## ⚙️ Configuration (`config.yaml`)

| Parameter              |                    Default | Meaning                                            |
| ---------------------- | -------------------------: | -------------------------------------------------- |
| `sub_imu_topic`        |               `SMX/Go2IMU` | IMU topic                                          |
| `sub_joint_topic`      |             `SMX/Go2Joint` | joint states topic                                 |
| `sub_mode_topic`       |             `SMX/SportCmd` | reset / mode topic (optional)                      |
| `pub_estimation_topic` |           `SMX/Estimation` | internal fused state (debug)                       |
| `pub_odom_topic`       |                 `SMX/Odom` | 6DoF odometry                                      |
| `pub_odom2d_topic`     |              `SMX/Odom_2D` | planar odometry                                    |
| `odom_frame`           |                     `odom` | world frame                                        |
| `base_frame`           |                `base_link` | body frame                                         |
| `base_frame_2d`        |             `base_link_2D` | planar body frame                                  |
| `urdf_file`            | `cfg/go2_description.urdf` | used to parse leg kinematic parameters             |
| `imu_data_enable`      |                     `true` | enable IMU                                         |
| `leg_pos_enable`       |                     `true` | enable leg position kinematics                     |
| `leg_vel_enable`       |                     `true` | enable leg velocity kinematics                     |
| `leg_ori_enable`       |                    `false` | enable kinematics-based yaw correction (see paper) |
| `leg_velcke_enable`    |                    `false` | enable IKVel-CKF (CAPO-CKE)                        |
| `foot_force_threshold` |                     `20.0` | contact threshold (platform-dependent)             |
| `Modify_Par_*`         |                      `0.0` | online compensation terms (meters or degrees)      |

---

## ⚙️ Build & Run

```bash
# 1) Clone into your workspace
cd ~/ros2_ws/LeggedRobot/src
git clone --recursive https://github.com/ShineMinxing/Ros2Go2Estimator.git

# 2) Build
cd .. && colcon build --packages-select fusion_estimator
source install/setup.bash

# 3) Run
ros2 run fusion_estimator fusion_estimator_node
```

---

## 📑 ROS 2 Interfaces

```text
/fusion_estimator_node (rclcpp)
├─ Publishes
│   • SMX/Odom         nav_msgs/Odometry (odom → base_link)
│   • SMX/Odom_2D      nav_msgs/Odometry (odom → base_link_2D)
│   • SMX/Estimation   custom / debug
├─ Subscribes
│   • SMX/Go2IMU       sensor_msgs/Imu
│   • SMX/Go2Joint     std_msgs/Float64MultiArray (q, dq, foot_force)
│   • SMX/SportCmd     std_msgs/Float64MultiArray (reset/mode)
└─ TF publication is handled by Ros2Go2Base / message_handle
```

---

## 🧠 Algorithm Overview (High Level)

1. **Contact detection**: detect stance legs from force/threshold logic.
2. **Forward kinematics**: compute foot-end position/velocity in the body frame from URDF and joint measurements.
3. **Contact anchoring**: record touchdown footfall points and use them as intermittent world-frame constraints during stance.
4. **Height stabilization**: support-plane height clustering with time-decayed confidence to reduce long-horizon elevation drift.
5. **Optional IKVel-CKF (CAPO-CKE)**: inverse-kinematics CKF to suppress encoder-induced velocity spikes.
6. **Optional yaw stabilization**: multi-contact geometric consistency to arrest IMU yaw drift during prolonged standing (see paper).
7. **2D projection**: publish an IMU-free planar odometry (`SMX/Odom_2D`) by flattening roll/pitch.

For full derivations, see the paper.

---

## 🎥 Videos

| Topic                                        | Link                                                                                         |
| -------------------------------------------- | -------------------------------------------------------------------------------------------- |
| Odometry-only mapping (morphology switching) | [https://www.bilibili.com/video/BV1UtQfYJExu](https://www.bilibili.com/video/BV1UtQfYJExu)   |
| Indoor walking (0.5%–1% error)               | [https://www.bilibili.com/video/BV18Q9JYEEdn/](https://www.bilibili.com/video/BV18Q9JYEEdn/) |
| Stair climbing (height error < 5 cm)         | [https://www.bilibili.com/video/BV1VV9ZYZEcH/](https://www.bilibili.com/video/BV1VV9ZYZEcH/) |
| Outdoor walking (380 m, 3.3% error)          | [https://www.bilibili.com/video/BV1BhRAYDEsV/](https://www.bilibili.com/video/BV1BhRAYDEsV/) |
| Voice interaction + navigation               | [https://www.bilibili.com/video/BV1HCQBYUEvk/](https://www.bilibili.com/video/BV1HCQBYUEvk/) |
| Face tracking + laser spot tracking          | [https://www.bilibili.com/video/BV1faG1z3EFF/](https://www.bilibili.com/video/BV1faG1z3EFF/) |
| AR glasses head-following                    | [https://www.bilibili.com/video/BV1pXEdzFECW](https://www.bilibili.com/video/BV1pXEdzFECW)   |
| YOLO drone tracking                          | [https://www.bilibili.com/video/BV18v8xzJE4G](https://www.bilibili.com/video/BV18v8xzJE4G)   |
| Gimbal + fixed camera collaboration          | [https://www.bilibili.com/video/BV1fTY7z7E5T](https://www.bilibili.com/video/BV1fTY7z7E5T)   |
| Multiple SLAM integrations                   | [https://www.bilibili.com/video/BV1kjCqBEEK8](https://i0.hdslb.com/bfs/archive/f299bafc7486f71e061eb31f9f00347063e1e621.jpg)   |

---

## 📄 Further Reading

* Paper (arXiv): [https://arxiv.org/abs/2602.17393](https://arxiv.org/abs/2602.17393)
* Technical notes (Notion): [https://www.notion.so/Ros2Go2-1e3a3ea29e778044a4c9c35df4c27b22](https://www.notion.so/Ros2Go2-1e3a3ea29e778044a4c9c35df4c27b22)
* ROS1 reference implementation: [https://github.com/ShineMinxing/FusionEstimation](https://github.com/ShineMinxing/FusionEstimation)

---

## 📨 Contact

| Email                                       | Affiliation                              |
| ------------------------------------------- | ---------------------------------------- |
| [sunminxing20@mails.ucas.ac.cn](mailto:sunminxing20@mails.ucas.ac.cn) | Institute of Optics and Electronics, CAS |

> This repository is under active development. Issues and PRs are welcome.

---

# Ros2Go2Estimator 🦾 (中文)

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

**语言 / Language：** [English](#ros2go2estimator-) | **中文**

**Ros2Go2Estimator** 是四足/双足机器人在 **ROS 2 Humble / Ubuntu 22.04** 上的**估计算法仓库**，主要使用 **IMU + 关节编码器 + 足端接触/力信号** 输出高精度里程计（不依赖相机或激光雷达）。

---

## 📄 论文

**Contact-Anchored Proprioceptive Odometry for Quadruped Robots** (arXiv:2602.17393)

* 论文链接：[https://arxiv.org/abs/2602.17393](https://arxiv.org/abs/2602.17393)

---

## ✨ 算法特性

| 类别               | 说明                                                   |
| ---------------- | ---------------------------------------------------- |
| **双足/四足通用**      | 行进中自动识别支撑足，无需手动切模式；支持站立/行走快速切换                       |
| **全 3D & 平面 2D** | 同时发布 6DoF 里程计（`SMX/Odom`）与压平后的 2D 里程计（`SMX/Odom_2D`） |
| **零依赖外部传感器**     | 无视觉/激光传感器也可运行；闭环性能参考论文与视频                            |
| **运行时参数调节**      | 关键补偿量支持 `ros2 param` 在线微调（见 `config.yaml`）           |

---

## 🏗️ 生态仓库一览

| 范畴       | 仓库                                                                                                   | 功能简介                                              |
| -------- | ---------------------------------------------------------------------------------------------------- | ------------------------------------------------- |
| **底层驱动** | [https://github.com/ShineMinxing/Ros2Go2Base](https://github.com/ShineMinxing/Ros2Go2Base)           | DDS 桥、Unitree SDK2 控制、点云→Scan、TF 工具               |
| 里程计      | **Ros2Go2Estimator（本仓库）**                                                                            | 纯本体多传感器融合，发布 `SMX/Odom` / `SMX/Odom_2D`           |
| SLAM/建图  | [https://github.com/ShineMinxing/Ros2SLAM](https://github.com/ShineMinxing/Ros2SLAM)                 | 集成 Cartographer 3D、KISS-ICP、FAST-LIO2、Point-LIO 等 |
| 语音/LLM   | [https://github.com/ShineMinxing/Ros2Chat](https://github.com/ShineMinxing/Ros2Chat)                 | 离线 ASR + OpenAI Chat + 语音合成                       |
| 图像处理     | [https://github.com/ShineMinxing/Ros2ImageProcess](https://github.com/ShineMinxing/Ros2ImageProcess) | 相机、光点/人脸/无人机检测                                    |
| 吊舱跟随     | [https://github.com/ShineMinxing/Ros2AmovG1](https://github.com/ShineMinxing/Ros2AmovG1)             | Amov G1 吊舱控制、目标跟踪                                 |
| 工具集      | [https://github.com/ShineMinxing/Ros2Tools](https://github.com/ShineMinxing/Ros2Tools)               | 蓝牙 IMU、手柄映射、数据采集等                                 |

> ⚠️ 按需克隆：只做状态估计可仅使用本仓库；需要建图可搭配 `Ros2SLAM` 与 `Ros2Go2Base`。

---

## 📂 本仓库结构

```text
Ros2Go2Estimator/
├── fusion_estimator/                 # ROS2 包
│   ├── launch/                       # 示例 launch
│   ├── cfg/                          # URDF / 参数
│   └── src/fusion_estimator_node.cpp # ROS2 接口
│   └── src/GO2FusionEstimator        # 纯 C++ 核心实现
├── config.yaml                       # 全局参数
└── Readme.md                         # 当前文档
```

---

## ⚙️ 安装与编译

```bash
cd ~/ros2_ws/LeggedRobot/src
git clone --recursive https://github.com/ShineMinxing/Ros2Go2Estimator.git
cd .. && colcon build --packages-select fusion_estimator
source install/setup.bash
ros2 run fusion_estimator fusion_estimator_node
```

---

## 📑 节点接口

```text
/fusion_estimator_node (rclcpp)
├─ 发布
│   • SMX/Odom         nav_msgs/Odometry (odom → base_link)
│   • SMX/Odom_2D      nav_msgs/Odometry (odom → base_link_2D)
│   • SMX/Estimation   自定义/调试
├─ 订阅
│   • SMX/Go2IMU       sensor_msgs/Imu
│   • SMX/Go2Joint     std_msgs/Float64MultiArray（q,dq,foot_force）
│   • SMX/SportCmd     std_msgs/Float64MultiArray（复位/模式）
└─ TF 发布通常由 Ros2Go2Base / message_handle 提供
```

---

## 🧠 算法概览（简述）

1. **触地检测**：基于足端力/阈值逻辑判定支撑腿。
2. **前向运动学**：由 URDF 与关节数据获得足端在机身坐标系中的位姿/速度。
3. **落足点锚定**：触地瞬间记录落足点，支撑期把接触端当作世界系锚点形成约束。
4. **高度稳定**：基于支撑平面高度聚类与时间衰减置信度，抑制长时间高度漂移。
5. **可选 IKVel-CKF（CAPO-CKE）**：用于抑制编码器速度尖峰，得到更平滑的足端速度。
6. **可选航向稳定**：多接触几何一致性抑制原地站立时 IMU yaw 漂移（详见论文）。
7. **2D 投影**：压平 roll/pitch，发布 IMU-free 的平面里程计。

完整推导请参考论文。

---

## 🎥 视频演示

| 主题                 | 链接                                                                                           |
| ------------------ | -------------------------------------------------------------------------------------------- |
| 纯里程计建图（形态切换）       | [https://www.bilibili.com/video/BV1UtQfYJExu](https://www.bilibili.com/video/BV1UtQfYJExu)   |
| 室内行走误差 0.5%–1%     | [https://www.bilibili.com/video/BV18Q9JYEEdn/](https://www.bilibili.com/video/BV18Q9JYEEdn/) |
| 爬楼梯高度误差 < 5 cm     | [https://www.bilibili.com/video/BV1VV9ZYZEcH/](https://www.bilibili.com/video/BV1VV9ZYZEcH/) |
| 户外行走 380 m 误差 3.3% | [https://www.bilibili.com/video/BV1BhRAYDEsV/](https://www.bilibili.com/video/BV1BhRAYDEsV/) |
| 语音交互+导航              | [https://www.bilibili.com/video/BV1HCQBYUEvk/](https://www.bilibili.com/video/BV1HCQBYUEvk/) |
| 跟踪人脸/光点          | [https://www.bilibili.com/video/BV1faG1z3EFF/](https://www.bilibili.com/video/BV1faG1z3EFF/) |
| AR眼镜引导                    | [https://www.bilibili.com/video/BV1pXEdzFECW](https://www.bilibili.com/video/BV1pXEdzFECW)   |
| YOLO无人机跟踪                         | [https://www.bilibili.com/video/BV18v8xzJE4G](https://www.bilibili.com/video/BV18v8xzJE4G)   |
| 多相机信号融合         | [https://www.bilibili.com/video/BV1fTY7z7E5T](https://www.bilibili.com/video/BV1fTY7z7E5T)   |
| 多种SLAM对比                   | [https://www.bilibili.com/video/BV1kjCqBEEK8](https://i0.hdslb.com/bfs/archive/f299bafc7486f71e061eb31f9f00347063e1e621.jpg)   |

---

## 📨 联系我们

| 邮箱                                          | 单位           |
| ------------------------------------------- | ------------ |
| [sunminxing20@mails.ucas.ac.cn](mailto:sunminxing20@mails.ucas.ac.cn) | 中国科学院光电技术研究所 |

> 📌 本仓库持续开发中，欢迎 Issue / PR 交流与贡献。
