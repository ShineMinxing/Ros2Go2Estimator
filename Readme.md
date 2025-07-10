# Ros2Go2Estimator 🦾
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

- 一种高精度里程计解决方案,
- 基于纯运动学的双足/四足机器人位置估计算法，目前仅使用IMU、足压力传感器、关节角度和角速度，不依赖相机或Lidar，但可将信号融合进去，进一步提高估计精度;
- 使用config.yaml进行话题名称设置.

## 📚 补充说明
- 切换两足、四足无需在估计器内做模式切换;
- 目前没有调整参数做补偿，工程使用时可进一步提升精度;
- dds_rostopic包将宇树dds提供的信息转换和发布为标准ros2话题;
- fusion_estimator包发布对应“base_link”的话题SMX/Odom和对应“base_link_2D”的话题SMX/Odom_2D;
- message_handle包完成SMX/Odom和SMX/Odom_2D的tf，此外，将frame“utlidar_lidar”的pointcloud2转换为“base_link_2D”话题SMX/Scan;
- sport_control包读取joystick输入和其他指令，使用unitree_sdk2提供的接口控制机器狗;
- 使用SLAM Toolbox建图时额外ros2 launch sport_control slam_launch.py;
- 使用Nav2导航时额外ros2 launch sport_control nav_launch.py;
- 使用Amov机架跟踪时额外ros2 launch sport_control g1_launch.py;
- SLAM Toolbox目前是纯里程计建图，请擅长SLAM的同志自行把地图匹配加进去;
- Nav2同样请自行调整，加载的地图记得改成自己的;
- 也适用于Ubuntu20.04 foxy系统，把apt install的软件改为-foxy-即可;

## 🎥 视频演示
### 最新进展(点击图片进入视频)
纯里程计站立/四足切换建图效果
[![主演示视频](https://i1.hdslb.com/bfs/archive/4f60453cb37ce5e4f593f03084dbecd0fdddc27e.jpg)](https://www.bilibili.com/video/BV1UtQfYJExu)

#### 实验记录
1. 站立行走误差1%，四足行走误差0.5%
[![实验1](https://i1.hdslb.com/bfs/archive/10e501bc7a93c77c1c3f41f163526b630b0afa3f.jpg)](https://www.bilibili.com/video/BV18Q9JYEEdn/)

2. 爬楼梯高度误差小于5cm
[![实验2](https://i0.hdslb.com/bfs/archive/c469a3dd37522f6b7dcdbdbb2c135be599eefa7b.jpg)](https://www.bilibili.com/video/BV1VV9ZYZEcH/)

3. 长距离测试，受磁场变化影响，380米运动偏差3.3%
[![实验3](https://i0.hdslb.com/bfs/archive/481731d2db755bbe087f44aeb3f48db29c159ada.jpg)](https://www.bilibili.com/video/BV1BhRAYDEsV/)

4. 语音控制机器狗，实现意图猜测和在预建地图导航。比如说“没有纸张了”，自动执行导航‘去仓库’
[![实验4](https://i2.hdslb.com/bfs/archive/5b95c6eda3b6c9c8e0ba4124c1af9f3da10f39d2.jpg)](https://www.bilibili.com/video/BV1HCQBYUEvk/)
- 额外安装https://github.com/ShineMinxing/Ros2Chat

5. 机器狗与吊舱的协同光点/人脸跟踪
[![实验5](https://i0.hdslb.com/bfs/archive/5496e9d0b40915c62b69701fd1e23af7d6ffe7de.jpg)](https://www.bilibili.com/video/BV1faG1z3EFF/)
- 额外安装https://github.com/ShineMinxing/Ros2ImageProcess.git
- 额外安装https://github.com/ShineMinxing/Ros2AmovG1.git

## ⚙️ 安装指南

- Use Ubuntu 22.04, ROS2 Humble
```bash
sudo apt install ros-humble-joy ros-humble-nav2-msgs ros-humble-slam-toolbox ros-humble-nav2-bringup python3-pip libopencv-dev ros-humble-cv-bridge ros-humble-image-transport ros-humble-compressed-image-transport
mkdir -p ~/ros2_ws/LeggedRobot/src && cd ~/ros2_ws/LeggedRobot/src
git clone --recursive https://github.com/ShineMinxing/Ros2Go2Estimator.git
cd ..
# 1. 搜索工程中的所有 /home/unitree/ros2_ws/LeggedRobot，替换为您的路径
# 2. 把 src/Ros2Go2Estimator/config.yaml 中的所有 br0 替换为您的网卡名，如 enxf8e43b808e06
colcon build
ros2 launch sport_control go2_launch.py
```
- 同时按下手柄的LT、RT，解锁/锁定手柄；按住RT+左摇杆进行移动；按住RT+右摇杆进行旋转；更多操作请看sport_control_node.cpp。

## 📄 相关文档
- 核心算法原理: [技术文档](https://www.notion.so/Ros2Go2-1e3a3ea29e778044a4c9c35df4c27b22)
- 历史项目参考: [Aliengo ROS1项目](https://github.com/ShineMinxing/FusionEstimation.git)

## 📧 联系我们
``` 
博士团队: 401435318@qq.com  
研究所: 中国科学院光电技术研究所
```

> 📌 注意：当前为开发预览版，完整文档正在编写中
``