# Ros2Go2Estimator 🦾
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

- 一种高精度里程计解决方案，
- 基于纯运动学的双足/四足机器人位置估计算法，目前仅使用IMU、足压力传感器、关节角度和角速度，不依赖相机或Lidar，但可将信号融合进去，进一步提高估计精度；
- fusion_estimator包发布对应“base_link”的话题SMXFE/Odom和对应“base_link_2D”的话题SMXFE/Odom_2D；
- message_handel包完成SMXFE/Odom和SMXFE/Odom_2D的tf，此外，将机器狗提供的“utlidar_lidar”的话题/utlidar/cloud转换为“base_link_2D”话题/SMXFE/Scan；
- sport_control包读取joystick输入、voice_chat指令，使用unitree_sdk2提供的接口控制机器狗；
- 使用SLAM Toolbox建图时放开sport_control/launch/sport_control_launch.py的启动项ros2 run slam_toolbox async_slam_toolbox_node ......；
- 使用Nav2导航时放开sport_control/launch/sport_control_launch.py的启动项ros2 launch nav2_bringup bringup_launch.py ......；

## 📚 补充说明
- 切换两足、四足无需在估计器内做模式切换
- 两足站立行走误差率1%  
- 动态行走模式误差率0.5%
- 支持长距离定位
- 目前没有调整参数做补偿，工程使用时可进一步提升精度
- SLAM Toolbox目前是纯里程计建图，请擅长SLAM的同志自行把地图匹配加进去
- Nav2同样请自行调整，加载的地图记得改成自己的
- voice_chat监听麦克风，听到唤醒词“来福”时，开启录制，VOSK语音转文字，Deepseek API联网获取文字回复，CosyVoice2/pyttsx3文本转语音并播放。
- voice_chat被转移到我的另一个库Ros2Chat中，它可用于语音交流，但需要安装各种依赖；如长期使用，请把api_key换成您自己的https://cloud.siliconflow.cn/i/5kSHnwpA

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

4. 语音控制机器狗，DeepSeek大模型语音交流
[![实验4](https://i0.hdslb.com/bfs/archive/6aaac2a8d2726fa2c7d77f20544c9692f9fb752f.jpg)](https://www.bilibili.com/video/BV1YjQVYcEdX/)

5. 语音控制机器狗，实现意图猜测和在预建地图导航。比如说“没有纸张了”，自动执行导航‘去仓库’
[![实验5](https://i2.hdslb.com/bfs/archive/5b95c6eda3b6c9c8e0ba4124c1af9f3da10f39d2.jpg)](https://www.bilibili.com/video/BV1HCQBYUEvk/)

## ⚙️ 安装指南

- Use Ubuntu 22.04, ROS2 Humble
```bash
sudo apt install ros-humble-joy ros-humble-nav2-msgs ros-humble-slam-toolbox ros-humble-nav2-bringup portaudio19-dev ffmpeg libasound-dev python3-pyaudio python3-pip libopencv-dev ros-humble-cv-bridge
pip3 install pyaudio pydub pygame vosk pyttsx3 "openai>=1.0" --user
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone --recursive https://github.com/ShineMinxing/Ros2Go2Estimator.git
cd ..
colcon build
ros2 launch sport_control sport_control_launch.py
```
- 记得在src/joystick_control/launch/joystick_control_launch.py中，修改机器狗的网口名，我个人的是“enx00e04c8d0eff”。
- 同时按下手柄的LT、RT，解锁/锁定手柄；按住RT+左摇杆进行移动；按住RT+右摇杆进行旋转；更多操作请看joystick_control_node.cpp。

## 📄 相关文档
- 核心算法原理: [技术文档](https://github.com/ShineMinxing/FusionEstimation.git)
- 历史项目参考: [Aliengo ROS1项目](https://github.com/ShineMinxing/FusionEstimation.git)

## 📧 联系我们
``` 
博士团队: 401435318@qq.com  
研究所: 中国科学院光电技术研究所
```

> 📌 注意：当前为开发预览版，完整文档正在编写中
``