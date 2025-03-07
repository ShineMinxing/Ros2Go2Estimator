# Ros2Go2Estimator 🦾
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

- 一种高精度里程计解决方案，
- 基于纯运动学的双足/四足机器人位置估计算法，
- 不依赖相机或Lidar，但可将信号融合进去，进一步提高估计精度，
- 目前仅使用IMU、足压力传感器、关节角度和角速度，
- 里程计被发布于话题/SMXFE_odom，frame_id是"SMXFE_odom"。

## 📚 项目亮点
- 切换两足、四足无需在估计器内做模式切换
- 两足站立行走误差率1%  
- 动态行走模式误差率0.5%
- 支持长距离定位
- 目前没有调整参数做补偿，工程使用时可进一步提升精度

## 🎥 视频演示
### 最新进展(点击图片进入视频)
1. 站立行走误差1%，四足行走误差0.5%
[![主演示视频](https://i1.hdslb.com/bfs/archive/10e501bc7a93c77c1c3f41f163526b630b0afa3f.jpg)](https://www.bilibili.com/video/BV18Q9JYEEdn/)

#### 实验记录
2. 爬楼梯高度误差小于5cm
[![实验1](https://i0.hdslb.com/bfs/archive/c469a3dd37522f6b7dcdbdbb2c135be599eefa7b.jpg)](https://www.bilibili.com/video/BV1VV9ZYZEcH/)

3. 长距离测试，受磁场变化影响，380米运动偏差3.3%
[![实验2](https://i0.hdslb.com/bfs/archive/481731d2db755bbe087f44aeb3f48db29c159ada.jpg)](https://www.bilibili.com/video/BV1BhRAYDEsV/)

## ⚙️ 安装指南
```bash
git clone --recursive https://github.com/ShineMinxing/Ros2Go2Estimator.git
cd Ros2Go2Estimator
colcon build
ros2 launch joystick_control joystick_control_launch.py
```
- 记得在src/joystick_control/launch/joystick_control_launch.py中，修改机器狗的网口名，我的是“enx00e04c8d0eff”。
- 同时按下手柄的LT、RT，解锁/锁定手柄；按住RT+左摇杆进行移动；按住RT+右摇杆进行旋转；更多操作请看joystick_control_node.cpp。

## 📄 相关文档
- 核心算法原理: [技术白皮书](https://github.com/ShineMinxing/FusionEstimation.git)
- 历史项目参考: [FusionEstimation项目](https://github.com/ShineMinxing/FusionEstimation.git)

## 📧 联系我们
``` 
博士团队: 401435318@qq.com  
研究所: 中国科学院光电技术研究所
```

> 📌 注意：当前为开发预览版，完整文档正在编写中
``