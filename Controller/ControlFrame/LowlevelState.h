#ifndef __CONTROL_FRAME_LOWLEVELSTATE_H__
#define __CONTROL_FRAME_LOWLEVELSTATE_H__

#include <iostream>
#include <vector>
#include <cstdint>

#define MOTOR_NUM 16

struct MotorState
{
    unsigned int mode;
    float q;
    float dq;
    float ddq;
    float tauEst;
    float temp;
    unsigned int cnt;

    MotorState()
    {
        q = 0;
        dq = 0;
        ddq = 0;
        tauEst = 0;
    }
};

struct IMU
{
    float quaternion[4];
    float gyroscope[3];
    float accelerometer[3];
    float pitch, roll, yaw;
    uint32_t timestamp;
    IMU()
    {
        for (int i = 0; i < 3; i++)
        {
            quaternion[i] = 0;
            gyroscope[i] = 0;
            accelerometer[i] = 0;
        }
        quaternion[3] = 0;
        pitch = 0.0f;
        roll = 0.0f;
        yaw = 0.0f;
    }
};

struct Odometer
{
    float x;
    float y;
    float z;
    float angularX;
    float angularY;
    float angularZ;

    Odometer()
    {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        angularX = 0.0f;
        angularY = 0.0f;
        angularZ = 0.0f;
    }
};

struct LowlevelState
{
    bool dataAvailable; // 数据可用标志
    IMU imu;            // imu
    Odometer odometer;  // 里程计
    int lidarCount;
    float output_points[442]; // jun yang add to lidar move
    float output_points1[442];
    float output_points2[442];
    MotorState motorState[MOTOR_NUM]; // 电机状态
    bool exceedLimits = false;
    bool sdkControl = false;
    bool cmdUpStairs = false;
    float load_estimated=0.0;   //估计的负重值。fixedstand里面计算和估计。
    bool isExitCharging = false;
	
    float vCmdBody[4]; // jun yang change 1107, last one is used for zero statistics
};

#endif /* __CONTROL_FRAME_LOWLEVELSTATE_H__ */
