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
    int64_t timestamp;
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
    IMU imu;            // imu
    Odometer odometer;  // 里程计
    MotorState motorState[MOTOR_NUM]; // 电机状态
};

#endif /* __CONTROL_FRAME_LOWLEVELSTATE_H__ */
