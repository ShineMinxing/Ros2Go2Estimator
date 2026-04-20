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
    float XPos, YPos, ZPos;
    float XVel, YVel, ZVel;
    float XAcc, YAcc, ZAcc;
    float RollRad, PitchRad, YawRad;
    float RollVel, PitchVel, YawVel;
    float RollAcc, PitchAcc, YawAcc;
    float FootfallAverageX, FootfallAverageY, FootfallAverageYaw;
    float FLFootLanded, FRFootLanded, RLFootLanded, RRFootLanded;
    float LoadedWeight;

    Odometer()
        : XPos(0.0f), YPos(0.0f), ZPos(0.0f),
          XVel(0.0f), YVel(0.0f), ZVel(0.0f),
          XAcc(0.0f), YAcc(0.0f), ZAcc(0.0f),
          RollRad(0.0f), PitchRad(0.0f), YawRad(0.0f),
          RollVel(0.0f), PitchVel(0.0f), YawVel(0.0f),
          RollAcc(0.0f), PitchAcc(0.0f), YawAcc(0.0f),
          FootfallAverageX(0.0f), FootfallAverageY(0.0f), FootfallAverageYaw(0.0f),
          FLFootLanded(0.0f), FRFootLanded(0.0f), RLFootLanded(0.0f), RRFootLanded(0.0f),
          LoadedWeight(0.0f)
    {}
};


struct LowlevelState
{
    IMU imu;            // imu
    Odometer odometer;  // 里程计
    MotorState motorState[MOTOR_NUM]; // 电机状态
};

#endif /* __CONTROL_FRAME_LOWLEVELSTATE_H__ */
