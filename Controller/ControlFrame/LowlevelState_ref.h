#ifndef __CONTROL_FRAME_LOWLEVELSTATE_H__
#define __CONTROL_FRAME_LOWLEVELSTATE_H__

#include <iostream>
#include <vector>

#include "Controller/ControlFrame/MathTools.h"
#include "Controller/ControlFrame/MathTypes.h"

#include "Input/CmdPanel.h"
#include "Controller/ControlFrame/ControlEnum.h"

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
    float quaternion[4]; // w, x, y, z
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

    RotMat getRotMat()
    {
        Quat quat;
        quat << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return quatToRotMat(quat);
    }

    Vec3 getAcc()
    {
        Vec3 acc;
        acc << accelerometer[0], accelerometer[1], accelerometer[2];
        return acc;
    }

    Vec3 getGyro()
    {
        Vec3 gyro;
        gyro << gyroscope[0], gyroscope[1], gyroscope[2];
        return gyro;
    }

    Quat getQuat()
    {
        Quat q;
        q << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return q;
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
    IMU imu;
    bool imuDataAvailable;


    bool dataAvailable; // 数据可用标志
    
    Odometer odometer;  // 里程计
    int lidarCount;
    std::vector<Eigen::Vector3d> lidarPoints;
    float output_points[442]; // jun yang add to lidar move
    float output_points1[442];
    float output_points2[442];
    MotorState motorState[MOTOR_NUM]; // 电机状态
    UserCommand userCmd;              // 输入用户指令
    UserValue userValue;              // 摇杆
    UserValue last_userValue;         // 上次遥感值，用于平滑
    bool exceedLimits = false;
    bool sdkControl = false;
    bool cmdUpStairs = false;
    float load_estimated=0.0;   //估计的负重值。fixedstand里面计算和估计。
    bool isExitCharging = false;
    bool motorRestartPowerOn = false;
    bool exitChargingFinish = false;
	bool chargemode = false;  //进入充电模式标志， fixdown会受影响。
    bool fixedstand_canMove = false; //进入fixedstand模式是否能够移动标志
    float vCmdBody[4]; // jun yang change 1107, last one is used for zero statistics

    Vec34 getQ34()
    {
        Vec34 qLegs;
        for (int i(0); i < 4; ++i)
        {
            qLegs.col(i)(0) = motorState[4 * i].q;
            qLegs.col(i)(1) = motorState[4 * i + 1].q;
            qLegs.col(i)(2) = motorState[4 * i + 2].q;
        }
        return qLegs;
    }

    Vec34 getQd34()
    {
        Vec34 qdLegs;
        for (int i(0); i < 4; ++i)
        {
            qdLegs.col(i)(0) = motorState[4 * i].dq;
            qdLegs.col(i)(1) = motorState[4 * i + 1].dq;
            qdLegs.col(i)(2) = motorState[4 * i + 2].dq;
        }
        return qdLegs;
    }

    Vec44 getQ44()
    {
        Vec44 qLegs;
        for (int i(0); i < 4; ++i)
        {
            qLegs.col(i)(0) = motorState[4 * i].q;
            qLegs.col(i)(1) = motorState[4 * i + 1].q;
            qLegs.col(i)(2) = motorState[4 * i + 2].q;
            qLegs.col(i)(3) = motorState[4 * i + 3].q;
        }
        return qLegs;
    }

    Vec44 getQd44()
    {
        Vec44 qdLegs;
        for (int i(0); i < 4; ++i)
        {
            qdLegs.col(i)(0) = motorState[4 * i].dq;
            qdLegs.col(i)(1) = motorState[4 * i + 1].dq;
            qdLegs.col(i)(2) = motorState[4 * i + 2].dq;
            qdLegs.col(i)(2) = motorState[4 * i + 3].dq;
        }
        return qdLegs;
    }

    RotMat getRotMat()
    {
        return imu.getRotMat();
    }

    Vec3 getAcc()
    {
        return imu.getAcc();
    }

    Vec3 getGyro()
    {
        return imu.getGyro();
    }

    Vec3 getAccGlobal()
    {
        return getRotMat() * getAcc();
    }

    Vec3 getGyroGlobal()
    {
        return getRotMat() * getGyro();
    }

    double getYaw()
    {
        return rotMatToRPY(getRotMat())(2);
    }

    double getDYaw()
    {
        return getGyroGlobal()(2);
    }

    void setQ12(Vec12 q)
    {
        for (int i(0); i < 12; ++i)
        {
            motorState[i].q = q(i);
        }
    }

    void setQ(Vec16 q)
    {
        for (int i(0); i < 16; ++i)
        {
            motorState[i].q = q(i);
        }
    }
};

#endif /* __CONTROL_FRAME_LOWLEVELSTATE_H__ */
