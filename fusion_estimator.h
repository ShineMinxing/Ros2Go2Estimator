// // 调用示例
// // 初始化一次
// auto Robot_Estimation = CreateRobot_Estimation();

// // 按需传参
// double algo_config[999] = {0};
// algo_config[100] = 1.0;     // enable_imu_update
// algo_config[200] = 1.0;     // enable_leg_update
// algo_config[201] = 8.0;     // foot_force_threshold
// algo_config[202] = 0.10;    // min_stair_height
// algo_config[203] = 60.0;    // stair_fade_time
// algo_config[204] = 0.0;     // leg_ori_enable
// algo_config[205] = 0.001;   // leg_ori_init_weight
// algo_config[206] = 1000.0;  // leg_ori_time_weight
// algo_config[300] = 1.0;     // enable_kin_update
// algo_config[301] = 0.1709;  // kin_hip_length
// algo_config[302] = 0.26;    // kin_thigh_length
// algo_config[303] = 0.26;    // kin_calf_length
// algo_config[304] = 0.03;    // kin_foot_radius
// // 设置kinematics_matrix的默认值
// double default_kinematics[52] = {
//     0.2878,  0.07,  0.000, 0.0,  0.1709, 0.0, 0.0, 0.0, -0.26, 0.0, 0.0, -0.26, 0.03,
//     0.2878, -0.07,  0.000, 0.0, -0.1709, 0.0, 0.0, 0.0, -0.26, 0.0, 0.0, -0.26, 0.03,
//     -0.2878,  0.07,  0.000, 0.0,  0.1709, 0.0, 0.0, 0.0, -0.26, 0.0, 0.0, -0.26, 0.03,
//     -0.2878, -0.07,  0.000, 0.0, -0.1709, 0.0, 0.0, 0.0, -0.26, 0.0, 0.0, -0.26, 0.03
//     };
// memcpy(&algo_config[305], default_kinematics, sizeof(default_kinematics));

// // 随lowstate的更新触发调用
// Robot_Estimation.fusion_estimator_config(algo_config);

#ifndef __FUSION_ESTIMATOR_H_
#define __FUSION_ESTIMATOR_H_

#include <memory>
#include <vector>
#include <cmath>
#include <cstdio>
#include <iostream>

#include "Controller/ControlFrame/LowlevelState.h"
#include "GO2FusionEstimator/SensorBase.h"
#include "GO2FusionEstimator/Sensor_Legs.h"
#include "GO2FusionEstimator/Sensor_IMU.h"

using namespace DataFusion;

// 配置文件索引协议
enum ConfigIndex {
    // ============ IMU 相关 (100 - 199) ============
    // IMU 参数更新使能位 (1.0 = Enable, 0.0 = Disable)
    IndexUpdateEnableImu = 100,

    IndexImuAccPositionX = 101, 
    IndexImuAccPositionY, IndexImuAccPositionZ,
    IndexImuAccRotationRoll, IndexImuAccRotationPitch, IndexImuAccRotationYaw,
    
    IndexImuGyroPositionX = 110, 
    IndexImuGyroPositionY, IndexImuGyroPositionZ,
    IndexImuGyroRotationRoll, IndexImuGyroRotationPitch, IndexImuGyroRotationYaw,

    // ============ Leg 算法相关 (200 - 299) ============
    // Leg 算法参数更新使能位
    IndexUpdateEnableLeg = 200,

    IndexLegFootForceThreshold = 201,  
    IndexLegMinStairHeight,  
    IndexLegStairFadeTime,  

    IndexLegOrientationEnable, 
    IndexLegOrientationInitialWeight, 
    IndexLegOrientationTimeWeight,

    // ============ Kinematics 运动学相关 (300 - 399) ============
    // 运动学参数更新使能位 (注意：如果为0，则使用构造函数中的默认值)
    IndexUpdateEnableKinematics = 300,

    // 4个标量参数
    IndexKinematicsHipLength,     // Par_HipLength
    IndexKinematicsThighLength,   // Par_ThighLength
    IndexKinematicsCalfLength,    // Par_CalfLength
    IndexKinematicsFootLength,    // Par_FootLength

    // 4x13 矩阵参数起始位置 (占用 52 个位置: 305 ~ 356)
    // 顺序：Row 0 (0~12), Row 1 (0~12), Row 2 (0~12), Row 3 (0~12)
    IndexKinematicsMatrixStart = 305 
};

// quaternion(w,x,y,z) -> roll/pitch/yaw，等价 tf2::Matrix3x3(q).getRPY
static inline void quat_wxyz_to_rpy(double w, double x, double y, double z,
                                   double &roll, double &pitch, double &yaw)
{
    const double n = std::sqrt(w*w + x*x + y*y + z*z);
    if (n < 1e-12) { roll = pitch = yaw = 0.0; return; }
    w /= n; x /= n; y /= n; z /= n;

    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (w * y - z * x);
    if (std::fabs(sinp) >= 1.0) pitch = std::copysign(M_PI / 2.0, sinp);
    else pitch = std::asin(sinp);

    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

class FusionEstimatorCore
{
public:
    FusionEstimatorCore()
    {
        // 与 ROS2 一致：两个 EstimatorPortN
        for (int i = 0; i < 2; ++i) {
            auto *ptr = new EstimatorPortN;
            StateSpaceModel_Go2_Initialization(ptr);
            sensors.emplace_back(ptr);
        }

        imu_acc     = std::make_shared<SensorIMUAcc>    (sensors[0]);
        imu_gyro    = std::make_shared<SensorIMUMagGyro>(sensors[1]);
        legs_pos    = std::make_shared<SensorLegsPos>   (sensors[0]);
        legs_ori    = std::make_shared<SensorLegsOri>   (sensors[1]);
        legs_ori->SetLegsPosRef(legs_pos.get());

        // 开关（只保留必要）
        imu_enable  = true;
        legpos_en   = true;
        legori_en   = false;

        yaw_correct = 0.0;
    }

    ~FusionEstimatorCore()
    {
        for (auto* p : sensors) delete p;
        sensors.clear();
    }

    FusionEstimatorCore(const FusionEstimatorCore&) = delete;
    FusionEstimatorCore& operator=(const FusionEstimatorCore&) = delete;

    FusionEstimatorCore(FusionEstimatorCore&&) noexcept = default;
    FusionEstimatorCore& operator=(FusionEstimatorCore&&) noexcept = default;

    void fusion_estimator_config(const double config[999])
    {
        // 1. IMU 参数更新 (检查使能位)
        if (config[IndexUpdateEnableImu] == 1) 
        {
            imu_acc->SensorPosition[0] = config[IndexImuAccPositionX];
            imu_acc->SensorPosition[1] = config[IndexImuAccPositionY];
            imu_acc->SensorPosition[2] = config[IndexImuAccPositionZ];

            {
                double r = config[IndexImuAccRotationRoll]  * M_PI / 180.0;
                double p = config[IndexImuAccRotationPitch] * M_PI / 180.0;
                double y = config[IndexImuAccRotationYaw]   * M_PI / 180.0;
                imu_acc->SensorQuaternion = 
                    Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());
                imu_acc->SensorQuaternionInv = imu_acc->SensorQuaternion.inverse();
            }

            imu_gyro->SensorPosition[0] = config[IndexImuGyroPositionX];
            imu_gyro->SensorPosition[1] = config[IndexImuGyroPositionY];
            imu_gyro->SensorPosition[2] = config[IndexImuGyroPositionZ];

            {
                double r = config[IndexImuGyroRotationRoll]  * M_PI / 180.0;
                double p = config[IndexImuGyroRotationPitch] * M_PI / 180.0;
                double y = config[IndexImuGyroRotationYaw]   * M_PI / 180.0;
                imu_gyro->SensorQuaternion = 
                    Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());
                imu_gyro->SensorQuaternionInv = imu_gyro->SensorQuaternion.inverse();
            }
            std::cout << "[Config] IMU Parameters Updated.\n";
        }

        // 2. Leg 算法参数更新 (检查使能位)
        if (config[IndexUpdateEnableLeg] == 1)
        {
            legs_pos->FootEffortThreshold       = config[IndexLegFootForceThreshold];
            legs_pos->Environement_Height_Scope = config[IndexLegMinStairHeight];
            legs_pos->Data_Fading_Time          = config[IndexLegStairFadeTime];

            legori_en = (config[IndexLegOrientationEnable] == 1);
            legs_ori->legori_init_weight = config[IndexLegOrientationInitialWeight];
            legs_ori->legori_time_weight = config[IndexLegOrientationTimeWeight];
            std::cout << "[Config] Leg Algo Parameters Updated.\n";
        }

        // 3. 运动学参数更新 (检查使能位)
        if (config[IndexUpdateEnableKinematics] == 1)
        {
            // 更新 4 个标量
            legs_pos->Par_HipLength   = config[IndexKinematicsHipLength];
            legs_pos->Par_ThighLength = config[IndexKinematicsThighLength];
            legs_pos->Par_CalfLength  = config[IndexKinematicsCalfLength];
            legs_pos->Par_FootLength  = config[IndexKinematicsFootLength];

            // 更新 4x13 矩阵 (直接搬运)
            int idx = IndexKinematicsMatrixStart;
            for(int r = 0; r < 4; ++r) {
                for(int c = 0; c < 13; ++c) {
                    legs_pos->KinematicParams(r, c) = config[idx++];
                }
            }
            std::cout << "[Config] Kinematic Parameters Updated (Scalar + Matrix).\n";
        }
    }

    Odometer fusion_estimator(const LowlevelState& st)
    {
        const double CurrentTimestamp = 1e-3 * static_cast<double>(st.imu.timestamp);

        // ---------------- 1) IMU：对应 ROS2 imu_callback ----------------
        if (imu_enable) {
            double msg_acc[100] = {0};
            double msg_rpy[100] = {0};

            // acc -> [2,5,8]
            msg_acc[3*0 + 2] = st.imu.accelerometer[0];
            msg_acc[3*1 + 2] = st.imu.accelerometer[1];
            msg_acc[3*2 + 2] = st.imu.accelerometer[2];

            // quat(wxyz) -> rpy
            double roll, pitch, yaw;
            const float* q = st.imu.quaternion; // 你已经确认是 [w,x,y,z]
            quat_wxyz_to_rpy(q[0], q[1], q[2], q[3], roll, pitch, yaw);

            // rpy(+yaw_correct) -> [0,3,6]
            msg_rpy[3*0] = roll;
            msg_rpy[3*1] = pitch;
            msg_rpy[3*2] = yaw + yaw_correct;

            // gyro -> [1,4,7]
            msg_rpy[3*0 + 1] = st.imu.gyroscope[0];
            msg_rpy[3*1 + 1] = st.imu.gyroscope[1];
            msg_rpy[3*2 + 1] = st.imu.gyroscope[2];

            imu_acc->SensorDataHandle(msg_acc, CurrentTimestamp);
            imu_gyro->SensorDataHandle(msg_rpy, CurrentTimestamp);
        }

        // ---------------- 2) Joint：对应 ROS2 joint_callback ----------------
        if (legpos_en || legori_en) {
            double joint[100] = {0};

            // 布局：[0..11] q, [12..23] dq, [24..35] tau
            int desired_joints[] = {0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14};
            for (int i = 0; i < 12; ++i) {
                const auto& m = st.motorState[desired_joints[i]];
                joint[0 + i] = m.q;
                joint[12 + i] = m.dq;
                joint[24 + i] = m.tauEst;
            }

            if (legpos_en) {
                legs_pos->SensorDataHandle(joint, CurrentTimestamp);
            }

            if (legori_en) {
                const double last_yaw = sensors[1]->EstimatedState[6] - yaw_correct;

                legs_ori->SensorDataHandle(joint, CurrentTimestamp);

                // ROS2 原逻辑：更新 yaw_correct
                yaw_correct = sensors[1]->Double_Par[99] - last_yaw;

                if (!imu_enable) {
                    sensors[1]->EstimatedState[6] = sensors[1]->Double_Par[99];
                }
            }
        }

        // ---------------- 3) 输出 Odometer ----------------
        Odometer odom;
        odom.x = static_cast<float>(sensors[0]->EstimatedState[0]);
        odom.y = static_cast<float>(sensors[0]->EstimatedState[3]);
        odom.z = static_cast<float>(sensors[0]->EstimatedState[6]);

        odom.angularX = static_cast<float>(sensors[1]->EstimatedState[0]);
        odom.angularY = static_cast<float>(sensors[1]->EstimatedState[3]);
        odom.angularZ = static_cast<float>(sensors[1]->EstimatedState[6]);
        
        return odom;
    }

private:
    std::vector<EstimatorPortN*> sensors;

    std::shared_ptr<SensorIMUAcc>     imu_acc;
    std::shared_ptr<SensorIMUMagGyro> imu_gyro;
    std::shared_ptr<SensorLegsPos>    legs_pos;
    std::shared_ptr<SensorLegsOri>    legs_ori;

    bool imu_enable, legpos_en, legori_en;

    double legori_init_weight, legori_time_weight;

    double yaw_correct;
};

inline FusionEstimatorCore CreateRobot_Estimation()
{
    return FusionEstimatorCore{};
}

#endif  /* __FUSION_ESTIMATOR_H_ */
