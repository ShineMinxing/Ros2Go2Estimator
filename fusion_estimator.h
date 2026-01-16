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


enum ConfigIndex {

    IndexInOrOut = 0,
    IndexStatusOK = 1,
    
    IndexLegFootForceThreshold = 4,  
    IndexLegMinStairHeight = 5,  
    IndexLegStairFadeTime = 6,  
    IndexLegOrientationEnable = 7, 
    IndexLegOrientationInitialWeight = 8, 
    IndexLegOrientationTimeWeight = 9,

    IndexUpdateEnableImu = 10, 
    IndexImuAccPositionX = 11, 
    IndexImuAccPositionY, IndexImuAccPositionZ,
    IndexImuAccRotationRoll, IndexImuAccRotationPitch, IndexImuAccRotationYaw,
    
    IndexImuGyroPositionX = 20, 
    IndexImuGyroPositionY, IndexImuGyroPositionZ,
    IndexImuGyroRotationRoll, IndexImuGyroRotationPitch, IndexImuGyroRotationYaw,

    IndexUpdateEnableKinematics = 30,
    
    IndexKinematicsHipLength,
    IndexKinematicsThighLength,
    IndexKinematicsCalfLength,
    IndexKinematicsFootLength,
    
    IndexKinematicsMatrixStart = 35 
    
};

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

    void fusion_estimator_status(double status[200])
    {
        if (status[IndexInOrOut] == 1) 
        {
            status[IndexInOrOut] = 0;
            status[IndexStatusOK] = 1;
            
            legs_pos->FootEffortThreshold       = status[IndexLegFootForceThreshold];
            legs_pos->Environement_Height_Scope = status[IndexLegMinStairHeight];
            legs_pos->Data_Fading_Time          = status[IndexLegStairFadeTime];
            legori_en = (status[IndexLegOrientationEnable] == 1);
            legs_ori->legori_init_weight = status[IndexLegOrientationInitialWeight];
            legs_ori->legori_time_weight = status[IndexLegOrientationTimeWeight];
            
            if (status[IndexUpdateEnableImu] == 1) 
            {
                imu_acc->SensorPosition[0] = status[IndexImuAccPositionX];
                imu_acc->SensorPosition[1] = status[IndexImuAccPositionY];
                imu_acc->SensorPosition[2] = status[IndexImuAccPositionZ];

                {
                    double r = status[IndexImuAccRotationRoll]  * M_PI / 180.0;
                    double p = status[IndexImuAccRotationPitch] * M_PI / 180.0;
                    double y = status[IndexImuAccRotationYaw]   * M_PI / 180.0;
                    imu_acc->SensorQuaternion = 
                        Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());
                    imu_acc->SensorQuaternionInv = imu_acc->SensorQuaternion.inverse();
                }

                imu_gyro->SensorPosition[0] = status[IndexImuGyroPositionX];
                imu_gyro->SensorPosition[1] = status[IndexImuGyroPositionY];
                imu_gyro->SensorPosition[2] = status[IndexImuGyroPositionZ];

                {
                    double r = status[IndexImuGyroRotationRoll]  * M_PI / 180.0;
                    double p = status[IndexImuGyroRotationPitch] * M_PI / 180.0;
                    double y = status[IndexImuGyroRotationYaw]   * M_PI / 180.0;
                    imu_gyro->SensorQuaternion = 
                        Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());
                    imu_gyro->SensorQuaternionInv = imu_gyro->SensorQuaternion.inverse();
                }
                status[IndexStatusOK] = status[IndexStatusOK] + 2;
                std::cout << "[Config] IMU Parameters Updated.\n";
            }

            if (status[IndexUpdateEnableKinematics] == 1)
            {
                legs_pos->Par_HipLength   = status[IndexKinematicsHipLength];
                legs_pos->Par_ThighLength = status[IndexKinematicsThighLength];
                legs_pos->Par_CalfLength  = status[IndexKinematicsCalfLength] + status[IndexKinematicsFootLength];
                legs_pos->Par_FootLength  = 0.0;

                int idx = IndexKinematicsMatrixStart;
                for(int r = 0; r < 4; ++r) {
                    for(int c = 0; c < 13; ++c) {
                        legs_pos->KinematicParams(r, c) = status[idx++];
                    }
                }
                status[IndexStatusOK] = status[IndexStatusOK] + 4;
                std::cout << "[Config] Kinematic Parameters Updated (Scalar + Matrix).\n";
            }
        }
        else{
            for(int i = 0; i < 100; i++){
                status[100+i] = sensors[0]->Double_Par[i];
            }
        }
    }

    Odometer fusion_estimator(const LowlevelState& st)
    {
        const double CurrentTimestamp = 1e-3 * static_cast<double>(st.imu.timestamp);

        if (!t0_inited_) {
            t0_inited_ = true;
            sensors[0]->StateUpdateTimestamp = CurrentTimestamp;
            sensors[1]->StateUpdateTimestamp = CurrentTimestamp;
        }

        if (imu_enable) {
            double msg_acc[100] = {0};
            double msg_rpy[100] = {0};

            msg_acc[3*0 + 2] = st.imu.accelerometer[0];
            msg_acc[3*1 + 2] = st.imu.accelerometer[1];
            msg_acc[3*2 + 2] = st.imu.accelerometer[2];

            double roll, pitch, yaw;
            const float* q = st.imu.quaternion;
            quat_wxyz_to_rpy(q[0], q[1], q[2], q[3], roll, pitch, yaw);

            msg_rpy[3*0] = roll;
            msg_rpy[3*1] = pitch;
            msg_rpy[3*2] = yaw + yaw_correct;

            msg_rpy[3*0 + 1] = st.imu.gyroscope[0];
            msg_rpy[3*1 + 1] = st.imu.gyroscope[1];
            msg_rpy[3*2 + 1] = st.imu.gyroscope[2];

            imu_acc->SensorDataHandle(msg_acc, CurrentTimestamp);
            imu_gyro->SensorDataHandle(msg_rpy, CurrentTimestamp);
        }
        
        if (legpos_en || legori_en) {
            double joint[100] = {0};

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

                yaw_correct = sensors[1]->Double_Par[99] - last_yaw;

                if (!imu_enable) {
                    sensors[1]->EstimatedState[6] = sensors[1]->Double_Par[99];
                }
            }
        }

        sensors[0]->Double_Par[0] = sensors[0]->EstimatedState[0];
        sensors[0]->Double_Par[1] = sensors[0]->EstimatedState[1];
        sensors[0]->Double_Par[2] = sensors[0]->EstimatedState[2];
        sensors[0]->Double_Par[3] = sensors[0]->EstimatedState[3];
        sensors[0]->Double_Par[4] = sensors[0]->EstimatedState[4];
        sensors[0]->Double_Par[5] = sensors[0]->EstimatedState[5];
        sensors[0]->Double_Par[6] = sensors[0]->EstimatedState[6];
        sensors[0]->Double_Par[7] = sensors[0]->EstimatedState[7];
        sensors[0]->Double_Par[8] = sensors[0]->EstimatedState[8];
        sensors[0]->Double_Par[10] = sensors[1]->EstimatedState[0];
        sensors[0]->Double_Par[11] = sensors[1]->EstimatedState[1];
        sensors[0]->Double_Par[12] = sensors[1]->EstimatedState[2];
        sensors[0]->Double_Par[13] = sensors[1]->EstimatedState[3];
        sensors[0]->Double_Par[14] = sensors[1]->EstimatedState[4];
        sensors[0]->Double_Par[15] = sensors[1]->EstimatedState[5];
        sensors[0]->Double_Par[16] = sensors[1]->EstimatedState[6];
        sensors[0]->Double_Par[17] = sensors[1]->EstimatedState[7];
        sensors[0]->Double_Par[18] = sensors[1]->EstimatedState[8];

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

    bool     t0_inited_ = false;
};

inline FusionEstimatorCore CreateRobot_Estimation()
{
    return FusionEstimatorCore{};
}

#endif  /* __FUSION_ESTIMATOR_H_ */
