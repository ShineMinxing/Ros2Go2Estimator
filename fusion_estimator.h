#ifndef __FUSION_ESTIMATOR_H_
#define __FUSION_ESTIMATOR_H_

#include <memory>
#include <vector>
#include <cmath>
#include <cstdio>
#include <iostream>

#include "Controller/ControlFrame/LowlevelCmd.h"
#include "Controller/ControlFrame/LowlevelState.h"
#include "GO2FusionEstimator/SensorBase.h"
#include "GO2FusionEstimator/Sensor_Legs.h"
#include "GO2FusionEstimator/Sensor_IMU.h"

enum ConfigIndex {

    IndexInOrOut = 0,
    IndexStatusOK = 1,
    IndexIMUAccEnable = 2,  
    IndexIMUQuaternionEnable = 3,
    IndexIMUGyroEnable = 4,  
    IndexJointsXYZEnable = 5,
    IndexJointsVelocityXYZEnable = 6,
    IndexJointsRPYEnable = 7,
    
    IndexLoadedWeight = 9,

    IndexLegFootForceThreshold = 10,  
    IndexLegMinStairHeight = 11,
    IndexLegOrientationInitialWeight = 12, 
    IndexLegOrientationTimeWeight = 13,
};

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

        imu_acc     = std::make_shared<DataFusion::SensorIMUAcc>    (sensors[0]);
        imu_gyro    = std::make_shared<DataFusion::SensorIMUMagGyro>(sensors[1]);
        legs_pos    = std::make_shared<DataFusion::SensorLegsPos>   (sensors[0]);
        legs_ori    = std::make_shared<DataFusion::SensorLegsOri>   (sensors[1]);
        legs_ori->SetLegsPosRef(legs_pos.get());

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
            status[IndexStatusOK] = status[IndexStatusOK] + 1;

            if(!(status[IndexIMUAccEnable]||status[IndexIMUQuaternionEnable]||status[IndexIMUGyroEnable]||status[IndexJointsXYZEnable]||status[IndexJointsRPYEnable]))
                status[IndexStatusOK] = -999;

            imu_acc->IMUAccEnable               = status[IndexIMUAccEnable];
            imu_gyro->IMUQuaternionEnable       = status[IndexIMUQuaternionEnable];
            imu_gyro->IMUGyroEnable             = status[IndexIMUGyroEnable];
            legs_pos->JointsXYZEnable           = status[IndexJointsXYZEnable];
            legs_pos->JointsXYZVelocityEnable   = status[IndexJointsVelocityXYZEnable];
            legs_ori->JointsRPYEnable           = status[IndexJointsRPYEnable];

            legs_pos->LoadedWeight              = status[IndexLoadedWeight];

            legs_pos->FootEffortThreshold       = status[IndexLegFootForceThreshold];
            legs_pos->Environement_Height_Scope = status[IndexLegMinStairHeight];
            legs_ori->legori_init_weight        = status[IndexLegOrientationInitialWeight];
            legs_ori->legori_time_weight        = status[IndexLegOrientationTimeWeight];
        }
        else if (status[IndexInOrOut] == 2){
            status[IndexInOrOut] = 0;
            status[IndexStatusOK] = status[IndexStatusOK] + 10;
            if (status[IndexStatusOK] > 999)
                status[IndexStatusOK] = 1;

            status[IndexIMUAccEnable]             = imu_acc->IMUAccEnable;
            status[IndexIMUQuaternionEnable]      = imu_gyro->IMUQuaternionEnable;
            status[IndexIMUGyroEnable]            = imu_gyro->IMUGyroEnable;
            status[IndexJointsXYZEnable]          = legs_pos->JointsXYZEnable;
            status[IndexJointsVelocityXYZEnable]  = legs_pos->JointsXYZVelocityEnable;
            status[IndexJointsRPYEnable]          = legs_ori->JointsRPYEnable;
                
            status[IndexLoadedWeight]             = legs_pos->LoadedWeight;

            status[IndexLegFootForceThreshold]    = legs_pos->FootEffortThreshold;
            status[IndexLegMinStairHeight]        = legs_pos->Environement_Height_Scope;
            status[IndexLegOrientationInitialWeight] = legs_ori->legori_init_weight;
            status[IndexLegOrientationTimeWeight]    = legs_ori->legori_time_weight;
        }
        else if (status[IndexInOrOut] == 3){
            status[IndexInOrOut] = 0;
            status[IndexStatusOK] = status[IndexStatusOK] + 20;
            if (status[IndexStatusOK] > 999)
                status[IndexStatusOK] = 1;
            sensors[0]->EstimatedState[0] = 0;
            sensors[0]->EstimatedState[3] = 0;
            sensors[0]->EstimatedState[6] = 0;
            yaw_correct = - sensors[1]->EstimatedState[6];
            legs_pos->FootfallPositionRecordIsInitiated[0] = false;
            legs_pos->FootfallPositionRecordIsInitiated[1] = false;
            legs_pos->FootfallPositionRecordIsInitiated[2] = false;
            legs_pos->FootfallPositionRecordIsInitiated[3] = false;
        }
        else if (status[IndexInOrOut] == 4){
            status[IndexInOrOut] = 0;
            status[IndexStatusOK] = status[IndexStatusOK] + 40;
            if (status[IndexStatusOK] > 999)
                status[IndexStatusOK] = 1;
            legs_pos->UseMP();

        }
        else if (status[IndexInOrOut] == 5){
            status[IndexInOrOut] = 0;
            status[IndexStatusOK] = status[IndexStatusOK] + 80;
            if (status[IndexStatusOK] > 999)
                status[IndexStatusOK] = 1;
            legs_pos->UseLW();
        }
        else{
        }
    }

    Odometer fusion_estimator(const LowlevelState& st)
    {
        const double CurrentTimestamp = 1e-3 * static_cast<double>(st.imu.timestamp);
        static double LastUsedTimestamp = 0, StartTimeStamp = 0;

        if(st.imu.timestamp==0){
            
            Odometer odom;
            odom.x = static_cast<float>(0);
            odom.y = static_cast<float>(0);
            odom.z = static_cast<float>(0);

            odom.angularX = static_cast<float>(0);
            odom.angularY = static_cast<float>(0);
            odom.angularZ = static_cast<float>(0);
            
            return odom;
        }

        if (!(CurrentTimestamp - StartTimeStamp - LastUsedTimestamp < 1) || !(CurrentTimestamp - StartTimeStamp - LastUsedTimestamp >0))
            StartTimeStamp = CurrentTimestamp - LastUsedTimestamp;

        double UsedTimestamp = CurrentTimestamp - StartTimeStamp;
        LastUsedTimestamp = UsedTimestamp;

        if (imu_acc->IMUAccEnable) {
            double msg_acc[9] = {0};
            msg_acc[3*0 + 2] = static_cast<double>(st.imu.accelerometer[0]);
            msg_acc[3*1 + 2] = static_cast<double>(st.imu.accelerometer[1]);
            msg_acc[3*2 + 2] = static_cast<double>(st.imu.accelerometer[2]);
            
            if(Signal_Available_Check(msg_acc,0))
                imu_acc->SensorDataHandle(msg_acc, UsedTimestamp);
        }

        const double q[4] = {
            static_cast<double>(st.imu.quaternion[0]),
            static_cast<double>(st.imu.quaternion[1]),
            static_cast<double>(st.imu.quaternion[2]),
            static_cast<double>(st.imu.quaternion[3])
        };
        double roll, pitch, yaw;
        double msg_rpy[9] = {0};

        DataFusion::quat_to_eulerZYX(q, roll, pitch, yaw);

        msg_rpy[3*0] = roll;
        msg_rpy[3*1] = pitch;
        msg_rpy[3*2] = yaw + yaw_correct;

        msg_rpy[3*0 + 1] = static_cast<double>(st.imu.gyroscope[0]);
        msg_rpy[3*1 + 1] = static_cast<double>(st.imu.gyroscope[1]);
        msg_rpy[3*2 + 1] = static_cast<double>(st.imu.gyroscope[2]);

        if(Signal_Available_Check(msg_rpy,1))
            imu_gyro->SensorDataHandle(msg_rpy, UsedTimestamp);
        
        if (legs_pos->JointsXYZEnable||legs_pos->JointsXYZVelocityEnable){
            double joint[36];

            static const int desired_joints[] = {0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14};
            for (int i = 0; i < 12; ++i) {
                const auto& m = st.motorState[desired_joints[i]];
                joint[0 + i]  = static_cast<double>(m.q);
                joint[12 + i] = static_cast<double>(m.dq);
                joint[24 + i] = static_cast<double>(m.tauEst);
            }

            if (Signal_Available_Check(joint,2)||legs_pos->CalculateWeightEnable) {
                legs_pos->SensorDataHandle(joint, UsedTimestamp);
                
                if (legs_ori->JointsRPYEnable) {
                    const double last_yaw = sensors[1]->EstimatedState[6] - yaw_correct;

                    legs_ori->SensorDataHandle(joint, UsedTimestamp);

                    yaw_correct = legs_ori->legori_correct - last_yaw;

                    if (!imu_gyro->IMUQuaternionEnable) {
                        sensors[1]->EstimatedState[6] = legs_ori->legori_correct;
                    }
                }
            }
        }

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

    std::shared_ptr<DataFusion::SensorIMUAcc>     imu_acc;
    std::shared_ptr<DataFusion::SensorIMUMagGyro> imu_gyro;
    std::shared_ptr<DataFusion::SensorLegsPos>    legs_pos;
    std::shared_ptr<DataFusion::SensorLegsOri>    legs_ori;

    double legori_init_weight, legori_time_weight;
    double yaw_correct;

    bool Signal_Available_Check(double Signal[], int type)
    {
        static double last[3][36] = {0};
        static int Number[3] = {9,9,36};
        bool diff = false;

        for (int i = 0; i < Number[type]; ++i) {
            if (!(Signal[i] < 9999.0 && Signal[i] > -9999.0))
                return false;
            if (Signal[i] != last[type][i])
                diff = true;
        }
        if(!diff)
            return false;
        else
            for (int i = 0; i < Number[type]; ++i)
                last[type][i] = Signal[i];
        return true;
    }
};

inline FusionEstimatorCore CreateRobot_Estimation()
{
    return FusionEstimatorCore{};
}

#endif  /* __FUSION_ESTIMATOR_H_ */
