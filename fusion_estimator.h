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
    
    IndexDebugEnable = 2,
    IndexCheckedLegNumber = 3,  
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

            sensors[0]->Int_Par[0]              = status[IndexCheckedLegNumber];
            legs_pos->FootEffortThreshold       = status[IndexLegFootForceThreshold];
            legs_pos->Environement_Height_Scope = status[IndexLegMinStairHeight];
            legs_pos->Data_Fading_Time          = status[IndexLegStairFadeTime];
            legori_en                           = status[IndexLegOrientationEnable];
            legs_ori->legori_init_weight        = status[IndexLegOrientationInitialWeight];
            legs_ori->legori_time_weight        = status[IndexLegOrientationTimeWeight];
        }
        else if (status[IndexInOrOut] == 2){
            status[IndexInOrOut] = 0;
            status[IndexStatusOK] = status[IndexStatusOK] + 10;
            if (status[IndexStatusOK] > 999)
                status[IndexStatusOK] = 1;
                
            status[IndexDebugEnable]                 = debug_en;
            status[IndexCheckedLegNumber]            = sensors[0]->Int_Par[0] ;
            status[IndexLegFootForceThreshold]       = legs_pos->FootEffortThreshold;
            status[IndexLegMinStairHeight]           = legs_pos->Environement_Height_Scope;
            status[IndexLegStairFadeTime]            = legs_pos->Data_Fading_Time;
            status[IndexLegOrientationEnable]        = legori_en;
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
            legs_pos->FootfallPositionRecordIsInitiated[0] = false;
            legs_pos->FootfallPositionRecordIsInitiated[1] = false;
            legs_pos->FootfallPositionRecordIsInitiated[2] = false;
            legs_pos->FootfallPositionRecordIsInitiated[3] = false;
            debug_en = false;
        }
        else if (status[IndexInOrOut] == 4){
            status[IndexInOrOut] = 0;
            if(debug_en)
                debug_en = false;
            else
                debug_en = true;
        }
        else if (status[IndexInOrOut] == 5){
            status[IndexInOrOut] = 0;
            status[IndexDebugEnable] = 2;
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
        static double LastUsedTimestamp = 0, StartTimeStamp = 0;

        if (!(CurrentTimestamp - StartTimeStamp - LastUsedTimestamp < 1) || !(CurrentTimestamp - StartTimeStamp - LastUsedTimestamp >0))
            StartTimeStamp = CurrentTimestamp - LastUsedTimestamp;

        double UsedTimestamp = CurrentTimestamp - StartTimeStamp;
        LastUsedTimestamp = UsedTimestamp;

        const float* q = st.imu.quaternion;
        double roll, pitch, yaw;
        if (imu_enable) {
            // double msg_acc[9] = {0};
            double msg_rpy[9] = {0};

            // msg_acc[3*0 + 2] = st.imu.accelerometer[0];
            // msg_acc[3*1 + 2] = st.imu.accelerometer[1];
            // msg_acc[3*2 + 2] = st.imu.accelerometer[2];

            quat_wxyz_to_rpy(q[0], q[1], q[2], q[3], roll, pitch, yaw);
            
            msg_rpy[3*0] = roll;
            msg_rpy[3*1] = pitch;
            msg_rpy[3*2] = yaw + yaw_correct;

            // msg_rpy[3*0 + 1] = st.imu.gyroscope[0];
            // msg_rpy[3*1 + 1] = st.imu.gyroscope[1];
            // msg_rpy[3*2 + 1] = st.imu.gyroscope[2];

            // if(Signal_Available_Check(msg_acc,0))
            //     imu_acc->SensorDataHandle(msg_acc, UsedTimestamp);
            if(Signal_Available_Check(msg_rpy,1))
                imu_gyro->SensorDataHandle(msg_rpy, UsedTimestamp);
        }
        
        if (legpos_en || legori_en) {
            double joint[36];

            static const int desired_joints[] = {0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14};
            for (int i = 0; i < 12; ++i) {
                const auto& m = st.motorState[desired_joints[i]];
                joint[0 + i] = m.q;
                joint[12 + i] = m.dq;
                joint[24 + i] = m.tauEst;
            }

            if (legpos_en && Signal_Available_Check(joint,2)) {
                legs_pos->SensorDataHandle(joint, UsedTimestamp);
                
                if (legori_en) {
                    const double last_yaw = sensors[1]->EstimatedState[6] - yaw_correct;

                    legs_ori->SensorDataHandle(joint, UsedTimestamp);

                    yaw_correct = legs_ori->legori_correct - last_yaw;

                    if (!imu_enable) {
                        sensors[1]->EstimatedState[6] = legs_ori->legori_correct;
                    }
                }
            }

        }

        if(debug_en)
        {
            sensors[0]->Double_Par[0] = sensors[0]->EstimatedState[0];
            sensors[0]->Double_Par[1] = sensors[0]->EstimatedState[3];
            sensors[0]->Double_Par[2] = sensors[0]->EstimatedState[6];
            sensors[0]->Double_Par[3] = sensors[0]->EstimatedState[1];
            sensors[0]->Double_Par[4] = sensors[0]->EstimatedState[4];
            sensors[0]->Double_Par[5] = sensors[0]->EstimatedState[7];
            sensors[0]->Double_Par[6] = sensors[0]->EstimatedState[2];
            sensors[0]->Double_Par[7] = sensors[0]->EstimatedState[5];
            sensors[0]->Double_Par[8] = sensors[0]->EstimatedState[8];
            sensors[0]->Double_Par[9] = UsedTimestamp;

            sensors[0]->Double_Par[10] = sensors[1]->EstimatedState[0];
            sensors[0]->Double_Par[11] = sensors[1]->EstimatedState[3];
            sensors[0]->Double_Par[12] = sensors[1]->EstimatedState[6];
            sensors[0]->Double_Par[13] = sensors[1]->EstimatedState[1];
            sensors[0]->Double_Par[14] = sensors[1]->EstimatedState[4];
            sensors[0]->Double_Par[15] = sensors[1]->EstimatedState[7];
            
            sensors[0]->Double_Par[20] = st.imu.accelerometer[0];
            sensors[0]->Double_Par[21] = st.imu.accelerometer[1];
            sensors[0]->Double_Par[22] = st.imu.accelerometer[2];
            sensors[0]->Double_Par[23] = roll;
            sensors[0]->Double_Par[24] = pitch;
            sensors[0]->Double_Par[25] = yaw + yaw_correct;
            sensors[0]->Double_Par[26] = st.imu.gyroscope[0];
            sensors[0]->Double_Par[27] = st.imu.gyroscope[1];
            sensors[0]->Double_Par[28] = st.imu.gyroscope[2];

            sensors[0]->Double_Par[30] = st.motorState[4*sensors[0]->Int_Par[0] + 0].q;
            sensors[0]->Double_Par[31] = st.motorState[4*sensors[0]->Int_Par[0] + 0].dq;
            sensors[0]->Double_Par[32] = st.motorState[4*sensors[0]->Int_Par[0] + 0].tauEst;
            sensors[0]->Double_Par[33] = st.motorState[4*sensors[0]->Int_Par[0] + 1].q;
            sensors[0]->Double_Par[34] = st.motorState[4*sensors[0]->Int_Par[0] + 1].dq;
            sensors[0]->Double_Par[35] = st.motorState[4*sensors[0]->Int_Par[0] + 1].tauEst;
            sensors[0]->Double_Par[36] = st.motorState[4*sensors[0]->Int_Par[0] + 2].q;
            sensors[0]->Double_Par[37] = st.motorState[4*sensors[0]->Int_Par[0] + 2].dq;
            sensors[0]->Double_Par[38] = st.motorState[4*sensors[0]->Int_Par[0] + 2].tauEst;
            sensors[0]->Double_Par[39] = st.motorState[4*sensors[0]->Int_Par[0] + 3].q;
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

    std::shared_ptr<SensorIMUAcc>     imu_acc;
    std::shared_ptr<SensorIMUMagGyro> imu_gyro;
    std::shared_ptr<SensorLegsPos>    legs_pos;
    std::shared_ptr<SensorLegsOri>    legs_ori;

    bool imu_enable, legpos_en, legori_en, debug_en = false;
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
