#include "Sensor_IMU.h"

namespace DataFusion
{
    void SensorIMUAcc::SensorDataHandle(const unitree_go::msg::dds_::LowState_& low_state)
    {
        // std::cout << "Flag1 " << std::endl;

        rclcpp::Clock ros_clock(RCL_SYSTEM_TIME);
        CurrentTime = ros_clock.now();
        CurrentTimestamp = CurrentTime.seconds();
        for(int i = 0; i < 3; i++)
        {
            Observation[3*i+2] = low_state.imu_state().accelerometer()[i];
        }
        
        for(int i = 0; i < StateSpaceModel->Nx * StateSpaceModel->Nz; i++)
        {
            StateSpaceModel->Matrix_H[i] = 0;
        }
        for(int i = 0; i < 3; i++)
        {
            StateSpaceModel->Matrix_H[(3 * i + 2) * StateSpaceModel->Nx + (3 * i + 2)] = 1;
        }
            
        ObservationCorrect_Acceleration(Observation);

        StateSpaceModel1_EstimatorPort(Observation, CurrentTimestamp, StateSpaceModel);

        for(int i = 0; i < StateSpaceModel->Nx; i++)
        {
            Est_StateXYZ(i, 0) = StateSpaceModel->EstimatedState[i];
        }
    }

    void SensorIMUMagGyro::SensorDataHandle(const unitree_go::msg::dds_::LowState_& low_state)
    {
        rclcpp::Clock ros_clock(RCL_SYSTEM_TIME);
        CurrentTime = ros_clock.now();
        CurrentTimestamp = CurrentTime.seconds();
        for(int i = 0; i < 3; i++)
        {
            Observation[3*i] = low_state.imu_state().rpy()[i];
        }
        for(int i = 0; i < 3; i++)
        {
            Observation[3*i+1] = low_state.imu_state().gyroscope()[i];
        }

        for(int i = 0; i < StateSpaceModel->Nx * StateSpaceModel->Nz; i++)
        {
            StateSpaceModel->Matrix_H[i] = 0;
        }
        for(int i = 0; i < 3; i++)
        {
            StateSpaceModel->Matrix_H[(3 * i + 0) * StateSpaceModel->Nx + (3 * i + 0)] = 1;
            StateSpaceModel->Matrix_H[(3 * i + 1) * StateSpaceModel->Nx + (3 * i + 1)] = 1;
        }

        ObservationCorrect_Orientation(Observation);
        ObservationCorrect_AngularVelocity(Observation);

        StateSpaceModel2_EstimatorPort(Observation, CurrentTimestamp, StateSpaceModel);
       
        for(int i = 0; i < StateSpaceModel->Nx; i++)
        {
            Est_StateRPY(i, 0) = StateSpaceModel->EstimatedState[i];
        }

        UpdateEst_Quaternion(&Est_StateRPY);
    }
}