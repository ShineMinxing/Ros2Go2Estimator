#include "Sensor_IMU.h"

namespace DataFusion
{
    void SensorIMUAcc::SensorDataHandle(double* Message, double Time) 
    {
      if(!IMUAccEnable)
        return;
      
      ObservationTime = Time;
      
      memcpy(Observation,  Message, (size_t)StateSpaceModel->Nz * sizeof(double));

      for(int i = 0; i < 3; i++)
      {
          StateSpaceModel->Matrix_H[(3 * i + 0) * StateSpaceModel->Nx + (3 * i + 0)] = 0;
          StateSpaceModel->Matrix_H[(3 * i + 1) * StateSpaceModel->Nx + (3 * i + 1)] = 0;
          StateSpaceModel->Matrix_H[(3 * i + 2) * StateSpaceModel->Nx + (3 * i + 2)] = 1;
      }
          
      ObservationCorrect_Acceleration();
      Observation[8] -= 9.43;
      
      StateSpaceModel_Go2_EstimatorPort(Observation, ObservationTime, StateSpaceModel);

    }

    void SensorIMUMagGyro::SensorDataHandle(double* Message, double Time)
    {      
      
      ObservationTime = Time;
      memcpy(Observation,  Message, (size_t)StateSpaceModel->Nz * sizeof(double));

      if((!IMUQuaternionEnable)&&(!IMUGyroEnable)){
        StateSpaceModel->EstimatedState[0] = Observation[0];
        StateSpaceModel->EstimatedState[3] = Observation[3];
        StateSpaceModel->EstimatedState[6] = Observation[6];
        UpdateEst_Quaternion();
      }

      for(int i = 0; i < 9; i++)
          StateSpaceModel->Matrix_H[i * StateSpaceModel->Nx + i] = 0;

      if(IMUQuaternionEnable){
        for(int i = 0; i < 3; i++)
            StateSpaceModel->Matrix_H[(3 * i + 0) * StateSpaceModel->Nx + (3 * i + 0)] = 1;
        ObservationCorrect_Orientation();
      }

      if(IMUGyroEnable){
        for(int i = 0; i < 3; i++)
            StateSpaceModel->Matrix_H[(3 * i + 1) * StateSpaceModel->Nx + (3 * i + 1)] = 1;
        ObservationCorrect_AngularVelocity();
      }
        
      OrientationCorrect();

      StateSpaceModel_Go2_EstimatorPort(Observation, ObservationTime, StateSpaceModel);

      UpdateEst_Quaternion();
    }

    void SensorIMUMagGyro::OrientationCorrect(){

      static int Est_OriRollTurningTimesRecord = 0, Est_OriYawTurningTimesRecord = 0, Est_OriPitchTurningTimesRecord = 0;
      static double Est_OrientationRecord[3] = {0};

      if (Est_OrientationRecord[0] > 0.9 * M_PI && Observation[0]< -0.9 * M_PI)
        Est_OriRollTurningTimesRecord += 1;
      if (Est_OrientationRecord[0] < -0.9 * M_PI && Observation[0]> 0.9 * M_PI)
        Est_OriRollTurningTimesRecord -= 1;
  
      if (Est_OrientationRecord[1] > 0.9 * M_PI && Observation[3]< -0.9 * M_PI)
        Est_OriPitchTurningTimesRecord += 1;
      if (Est_OrientationRecord[1] < -0.9 * M_PI && Observation[3]> 0.9 * M_PI)
        Est_OriPitchTurningTimesRecord -= 1;
  
      if (Est_OrientationRecord[2] > 0.9 * M_PI && Observation[6]< -0.9 * M_PI)
        Est_OriYawTurningTimesRecord += 1;
      if (Est_OrientationRecord[2] < -0.9 * M_PI && Observation[6] > 0.9 * M_PI)
        Est_OriYawTurningTimesRecord -= 1;

      Est_OrientationRecord[0] = Observation[0];
      Est_OrientationRecord[1] = Observation[3];
      Est_OrientationRecord[2] = Observation[6];
  
      Observation[0] = (Observation[0]+ Est_OriRollTurningTimesRecord * 2 * M_PI);
      Observation[3] = (Observation[3]+ Est_OriPitchTurningTimesRecord * 2 * M_PI);
      Observation[6] = (Observation[6]+ Est_OriYawTurningTimesRecord * 2 * M_PI);
  }
}