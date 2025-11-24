#include "Sensor_IMU.h"

namespace DataFusion
{
    void SensorIMUAcc::SensorDataHandle(double* Message, double Time) 
    {
      
      ObservationTime = Time;
      // std::cout << "Flag1 " << std::endl;
      for(int i = 0; i < 9; i++ )
          Observation[i] = Message[i];
      
      for(int i = 0; i < StateSpaceModel->Nx * StateSpaceModel->Nz; i++)
      {
          StateSpaceModel->Matrix_H[i] = 0;
      }
      for(int i = 0; i < 3; i++)
      {
          StateSpaceModel->Matrix_H[(3 * i + 2) * StateSpaceModel->Nx + (3 * i + 2)] = 1;
      }
          
      ObservationCorrect_Acceleration();
      Observation[8] -= 9.43;
      
      StateSpaceModel1_EstimatorPort(Observation, ObservationTime, StateSpaceModel);

    }

    void SensorIMUMagGyro::SensorDataHandle(double* Message, double Time){

      ObservationTime = Time;
      for(int i = 0; i < 9; i++ )
          Observation[i] = Message[i];

      for(int i = 0; i < StateSpaceModel->Nx * StateSpaceModel->Nz; i++)
      {
          StateSpaceModel->Matrix_H[i] = 0;
      }
      for(int i = 0; i < 3; i++)
      {
          StateSpaceModel->Matrix_H[(3 * i + 0) * StateSpaceModel->Nx + (3 * i + 0)] = 1;
          StateSpaceModel->Matrix_H[(3 * i + 1) * StateSpaceModel->Nx + (3 * i + 1)] = 1;
      }

      ObservationCorrect_Orientation();
      ObservationCorrect_AngularVelocity();
      OrientationCorrect();


      StateSpaceModel2_EstimatorPort(Observation, ObservationTime, StateSpaceModel);

      UpdateEst_Quaternion();
    }

    void SensorIMUMagGyro::OrientationCorrect(){

      static int Est_OriRollTurningTimesRecord = 0, Est_OriYawTurningTimesRecord = 0, Est_OriPitchTurningTimesRecord = 0;
      static double Est_OrientationRecord[3] = {0};

      if (sin(Est_OrientationRecord[0]) >= 0 && Observation[0]< -0.9 * M_PI)
        Est_OriRollTurningTimesRecord += 1;
      if (sin(Est_OrientationRecord[0]) < 0 && Observation[0]> 0.9 * M_PI)
        Est_OriRollTurningTimesRecord -= 1;
  
      if (sin(Est_OrientationRecord[1]) >= 0 && Observation[3]< -0.9 * M_PI)
        Est_OriPitchTurningTimesRecord += 1;
      if (sin(Est_OrientationRecord[1]) < 0 && Observation[3]> 0.9 * M_PI)
        Est_OriPitchTurningTimesRecord -= 1;
  
      if (sin(Est_OrientationRecord[2]) >= 0 && Observation[6]< -0.9 * M_PI)
        Est_OriYawTurningTimesRecord += 1;
      if (sin(Est_OrientationRecord[2]) < 0 && Observation[6] > 0.9 * M_PI)
        Est_OriYawTurningTimesRecord -= 1;

      Est_OrientationRecord[0] = Observation[0];
      Est_OrientationRecord[1] = Observation[3];
      Est_OrientationRecord[2] = Observation[6];
  
      Observation[0] = (Observation[0]+ Est_OriRollTurningTimesRecord * 2 * M_PI);
      Observation[3] = (Observation[3]+ Est_OriPitchTurningTimesRecord * 2 * M_PI);
      Observation[6] = (Observation[6]+ Est_OriYawTurningTimesRecord * 2 * M_PI);

  }
}