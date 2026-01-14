/*
Author：Sun Minxing
School: Institute of Optics And Electronics, Chinese Academy of Science
Email： 401435318@qq.com
*/
#pragma once

#include <fstream>
#include <memory>
#include <iostream>
#include <iomanip>
#include <cmath>
#include "EstimatorFrame/Estimator/EstimatorPortN.h"
#include "EstimatorFrame/Estimator/Cpp_Estimators/Eigen/Dense"

using namespace Eigen;

namespace DataFusion
{
extern Eigen::Quaterniond Est_Quaternion;    // Body Orientation Quaternion
  extern Eigen::Quaterniond Est_QuaternionInv; // Body Orientation Quaternion Inverse

  class Sensors
  {
    public:

      Sensors(EstimatorPortN* StateSpaceModel_)
      {
        std::cout << std::fixed << std::setprecision(4);
        
        StateSpaceModel = StateSpaceModel_;
        SensorQuaternion = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
        SensorQuaternionInv = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

        for(int i = 0; i < StateSpaceModel->Nz; i++)
          StateSpaceModel->Matrix_R[i*StateSpaceModel->Nz+i] = R_diag[i];

        Est_Quaternion.setIdentity();
        Est_QuaternionInv.setIdentity();
      }

      virtual ~Sensors() = default;  
      virtual void SensorDataHandle(double* Message, double Time) {}
      double SensorPosition[3] = {0,0,0};
      Eigen::Quaterniond SensorQuaternion;
      Eigen::Quaterniond SensorQuaternionInv;

    protected:

      EstimatorPortN* StateSpaceModel;
      Eigen::Quaterniond Est_QuaternionTemp1, Est_QuaternionTemp2, Est_QuaternionTemp3;
      Eigen::Vector3d Est_BodyAngleVel, Est_SensorWorldPosition, Est_SensorWorldVelocity, Est_SensorPosition, Est_Vector3dTemp1, Est_Vector3dTemp2;

      double Observation[9] = {0};
      double ObservationTime = 0;
      double R_diag[9] = {1,1,1,1,1,1,1,1,1};

      void UpdateEst_Quaternion();
      void ObservationCorrect_Position();
      void ObservationCorrect_Velocity();
      void ObservationCorrect_Acceleration();
      void ObservationCorrect_Orientation();
      void ObservationCorrect_AngularVelocity();
      void ObservationCorrect_AngularAcceleration();

  };
}