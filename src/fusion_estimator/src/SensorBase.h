/*
Author：Sun Minxing
School: Institute of Optics And Electronics, Chinese Academy of Science
Email： 401435318@qq.com
*/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>
#include <iomanip>
#include <cmath>
#include "unitree/idl/go2/LowState_.hpp"
#include "Estimator/EstimatorPortN.h"
#include "Estimator/Cpp_Estimators/Eigen/Dense"

using namespace Eigen;

namespace DataFusion
{
  extern Eigen::MatrixXd Est_StateXYZ;         // Estimated States [Xposition Xvelocity Xacceleration Yp Yv Ya Zp Zv Za]
  extern Eigen::MatrixXd Est_StateRPY;         // Estimated States [ROLLorientation ROLLvelocity ROLLacceleration Po Pv Pa Yo Yv Ya]
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

        rclcpp::Clock ros_clock(RCL_SYSTEM_TIME);
        CurrentTime = ros_clock.now();
        CurrentTimestamp = CurrentTime.seconds();
        StateSpaceModel->StateUpdateTimestamp = CurrentTimestamp;

        Est_StateXYZ = Eigen::MatrixXd::Zero(9, 1); 
        Est_StateRPY = Eigen::MatrixXd::Zero(9, 1); 
        Est_Quaternion.setIdentity();
        Est_QuaternionInv.setIdentity();
      }

      virtual ~Sensors() = default;  
      virtual void SensorDataHandle(const unitree_go::msg::dds_::LowState_& low_state){}

    protected:

      EstimatorPortN* StateSpaceModel;
      double SensorPosition[3] = {0,0,0};
      Eigen::Quaterniond SensorQuaternion;
      Eigen::Quaterniond SensorQuaternionInv;
      Eigen::Quaterniond Est_QuaternionTemp1, Est_QuaternionTemp2, Est_QuaternionTemp3;
      Eigen::Vector3d Est_BodyAngleVel, Est_SensorWorldPosition, Est_SensorWorldVelocity, Est_SensorPosition, Est_Vector3dTemp1, Est_Vector3dTemp2;
      rclcpp::Time CurrentTime;
      double Observation[9] = {0};
      double CurrentTimestamp;
      double R_diag[9] = {1,1,1,1,1,1,1,1,1};

      void UpdateEst_Quaternion(Eigen::MatrixXd* StateRPY);
      void ObservationCorrect_Acceleration(double* Observation);
      void ObservationCorrect_Orientation(double* Observation);
      void ObservationCorrect_AngularVelocity(double* Observation);

  };
}