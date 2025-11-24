#pragma once

#include "SensorBase.h"

using namespace Eigen;

namespace DataFusion
{
  class SensorIMUAcc : public Sensors
  {
    public:

    SensorIMUAcc(EstimatorPortN* StateSpaceModel_):Sensors(StateSpaceModel_)
    {
      double AngleCorrect[3] = {0};
      AngleCorrect[0] = 0 * M_PI / 180.0;
      AngleCorrect[1] = 1.7 * M_PI / 180.0;
      AngleCorrect[2] = 0 * M_PI / 180.0;
      Eigen::AngleAxisd rollAngle(AngleCorrect[0], Eigen::Vector3d::UnitX());  // 绕 X 轴旋转
      Eigen::AngleAxisd pitchAngle(AngleCorrect[1], Eigen::Vector3d::UnitY()); // 绕 Y 轴旋转
      Eigen::AngleAxisd yawAngle(AngleCorrect[2], Eigen::Vector3d::UnitZ());   // 绕 Z 轴旋转
      SensorQuaternion = yawAngle * pitchAngle * rollAngle;
      SensorQuaternionInv = SensorQuaternion.inverse();

      SensorPosition[0] = -0.02557;
      SensorPosition[1] = 0;
      SensorPosition[2] = 0.04232;
    }

    void SensorDataHandle(double* Message, double Time)  override;

    protected:

  };

  class SensorIMUMagGyro : public Sensors
  {
    public:

    SensorIMUMagGyro(EstimatorPortN* StateSpaceModel_):Sensors(StateSpaceModel_)
    {
      double AngleCorrect[3] = {0};
      AngleCorrect[0] = 0 * M_PI / 180.0;
      AngleCorrect[1] = 1.7 * M_PI / 180.0;
      AngleCorrect[2] = 0 * M_PI / 180.0;
      Eigen::AngleAxisd rollAngle(AngleCorrect[0], Eigen::Vector3d::UnitX());  // 绕 X 轴旋转
      Eigen::AngleAxisd pitchAngle(AngleCorrect[1], Eigen::Vector3d::UnitY()); // 绕 Y 轴旋转
      Eigen::AngleAxisd yawAngle(AngleCorrect[2], Eigen::Vector3d::UnitZ());   // 绕 Z 轴旋转
      SensorQuaternion = yawAngle * pitchAngle * rollAngle;
      SensorQuaternionInv = SensorQuaternion.inverse();

      SensorPosition[0] = -0.02557;
      SensorPosition[1] = 0;
      SensorPosition[2] = 0.04232;
    }

    void SensorDataHandle(double* Message, double Time) override;
    void OrientationCorrect();

    protected:

  };
}