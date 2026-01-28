#pragma once

#include "SensorBase.h"

namespace DataFusion
{
  class SensorIMUAcc : public Sensors
  {
    public:

    SensorIMUAcc(EstimatorPortN* StateSpaceModel_):Sensors(StateSpaceModel_)
    {
      double AngleCorrect[3] = {0};
      AngleCorrect[0] = 0 * M_PI / 180.0;
      AngleCorrect[1] = 0 * M_PI / 180.0;
      AngleCorrect[2] = 0 * M_PI / 180.0;
      eulerZYX_to_quat(AngleCorrect[0], AngleCorrect[1], AngleCorrect[2], SensorQuaternion);
      quat_normalize(SensorQuaternion);
      quat_conj(SensorQuaternion, SensorQuaternionInv);

      SensorPosition[0] = 0;
      SensorPosition[1] = 0;
      SensorPosition[2] = 0;
    }

    bool IMUAccEnable = false;

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
      AngleCorrect[1] = 0 * M_PI / 180.0;
      AngleCorrect[2] = 0 * M_PI / 180.0;
      eulerZYX_to_quat(AngleCorrect[0], AngleCorrect[1], AngleCorrect[2], SensorQuaternion);
      quat_normalize(SensorQuaternion);
      quat_conj(SensorQuaternion, SensorQuaternionInv);

      SensorPosition[0] = 0;
      SensorPosition[1] = 0;
      SensorPosition[2] = 0;
    }

    bool IMUQuaternionEnable = true;
    bool IMUGyroEnable = false;

    void SensorDataHandle(double* Message, double Time) override;
    void OrientationCorrect();

    protected:

  };
}