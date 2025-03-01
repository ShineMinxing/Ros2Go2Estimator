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
      }

      void SensorDataHandle(const unitree_go::msg::dds_::LowState_& low_state) override;

    protected:

  };

  class SensorIMUMagGyro : public Sensors
  {
    public:

      SensorIMUMagGyro(EstimatorPortN* StateSpaceModel_):Sensors(StateSpaceModel_)
      {
      }

      void SensorDataHandle(const unitree_go::msg::dds_::LowState_& low_state) override;

    protected:

  };
}