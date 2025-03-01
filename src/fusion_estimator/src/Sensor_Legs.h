#pragma once

#include "SensorBase.h"
#include <urdf_parser/urdf_parser.h>
#include <memory>
#include <fstream>

using namespace Eigen;

namespace DataFusion
{
  class SensorLegs : public Sensors
  {
    public:

      SensorLegs(EstimatorPortN* StateSpaceModel_):Sensors(StateSpaceModel_)
      {
        ObtainUrdfParameter();
      }

      void SensorDataHandle(const unitree_go::msg::dds_::LowState_& low_state) override;

      // 默认值
      Eigen::Vector3d KinematicPar_FLHipJoint = {0.1934, 0.0465, 0.001};
      Eigen::Vector3d KinematicPar_FLThighJoint = {0, 0.0955, 0};
      Eigen::Vector3d KinematicPar_FLCalfJoint = {0, 0, -0.213};
      Eigen::Vector3d KinematicPar_FLFootJoint = {0, 0, -0.213};
      double KinematicPar__FLFoot = 0.022;


      void ObtainUrdfParameter();

    protected:

  };
}