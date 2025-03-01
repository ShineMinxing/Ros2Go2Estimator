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

    void ObtainUrdfParameter();
    double FootHipCorrectPar[6] = {1, 1, 1, 1, 1, 1};

    protected:

    Eigen::Matrix<double, 4, 13> KinematicParams;
    double LatestJointAngle[4][3]={0}, LatestJointVelocity[4][3]={0};
    double LastJointAngle[4][3]={0}, LastJointVelocity[4][3]={0};
    double FootEffortThreshold = 1;
    bool FootIsOnGround = true, FootWasOnGround = true, FootLanding = false;
    int FootfallPositionRecordIsInitiated = 0;
    double FootfallPositionRecord[4][3]={0};
    double Par_HipLength, Par_ThighLength, Par_CalfLength, Par_FootLength;

    void Joint2HipFoot(int LegNumber, double *Observation);

  };
}