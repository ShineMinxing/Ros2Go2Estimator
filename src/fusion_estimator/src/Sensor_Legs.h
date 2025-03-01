#pragma once

#include "SensorBase.h"

using namespace Eigen;

namespace DataFusion
{
  class SensorLegs : public Sensors
  {
    public:

    SensorLegs(EstimatorPortN* StateSpaceModel_):Sensors(StateSpaceModel_)
    {
      for(int i=0;i<4;i++)
      {
        FootIsOnGround[i] = 1;
        FootWasOnGround[i] = 1;
        FootLanding[i] = 0;
      }
    }

    void SensorDataHandle(double* Message, double Time)  override;

    double FootHipCorrectPar[6] = {1, 1, 1, 1, 1, 1};
    Eigen::Matrix<double, 4, 13> KinematicParams;
    double Par_HipLength, Par_ThighLength, Par_CalfLength, Par_FootLength;

    protected:

    double LatestJointAngle[4][3]={0}, LatestJointVelocity[4][3]={0};
    double LastJointAngle[4][3]={0}, LastJointVelocity[4][3]={0};
    double FootEffortThreshold = 20;
    bool FootIsOnGround[4], FootWasOnGround[4], FootLanding[4];
    int FootfallPositionRecordIsInitiated = 0;
    double FootfallPositionRecord[4][3]={0};

    void Joint2HipFoot(int LegNumber);
    void PositionCorrect(int LegnNmber);

  };
}