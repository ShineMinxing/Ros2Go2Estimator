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

    Eigen::Matrix<double, 4, 13> KinematicParams;
    double Par_HipLength, Par_ThighLength, Par_CalfLength, Par_FootLength;
    int FootfallPositionRecordIsInitiated[4] = {0};

    protected:

    double FootEffortThreshold = 20;
    bool FootIsOnGround[4], FootWasOnGround[4], FootLanding[4];
    int LatestFeetEffort;
    double FootfallPositionRecord[4][3]={0};

    void Joint2HipFoot(int LegNumber);
    void PositionCorrect(int LegnNmber);

  };
}