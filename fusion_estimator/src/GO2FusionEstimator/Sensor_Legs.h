#pragma once

#include "SensorBase.h"

using namespace Eigen;

namespace DataFusion
{
  class SensorLegsPos : public Sensors
  {
    public:

    SensorLegsPos(EstimatorPortN* StateSpaceModel_):Sensors(StateSpaceModel_)
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
    double FootEffortThreshold = 30;
    bool FootfallPositionRecordIsInitiated[4] = {0}, FootIsOnGround[4], FootWasOnGround[4], FootLanding[4];
    int LatestFeetEffort;
    double FootBodyPosition[4][3]={0}, FootfallPositionRecord[4][3]={0};

    protected:

    void Joint2HipFoot(int LegNumber);
    void PositionCorrect(int LegnNmber);

  };

  class SensorLegsOri : public Sensors
  {
    public:

    SensorLegsOri(EstimatorPortN* StateSpaceModel_):Sensors(StateSpaceModel_){}
    void SensorDataHandle(double* Message, double Time) override;

    inline void SetLegsPosRef(class SensorLegsPos* ref){ legs_pos_ref_ = ref; }

    protected:

    void OrientationCorrect();
    class SensorLegsPos* legs_pos_ref_ = nullptr;
    
  };

}