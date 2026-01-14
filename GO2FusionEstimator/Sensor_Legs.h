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
      KinematicParams  << 
        0.2878,  0.07,  0.000,   0.0,  0.1709,  0.0,   0.0,  0.0, -0.26,   0.0,  0.0, -0.26,  0.03,
        0.2878, -0.07,  0.000,   0.0, -0.1709,  0.0,   0.0,  0.0, -0.26,   0.0,  0.0, -0.26,  0.03,
        -0.2878,  0.07,  0.000,   0.0,  0.1709,  0.0,   0.0,  0.0, -0.26,   0.0,  0.0, -0.26,  0.03,
        -0.2878, -0.07,  0.000,   0.0, -0.1709,  0.0,   0.0,  0.0, -0.26,   0.0,  0.0, -0.26,  0.03;
      Par_HipLength = 0.1709;
      Par_ThighLength = 0.26;
      Par_CalfLength = 0.26;
      Par_FootLength = 0.03;
    }

    void SensorDataHandle(double* Message, double Time)  override;

    Eigen::Matrix<double, 4, 13> KinematicParams;
    double Par_HipLength, Par_ThighLength, Par_CalfLength, Par_FootLength;
    double FootEffortThreshold = 8.0, Environement_Height_Scope = 0.10, Data_Fading_Time = 60.0;
    bool FootfallPositionRecordIsInitiated[4] = {0}, FootIsOnGround[4], FootWasOnGround[4], FootLanding[4];
    double LatestFeetEffort;
    double FootBodyPosition[4][3]={0}, FootfallPositionRecord[4][3]={0};

    protected:

    void Joint2HipFoot(double *Message, int LegNumber);
    void Joint2HipFoot(int LegNumber);
    void PositionCorrect(int LegnNmber);

  };

  class SensorLegsOri : public Sensors
  {
    public:

    SensorLegsOri(EstimatorPortN* StateSpaceModel_):Sensors(StateSpaceModel_){}
    void SensorDataHandle(double* Message, double Time) override;

    inline void SetLegsPosRef(class SensorLegsPos* ref){ legs_pos_ref_ = ref; }
    double legori_init_weight = 0.001, legori_time_weight = 1000.0;

    protected:

    void OrientationCorrect();
    class SensorLegsPos* legs_pos_ref_ = nullptr;
    
  };

}