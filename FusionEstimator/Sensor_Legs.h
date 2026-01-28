#pragma once

#include "SensorBase.h"

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
        FootLastMotion[i] = 1;
      }

      UseLW();
    }

    double KinematicParams[4][13];
    double Par_HipLength = 0, Par_ThighLength = 0, Par_CalfLength = 0, Par_FootLength = 0;

    // ========= 两套预设：中狗点足MP / 大狗轮足LW =========
    static constexpr double MP_PARAM[4][13] = {
      {  0.2878,  0.07,  0.000,   0.0,  0.1709,  0.0,   0.0,  0.0, -0.26,  0.0,  0.0, -0.26,  0.03 },
      {  0.2878, -0.07,  0.000,   0.0, -0.1709,  0.0,   0.0,  0.0, -0.26,  0.0,  0.0, -0.26,  0.03 },
      { -0.2878,  0.07,  0.000,   0.0,  0.1709,  0.0,   0.0,  0.0, -0.26,  0.0,  0.0, -0.26,  0.03 },
      { -0.2878, -0.07,  0.000,   0.0, -0.1709,  0.0,   0.0,  0.0, -0.26,  0.0,  0.0, -0.26,  0.03 }
    };
    static constexpr double MP_HIP   = 0.1709;
    static constexpr double MP_THIGH = 0.26;
    static constexpr double MP_CALF  = 0.26 + 0.03;
    static constexpr double MP_FOOT  = 0.00;

    static constexpr double LW_PARAM[4][13] = {
      {  0.3405,  0.1,  -0.0666,   0.0,  0.1522,  0.0,  0.0,  0.0, -0.27,  0.0,  0.0, -0.351,  0.184/2 },
      {  0.3405, -0.1,  -0.0666,   0.0, -0.1522,  0.0,  0.0,  0.0, -0.27,  0.0,  0.0, -0.351,  0.184/2 },
      { -0.3405,  0.1,  -0.0666,   0.0,  0.1522,  0.0,  0.0,  0.0, -0.27,  0.0,  0.0, -0.351,  0.184/2 },
      { -0.3405, -0.1,  -0.0666,   0.0, -0.1522,  0.0,  0.0,  0.0, -0.27,  0.0,  0.0, -0.351,  0.184/2 }
    };
    static constexpr double LW_HIP   = 0.1522;
    static constexpr double LW_THIGH = 0.27;
    static constexpr double LW_CALF  = 0.351 + 0.184/2;
    static constexpr double LW_FOOT  = 0.184/2;

    void UseMP()
    {
      std::memcpy(KinematicParams, MP_PARAM, sizeof(KinematicParams));
      Par_HipLength = MP_HIP; Par_ThighLength = MP_THIGH; Par_CalfLength = MP_CALF; Par_FootLength = MP_FOOT;
    }

    void UseLW()
    {
      std::memcpy(KinematicParams, LW_PARAM, sizeof(KinematicParams));
      Par_HipLength = LW_HIP; Par_ThighLength = LW_THIGH; Par_CalfLength = LW_CALF; Par_FootLength = LW_FOOT;
    }

    bool JointsXYZEnable = true;
    bool JointsXYZVelocityEnable = false;

    void SensorDataHandle(double* Message, double Time)  override;
    void LoadedWeightCheck(double* Message);

    double FootEffortThreshold = -80.0, Environement_Height_Scope = 0.08, Data_Fading_Time = 600.0;
    bool FootfallPositionRecordIsInitiated[4] = {0}, FootIsOnGround[4] = {0}, FootWasOnGround[4] = {0}, FootLastMotion[4] = {0}, FootLanding[4] = {0}, CalculateWeightEnable = false;
    double LatestFootEffort[4][3]={0}, FeetEffort2BodyMotion[4][6], FeetVelocity2BodyMotion[4][6];
    double FootBodyPosition[4][3]={0}, FootfallPositionRecord[4][3]={0}, WheelAnglePrev[4] = {0};
    double LoadedWeight = 0, DogWeight = -400;

    protected:

    void Joint2HipFoot(double *Message, int LegNumber);
    void PositionCorrect(double *Message, int LegnNmber);
    void FeetEffort2Body(int LegNumber);
    void FeetVelocity2Body(double *Message, int LegNumber);
  };

  class SensorLegsOri : public Sensors
  {
    public:

    SensorLegsOri(EstimatorPortN* StateSpaceModel_):Sensors(StateSpaceModel_){}
    void SensorDataHandle(double* Message, double Time) override;

    inline void SetLegsPosRef(class SensorLegsPos* ref){ legs_pos_ref_ = ref; }
    double legori_init_weight = 0.001, legori_time_weight = 1000.0, legori_current_weight = 0.001, legori_correct = 0;

    bool JointsRPYEnable = false;

    protected:

    class SensorLegsPos* legs_pos_ref_ = nullptr;
    
  };

}