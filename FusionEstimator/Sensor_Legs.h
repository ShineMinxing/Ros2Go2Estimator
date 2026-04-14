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

      UseGo2P();
    }
        
    ~SensorLegsPos()
    {
      if (IKVelCKF_Inited_) {
        StateSpaceModel_IKVel_EstimatorPortTermination(&IKVelCKF_);
        IKVelCKF_Inited_ = false;
      }
    }

    double KinematicParams[4][13];
    double Par_HipLength = 0, Par_ThighLength = 0, Par_CalfLength = 0, Par_WheelRadius = 0;
    double FootEffortThreshold = -80.0, Environement_Height_Scope = 0.08, Data_Fading_Time = 1200.0;

    // ========= 预设：Go2点足 =========
    static constexpr double Go2P_PARAM[4][13] = {
      {  0.1934, -0.0465,  0.000,   0.0, -0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022 },
      {  0.1934,  0.0465,  0.000,   0.0,  0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022 },
      { -0.1934, -0.0465,  0.000,   0.0, -0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022 },
      { -0.1934,  0.0465,  0.000,   0.0,  0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022 }
    };
    static constexpr double Go2P_HIP   = 0.0955;
    static constexpr double Go2P_THIGH = 0.213;
    static constexpr double Go2P_CALF  = 0.213 + 0.022;   // 把点足的半径放在这里
    static constexpr double Go2P_WHEEL = 0.00;            // 把轮子的半径长度放在这里
    static constexpr double Go2P_Height= 0.08;
    static constexpr double Go2P_FORCE = -1.0;

    void UseGo2P()
    {
      std::memcpy(KinematicParams, Go2P_PARAM, sizeof(KinematicParams));
      Par_HipLength = Go2P_HIP; Par_ThighLength = Go2P_THIGH; Par_CalfLength = Go2P_CALF; Par_WheelRadius = Go2P_WHEEL;
      Environement_Height_Scope = Go2P_Height;
      FootEffortThreshold = Go2P_FORCE;

      if (!IKVelCKF_Inited_) {
        StateSpaceModel_IKVel_Initialization(&IKVelCKF_);
        IKVelCKF_Inited_ = true;
      }
      for (int i = 0; i < 4; ++i) {
        IKVelLegInited_[i] = false;
        IKVelLastT_[i] = 0.0;
      }
      IKVelCKF_.Double_Par[1] = Par_HipLength;
      IKVelCKF_.Double_Par[2] = Par_ThighLength;
      IKVelCKF_.Double_Par[3] = Par_CalfLength;
      IKVelCKF_.Double_Par[4] = Par_WheelRadius;
    }
    
    // ========= 预设：中狗点足MP / 大狗轮足LW / 中狗轮足MW =========
    static constexpr double MP_PARAM[4][13] = {
      {  0.2878,  0.07,  0.000,   0.0,  0.1709,  0.0,   0.0,  0.0, -0.26,  0.0,  0.0, -0.26,  0.03 },
      {  0.2878, -0.07,  0.000,   0.0, -0.1709,  0.0,   0.0,  0.0, -0.26,  0.0,  0.0, -0.26,  0.03 },
      { -0.2878,  0.07,  0.000,   0.0,  0.1709,  0.0,   0.0,  0.0, -0.26,  0.0,  0.0, -0.26,  0.03 },
      { -0.2878, -0.07,  0.000,   0.0, -0.1709,  0.0,   0.0,  0.0, -0.26,  0.0,  0.0, -0.26,  0.03 }
    };
    static constexpr double MP_HIP   = 0.1709;
    static constexpr double MP_THIGH = 0.26;
    static constexpr double MP_CALF  = 0.26 + 0.03;
    static constexpr double MP_WHEEL = 0.00;
    static constexpr double MP_Height= 0.08;
    static constexpr double MP_FORCE = -80;

    static constexpr double LW_PARAM[4][13] = {
      {  0.3405,  0.1,  -0.0666,   0.0,  0.1522,  0.0,  0.0,  0.0, -0.27,  0.0,  0.0, -0.351,  0.195/2 },
      {  0.3405, -0.1,  -0.0666,   0.0, -0.1522,  0.0,  0.0,  0.0, -0.27,  0.0,  0.0, -0.351,  0.195/2 },
      { -0.3405,  0.1,  -0.0666,   0.0,  0.1522,  0.0,  0.0,  0.0, -0.27,  0.0,  0.0, -0.351,  0.195/2 },
      { -0.3405, -0.1,  -0.0666,   0.0, -0.1522,  0.0,  0.0,  0.0, -0.27,  0.0,  0.0, -0.351,  0.195/2 }
    };
    static constexpr double LW_HIP   = 0.1522;
    static constexpr double LW_THIGH = 0.27;
    static constexpr double LW_CALF  = 0.351;
    static constexpr double LW_WHEEL = 0.195/2;
    static constexpr double LW_Height= 0.10;
    static constexpr double LW_FORCE = -120;

    static constexpr double MW_PARAM[4][13] = {
      {  0.2878,  0.07,  0.000,   0.0,  0.1709,  0.0,   0.0,  0.0, -0.26,  0.0,  0.0, -0.26,  0.195/2 },
      {  0.2878, -0.07,  0.000,   0.0, -0.1709,  0.0,   0.0,  0.0, -0.26,  0.0,  0.0, -0.26,  0.195/2 },
      { -0.2878,  0.07,  0.000,   0.0,  0.1709,  0.0,   0.0,  0.0, -0.26,  0.0,  0.0, -0.26,  0.195/2 },
      { -0.2878, -0.07,  0.000,   0.0, -0.1709,  0.0,   0.0,  0.0, -0.26,  0.0,  0.0, -0.26,  0.195/2 }
    };
    static constexpr double MW_HIP   = 0.1709;
    static constexpr double MW_THIGH = 0.26;
    static constexpr double MW_CALF  = 0.26;
    static constexpr double MW_WHEEL = 0.195/2;
    static constexpr double MW_Height= 0.03;
    static constexpr double MW_FORCE = -85;

    void UseMP()
    {
      std::memcpy(KinematicParams, MP_PARAM, sizeof(KinematicParams));
      Par_HipLength = MP_HIP; Par_ThighLength = MP_THIGH; Par_CalfLength = MP_CALF; Par_WheelRadius = MP_WHEEL;
      Environement_Height_Scope = MP_Height;
      FootEffortThreshold = MP_FORCE;
      
      if (!IKVelCKF_Inited_) {
        StateSpaceModel_IKVel_Initialization(&IKVelCKF_);
        IKVelCKF_Inited_ = true;
      }
      for (int i = 0; i < 4; ++i) {
        IKVelLegInited_[i] = false;
        IKVelLastT_[i] = 0.0;
      }
      IKVelCKF_.Double_Par[1] = Par_HipLength;
      IKVelCKF_.Double_Par[2] = Par_ThighLength;
      IKVelCKF_.Double_Par[3] = Par_CalfLength;
      IKVelCKF_.Double_Par[4] = Par_WheelRadius;
    }

    void UseLW()
    {
      std::memcpy(KinematicParams, LW_PARAM, sizeof(KinematicParams));
      Par_HipLength = LW_HIP; Par_ThighLength = LW_THIGH; Par_CalfLength = LW_CALF; Par_WheelRadius = LW_WHEEL;
      Environement_Height_Scope = LW_Height;
      FootEffortThreshold = LW_FORCE;
      
      IKVelCKF_Inited_ = false;
    }
    
    void UseMW()
    {
      std::memcpy(KinematicParams, MW_PARAM, sizeof(KinematicParams));
      Par_HipLength = MW_HIP; Par_ThighLength = MW_THIGH; Par_CalfLength = MW_CALF; Par_WheelRadius = MW_WHEEL;
      Environement_Height_Scope = MW_Height;
      FootEffortThreshold = MW_FORCE;
      
      IKVelCKF_Inited_ = false;
    }

    bool JointsXYZEnable = true;
    bool JointsXYZVelocityEnable = true;

    void SensorDataHandle(double* Message, double Time)  override;
    void LoadedWeightCheck(double* Message, double Time);

    bool FootfallPositionRecordIsInitiated[4] = {0}, FootIsOnGround[4] = {0}, FootWasOnGround[4] = {0}, FootLastMotion[4] = {0}, FootLanding[4] = {0}, CalculateWeightEnable = false;
    double FootBodyEff_BF[4][3]={0}, FootBodyEff_WF[4][3]={0}, FeetEffort2BodyMotion[4][6];
    double FootBodyPos_BF[4][3]={0}, FootBodyPos_WF[4][3]={0}, FootBodyVel_WF[4][3]={0}, FootfallPositionRecord[4][4]={0}, FootfallAveragePosition[3] = {0}, FootfallProbability[4] = {0}, WheelAnglePrev[4] = {0};
    double MinimumWeight = 25.0, TimelyWeight = 25.0;
    double SlopeModeTimeThreshold  = 0.75;
    double SlopeModeAngleThreshold = 5.0 / 180.0 * M_PI;
    double SlopeModeStepHeightThreshold  = 0.03;
    double SlopeModeFootForceAccept  = 0.3;

    // ===== IKVel CKF(Estimator1003) =====
    bool IKVelEnable = false;
    EstimatorPortN IKVelCKF_;
    bool IKVelCKF_Inited_ = false;
    bool   IKVelLegInited_[4] = {false, false, false, false};
    double IKVelLastT_[4]     = {0.0, 0.0, 0.0, 0.0};
    double IKVelX_[4][6]      = {{0}};     // IK坐标（未做你那里 Observation[0]/[1] 的 x 翻转）
    double IKVelP_[4][36]     = {{0}};     // 6x6 row-major
    double FootBodyVel_CKE[4][3]={0};

    protected:

    void Joint2HipFoot(double *Message, int LegNumber);
    void FootFallPositionRecord(double *Message);
    void FeetEffort2Body(int LegNumber);
    void EstimateGroundPitchAlongHeading(double& move_dir_x, double& move_dir_y, double& move_dir_z);
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