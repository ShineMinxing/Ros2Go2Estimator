#pragma once

#include "SensorBase.h"

namespace DataFusion
{
  static constexpr int MAX_CONTACT_CHAIN = 4;
  static constexpr int MAX_CHAIN_NODE = 12;
  static constexpr int MAX_PITCH_SUM_JOINT = 8;

  enum TFJointAxis
  {
    TF_AXIS_FIXED = -1,
    TF_AXIS_X = 0,
    TF_AXIS_Y = 1,
    TF_AXIS_Z = 2
  };

  struct TFNode
  {
    int parent = -1;
    int q_index = -1;
    int dq_index = -1;
    int tau_index = -1;
    int axis = TF_AXIS_FIXED;

    double t[3] = {0.0, 0.0, 0.0};
    double q_fix[4] = {1.0, 0.0, 0.0, 0.0};

    TFNode() = default;

    TFNode(int parent_, int q_index_, int dq_index_, int tau_index_, int axis_, double x, double y, double z, double roll, double pitch, double yaw)
        : parent(parent_), q_index(q_index_), dq_index(dq_index_), tau_index(tau_index_), axis(axis_)
    {
      t[0] = x; t[1] = y; t[2] = z;
      eulerZYX_to_quat(roll, pitch, yaw, q_fix);
    }
  };

  struct LegTFChain
  {
    int node_num = 0;
    TFNode node[MAX_CHAIN_NODE];
    TFNode ee;

    double wheel_radius = 0.0;
    int wheel_q_index = -1;
    int wheel_dq_index = -1;

    int pitch_joint_num = 0;
    int pitch_q_index[MAX_PITCH_SUM_JOINT]  = {-1,-1,-1,-1,-1,-1,-1,-1};
    int pitch_dq_index[MAX_PITCH_SUM_JOINT] = {-1,-1,-1,-1,-1,-1,-1,-1};
  };

  class SensorLegsPos : public Sensors
  {
    public:

    SensorLegsPos(EstimatorPortN* StateSpaceModel_):Sensors(StateSpaceModel_)
    {
      for(int i=0;i<MAX_CONTACT_CHAIN;i++)
      {
        FootIsOnGround[i] = 1;
        FootWasOnGround[i] = 1;
        FootLanding[i] = 0;
        FootLastMotion[i] = 1;
      }

      UseGo2P();
    }
        
    ~SensorLegsPos() override = default;

    int ContactChainNum = 4;
    LegTFChain LegChains_[MAX_CONTACT_CHAIN];
    double FootEffortThreshold = -80.0, Environement_Height_Scope = 0.08, Data_Fading_Time = 1200.0;

    void UseGo2P()
    {
      // Go2 点足模型
      // TFNode(parent, q_index, dq_index, tau_index, axis, x, y, z, roll, pitch, yaw)
      // 含义：
      // 1) parent：父节点编号，-1 表示 body
      // 2) q/dq/tau_index：在 joint[48] 中的索引
      //    joint[0..15]   = q
      //    joint[16..31]  = dq
      //    joint[32..47]  = tau
      // 3) axis：该节点关节轴方向
      // 4) x,y,z：父节点坐标系下，到当前关节原点的固定平移
      // 5) roll,pitch,yaw：父节点到当前节点的固定安装旋转
      //
      // 本函数中每条腿统一采用 3 关节链：
      // node[0] : body -> q1 原点，挂 q1
      // node[1] : q1   -> q2 原点，挂 q2
      // node[2] : q2   -> q3 原点，挂 q3
      // ee      : q3   -> 足端固定点
      //
      // 点足设置原则：
      // 1) wheel_radius = 0
      // 2) ee 的 z 直接写“膝关节到足端接触点”的总长度
      // 3) pitch_joint_num = 0，因为没有轮子，不做滚动补偿
  
      for (int i = 0; i < MAX_CONTACT_CHAIN; ++i) LegChains_[i] = LegTFChain();

      ContactChainNum = 4;
      Environement_Height_Scope = 0.08;
      FootEffortThreshold = -1.0;

      // FL: q=(0,1,2), dq=(16,17,18), tau=(32,33,34)
      // body 安装点 = (0.1934, -0.0465, 0)
      // q1 为1号电机沿 X 轴旋转，q2/q3 为 Y 轴
      // q1->q2 侧向偏置 = +0.0955
      // q2->q3 大腿长度 = 0.213
      // q3->foot 足端总长度 = 0.235 = 0.213 + 0.022
      LegChains_[0].node_num = 3;
      LegChains_[0].node[0] = TFNode(-1,  0, 16, 32, TF_AXIS_X,  0.1934, -0.0465,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[0].node[1] = TFNode( 0,  1, 17, 33, TF_AXIS_Y,  0.0000,  0.0955,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[0].node[2] = TFNode( 1,  2, 18, 34, TF_AXIS_Y,  0.0000,  0.0000, -0.2130, 0.0, 0.0, 0.0);
      LegChains_[0].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2350, 0.0, 0.0, 0.0);
      LegChains_[0].wheel_radius = 0.0;
      LegChains_[0].wheel_q_index = -1;
      LegChains_[0].wheel_dq_index = -1;
      LegChains_[0].pitch_joint_num = 0;
      // FR
      LegChains_[1].node_num = 3;
      LegChains_[1].node[0] = TFNode(-1,  4, 20, 36, TF_AXIS_X,  0.1934,  0.0465,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[1].node[1] = TFNode( 0,  5, 21, 37, TF_AXIS_Y,  0.0000, -0.0955,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[1].node[2] = TFNode( 1,  6, 22, 38, TF_AXIS_Y,  0.0000,  0.0000, -0.2130, 0.0, 0.0, 0.0);
      LegChains_[1].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2350, 0.0, 0.0, 0.0);
      LegChains_[1].wheel_radius = 0.0;
      LegChains_[1].wheel_q_index = -1;
      LegChains_[1].wheel_dq_index = -1;
      LegChains_[1].pitch_joint_num = 0;
      // RL
      LegChains_[2].node_num = 3;
      LegChains_[2].node[0] = TFNode(-1,  8, 24, 40, TF_AXIS_X, -0.1934, -0.0465,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[2].node[1] = TFNode( 0,  9, 25, 41, TF_AXIS_Y,  0.0000,  0.0955,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[2].node[2] = TFNode( 1, 10, 26, 42, TF_AXIS_Y,  0.0000,  0.0000, -0.2130, 0.0, 0.0, 0.0);
      LegChains_[2].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2350, 0.0, 0.0, 0.0);
      LegChains_[2].wheel_radius = 0.0;
      LegChains_[2].wheel_q_index = -1;
      LegChains_[2].wheel_dq_index = -1;
      LegChains_[2].pitch_joint_num = 0;
      // RR
      LegChains_[3].node_num = 3;
      LegChains_[3].node[0] = TFNode(-1, 12, 28, 44, TF_AXIS_X, -0.1934,  0.0465,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[3].node[1] = TFNode( 0, 13, 29, 45, TF_AXIS_Y,  0.0000, -0.0955,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[3].node[2] = TFNode( 1, 14, 30, 46, TF_AXIS_Y,  0.0000,  0.0000, -0.2130, 0.0, 0.0, 0.0);
      LegChains_[3].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2350, 0.0, 0.0, 0.0);
      LegChains_[3].wheel_radius = 0.0;
      LegChains_[3].wheel_q_index = -1;
      LegChains_[3].wheel_dq_index = -1;
      LegChains_[3].pitch_joint_num = 0;
    }

    void UseMP()
    {
      for (int i = 0; i < MAX_CONTACT_CHAIN; ++i) LegChains_[i] = LegTFChain();

      ContactChainNum = 4;
      Environement_Height_Scope = 0.08;
      FootEffortThreshold = -80.0;

      LegChains_[0].node_num = 3;
      LegChains_[0].node[0] = TFNode(-1,  0, 16, 32, TF_AXIS_X,  0.2878,  0.0700,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[0].node[1] = TFNode( 0,  1, 17, 33, TF_AXIS_Y,  0.0000,  0.1709,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[0].node[2] = TFNode( 1,  2, 18, 34, TF_AXIS_Y,  0.0000,  0.0000, -0.2600, 0.0, 0.0, 0.0);
      LegChains_[0].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2900, 0.0, 0.0, 0.0);
      LegChains_[0].wheel_radius = 0.0;
      LegChains_[0].wheel_q_index = -1;
      LegChains_[0].wheel_dq_index = -1;
      LegChains_[0].pitch_joint_num = 0;

      LegChains_[1].node_num = 3;
      LegChains_[1].node[0] = TFNode(-1,  4, 20, 36, TF_AXIS_X,  0.2878, -0.0700,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[1].node[1] = TFNode( 0,  5, 21, 37, TF_AXIS_Y,  0.0000, -0.1709,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[1].node[2] = TFNode( 1,  6, 22, 38, TF_AXIS_Y,  0.0000,  0.0000, -0.2600, 0.0, 0.0, 0.0);
      LegChains_[1].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2900, 0.0, 0.0, 0.0);
      LegChains_[1].wheel_radius = 0.0;
      LegChains_[1].wheel_q_index = -1;
      LegChains_[1].wheel_dq_index = -1;
      LegChains_[1].pitch_joint_num = 0;

      LegChains_[2].node_num = 3;
      LegChains_[2].node[0] = TFNode(-1,  8, 24, 40, TF_AXIS_X, -0.2878,  0.0700,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[2].node[1] = TFNode( 0,  9, 25, 41, TF_AXIS_Y,  0.0000,  0.1709,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[2].node[2] = TFNode( 1, 10, 26, 42, TF_AXIS_Y,  0.0000,  0.0000, -0.2600, 0.0, 0.0, 0.0);
      LegChains_[2].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2900, 0.0, 0.0, 0.0);
      LegChains_[2].wheel_radius = 0.0;
      LegChains_[2].wheel_q_index = -1;
      LegChains_[2].wheel_dq_index = -1;
      LegChains_[2].pitch_joint_num = 0;

      LegChains_[3].node_num = 3;
      LegChains_[3].node[0] = TFNode(-1, 12, 28, 44, TF_AXIS_X, -0.2878, -0.0700,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[3].node[1] = TFNode( 0, 13, 29, 45, TF_AXIS_Y,  0.0000, -0.1709,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[3].node[2] = TFNode( 1, 14, 30, 46, TF_AXIS_Y,  0.0000,  0.0000, -0.2600, 0.0, 0.0, 0.0);
      LegChains_[3].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2900, 0.0, 0.0, 0.0);
      LegChains_[3].wheel_radius = 0.0;
      LegChains_[3].wheel_q_index = -1;
      LegChains_[3].wheel_dq_index = -1;
      LegChains_[3].pitch_joint_num = 0;
    }
    
    void UseLW()
    {
      // LW 轮足模型
      // 与点足相比，多了轮相关参数：
      // 1) wheel_radius：轮半径
      // 2) wheel_q_index / wheel_dq_index：轮电机角度/角速度在 joint[48] 中的索引
      // 3) pitch_joint_num：参与“小腿姿态补偿”的关节数
      // 4) pitch_q_index / pitch_dq_index：这些关节在 q/dq 中的索引
      //
      // 轮足设置原则：
      // 1) ee 的 z 写“膝关节到轮轴中心”或当前定义的末端刚体长度
      // 2) 轮滚动位移不在 FK 中体现，而在 FootFallPositionRecord() 中由 wheel_radius 和轮角增量补偿
      // 3) pitch_joint_num 一般取 2，对应 q2、q3，用于扣除小腿摆动带来的轮角假转动
      //
      // 重要：
      // 当前配置要求 joint[0..15] / joint[16..31] 中包含轮电机 3,7,11,15。
      // 如果外层只传 12 个关节而没有轮电机数据，则不能使用本配置。

      for (int i = 0; i < MAX_CONTACT_CHAIN; ++i) LegChains_[i] = LegTFChain();

      ContactChainNum = 4;
      Environement_Height_Scope = 0.10;
      FootEffortThreshold = -120.0;

      // FL: 腿关节 q=(0,1,2)，轮关节 q=3
      // dq=(16,17,18,19), tau=(32,33,34)
      // pitch 补偿关节取 q2,q3，即 (1,2)
      LegChains_[0].node_num = 3;
      LegChains_[0].node[0] = TFNode(-1,  0, 16, 32, TF_AXIS_X,  0.3405,  0.1000, -0.0666, 0.0, 0.0, 0.0);
      LegChains_[0].node[1] = TFNode( 0,  1, 17, 33, TF_AXIS_Y,  0.0000,  0.1522,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[0].node[2] = TFNode( 1,  2, 18, 34, TF_AXIS_Y,  0.0000,  0.0000, -0.2700, 0.0, 0.0, 0.0);
      LegChains_[0].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.3510, 0.0, 0.0, 0.0);
      LegChains_[0].wheel_radius = 0.195 / 2.0;
      LegChains_[0].wheel_q_index = 3;
      LegChains_[0].wheel_dq_index = 19;
      LegChains_[0].pitch_joint_num = 2;
      LegChains_[0].pitch_q_index[0] = 1;
      LegChains_[0].pitch_q_index[1] = 2;
      LegChains_[0].pitch_dq_index[0] = 17;
      LegChains_[0].pitch_dq_index[1] = 18;
      // FR
      LegChains_[1].node_num = 3;
      LegChains_[1].node[0] = TFNode(-1,  4, 20, 36, TF_AXIS_X,  0.3405, -0.1000, -0.0666, 0.0, 0.0, 0.0);
      LegChains_[1].node[1] = TFNode( 0,  5, 21, 37, TF_AXIS_Y,  0.0000, -0.1522,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[1].node[2] = TFNode( 1,  6, 22, 38, TF_AXIS_Y,  0.0000,  0.0000, -0.2700, 0.0, 0.0, 0.0);
      LegChains_[1].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.3510, 0.0, 0.0, 0.0);
      LegChains_[1].wheel_radius = 0.195 / 2.0;
      LegChains_[1].wheel_q_index = 7;
      LegChains_[1].wheel_dq_index = 23;
      LegChains_[1].pitch_joint_num = 2;
      LegChains_[1].pitch_q_index[0] = 5;
      LegChains_[1].pitch_q_index[1] = 6;
      LegChains_[1].pitch_dq_index[0] = 21;
      LegChains_[1].pitch_dq_index[1] = 22;
      // RL
      LegChains_[2].node_num = 3;
      LegChains_[2].node[0] = TFNode(-1,  8, 24, 40, TF_AXIS_X, -0.3405,  0.1000, -0.0666, 0.0, 0.0, 0.0);
      LegChains_[2].node[1] = TFNode( 0,  9, 25, 41, TF_AXIS_Y,  0.0000,  0.1522,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[2].node[2] = TFNode( 1, 10, 26, 42, TF_AXIS_Y,  0.0000,  0.0000, -0.2700, 0.0, 0.0, 0.0);
      LegChains_[2].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.3510, 0.0, 0.0, 0.0);
      LegChains_[2].wheel_radius = 0.195 / 2.0;
      LegChains_[2].wheel_q_index = 11;
      LegChains_[2].wheel_dq_index = 27;
      LegChains_[2].pitch_joint_num = 2;
      LegChains_[2].pitch_q_index[0] = 9;
      LegChains_[2].pitch_q_index[1] = 10;
      LegChains_[2].pitch_dq_index[0] = 25;
      LegChains_[2].pitch_dq_index[1] = 26;
      // RR
      LegChains_[3].node_num = 3;
      LegChains_[3].node[0] = TFNode(-1, 12, 28, 44, TF_AXIS_X, -0.3405, -0.1000, -0.0666, 0.0, 0.0, 0.0);
      LegChains_[3].node[1] = TFNode( 0, 13, 29, 45, TF_AXIS_Y,  0.0000, -0.1522,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[3].node[2] = TFNode( 1, 14, 30, 46, TF_AXIS_Y,  0.0000,  0.0000, -0.2700, 0.0, 0.0, 0.0);
      LegChains_[3].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.3510, 0.0, 0.0, 0.0);
      LegChains_[3].wheel_radius = 0.195 / 2.0;
      LegChains_[3].wheel_q_index = 15;
      LegChains_[3].wheel_dq_index = 31;
      LegChains_[3].pitch_joint_num = 2;
      LegChains_[3].pitch_q_index[0] = 13;
      LegChains_[3].pitch_q_index[1] = 14;
      LegChains_[3].pitch_dq_index[0] = 29;
      LegChains_[3].pitch_dq_index[1] = 30;
    }
    
    void UseMW()
    {
      for (int i = 0; i < MAX_CONTACT_CHAIN; ++i) LegChains_[i] = LegTFChain();

      ContactChainNum = 4;
      Environement_Height_Scope = 0.08;
      FootEffortThreshold = -85.0;

      LegChains_[0].node_num = 3;
      LegChains_[0].node[0] = TFNode(-1,  0, 16, 32, TF_AXIS_X,  0.2878,  0.0700,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[0].node[1] = TFNode( 0,  1, 17, 33, TF_AXIS_Y,  0.0000,  0.1709,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[0].node[2] = TFNode( 1,  2, 18, 34, TF_AXIS_Y,  0.0000,  0.0000, -0.2600, 0.0, 0.0, 0.0);
      LegChains_[0].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2600, 0.0, 0.0, 0.0);
      LegChains_[0].wheel_radius = 0.195 / 2.0;
      LegChains_[0].wheel_q_index = 3;
      LegChains_[0].wheel_dq_index = 19;
      LegChains_[0].pitch_joint_num = 2;
      LegChains_[0].pitch_q_index[0] = 1;
      LegChains_[0].pitch_q_index[1] = 2;
      LegChains_[0].pitch_dq_index[0] = 17;
      LegChains_[0].pitch_dq_index[1] = 18;

      LegChains_[1].node_num = 3;
      LegChains_[1].node[0] = TFNode(-1,  4, 20, 36, TF_AXIS_X,  0.2878, -0.0700,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[1].node[1] = TFNode( 0,  5, 21, 37, TF_AXIS_Y,  0.0000, -0.1709,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[1].node[2] = TFNode( 1,  6, 22, 38, TF_AXIS_Y,  0.0000,  0.0000, -0.2600, 0.0, 0.0, 0.0);
      LegChains_[1].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2600, 0.0, 0.0, 0.0);
      LegChains_[1].wheel_radius = 0.195 / 2.0;
      LegChains_[1].wheel_q_index = 7;
      LegChains_[1].wheel_dq_index = 23;
      LegChains_[1].pitch_joint_num = 2;
      LegChains_[1].pitch_q_index[0] = 5;
      LegChains_[1].pitch_q_index[1] = 6;
      LegChains_[1].pitch_dq_index[0] = 21;
      LegChains_[1].pitch_dq_index[1] = 22;

      LegChains_[2].node_num = 3;
      LegChains_[2].node[0] = TFNode(-1,  8, 24, 40, TF_AXIS_X, -0.2878,  0.0700,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[2].node[1] = TFNode( 0,  9, 25, 41, TF_AXIS_Y,  0.0000,  0.1709,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[2].node[2] = TFNode( 1, 10, 26, 42, TF_AXIS_Y,  0.0000,  0.0000, -0.2600, 0.0, 0.0, 0.0);
      LegChains_[2].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2600, 0.0, 0.0, 0.0);
      LegChains_[2].wheel_radius = 0.195 / 2.0;
      LegChains_[2].wheel_q_index = 11;
      LegChains_[2].wheel_dq_index = 27;
      LegChains_[2].pitch_joint_num = 2;
      LegChains_[2].pitch_q_index[0] = 9;
      LegChains_[2].pitch_q_index[1] = 10;
      LegChains_[2].pitch_dq_index[0] = 25;
      LegChains_[2].pitch_dq_index[1] = 26;

      LegChains_[3].node_num = 3;
      LegChains_[3].node[0] = TFNode(-1, 12, 28, 44, TF_AXIS_X, -0.2878, -0.0700,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[3].node[1] = TFNode( 0, 13, 29, 45, TF_AXIS_Y,  0.0000, -0.1709,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[3].node[2] = TFNode( 1, 14, 30, 46, TF_AXIS_Y,  0.0000,  0.0000, -0.2600, 0.0, 0.0, 0.0);
      LegChains_[3].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2600, 0.0, 0.0, 0.0);
      LegChains_[3].wheel_radius = 0.195 / 2.0;
      LegChains_[3].wheel_q_index = 15;
      LegChains_[3].wheel_dq_index = 31;
      LegChains_[3].pitch_joint_num = 2;
      LegChains_[3].pitch_q_index[0] = 13;
      LegChains_[3].pitch_q_index[1] = 14;
      LegChains_[3].pitch_dq_index[0] = 29;
      LegChains_[3].pitch_dq_index[1] = 30;
    }

    bool JointsXYZEnable = true;
    bool JointsXYZVelocityEnable = true;

    void SensorDataHandle(double* Message, double Time)  override;
    void LoadedWeightCheck(double* Message, double Time);

    bool FootfallPositionRecordIsInitiated[MAX_CONTACT_CHAIN] = {0}, FootIsOnGround[MAX_CONTACT_CHAIN] = {0}, FootWasOnGround[MAX_CONTACT_CHAIN] = {0}, FootLastMotion[MAX_CONTACT_CHAIN] = {0}, FootLanding[MAX_CONTACT_CHAIN] = {0}, CalculateWeightEnable = false;
    double FootBodyEff_BF[MAX_CONTACT_CHAIN][3] = {0}, FootBodyEff_WF[MAX_CONTACT_CHAIN][3] = {0}, FeetEffort2BodyMotion[MAX_CONTACT_CHAIN][6] = {0};
    double FootBodyPos_BF[MAX_CONTACT_CHAIN][3] = {0}, FootBodyPos_WF[MAX_CONTACT_CHAIN][3] = {0}, FootBodyVel_WF[MAX_CONTACT_CHAIN][3] = {0}, FootfallPositionRecord[MAX_CONTACT_CHAIN][4] = {0}, FootfallAveragePosition[3] = {0}, FootfallProbability[MAX_CONTACT_CHAIN] = {0}, WheelAnglePrev[MAX_CONTACT_CHAIN] = {0};

    double MinimumWeight = 25.0, TimelyWeight = 25.0;
    
    bool SlopeModeEnable = true;
    double SlopeModeTimeThreshold  = 1.0;
    double SlopeModeAngleThreshold = 5.0 / 180.0 * M_PI;
    double SlopeModeStepHeightThreshold  = 0.03;
    double SlopeModeFootForceAccept  = 0.5;

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