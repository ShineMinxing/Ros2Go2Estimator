#include "Sensor_Legs.h"

namespace DataFusion
{
    void SensorLegsPos::SensorDataHandle(double* Message, double Time) 
    {
        if((!JointsXYZEnable)&&(!JointsXYZVelocityEnable)){
            return;
        }

        int i, LegNumber;
        ObservationTime = Time;
        
        for(i = 0; i < StateSpaceModel->Nz; i++)
            Observation[i] = 0;

        for(LegNumber = 0; LegNumber<4; LegNumber++)
        {
            for(i = 0; i < 3; i++){
                FootBodyEff_WF[LegNumber][i] = 0;
            }
            for(i = 0; i < 6; i++){
                FeetEffort2BodyMotion[LegNumber][i] = 0;
            }
            for(i = 0; i < 3; i++)
            {
                SensorPosition[i] = KinematicParams[LegNumber][i];
            }
            
            Joint2HipFoot(Message,LegNumber);

            if(JointsXYZEnable){
                ObservationCorrect_Position();

                for(i = 0; i < 3; i++)
                {
                    StateSpaceModel->Double_Par[0 + LegNumber * 3 + i] = Observation[3 * i]; 
                    FootBodyPos_WF[LegNumber][i] = Observation[3 * i];
                }
            }

            if(JointsXYZVelocityEnable){
                ObservationCorrect_Velocity();

                for(i = 0; i < 3; i++)
                {
                    FootBodyVel_WF[LegNumber][i] = Observation[3 * i + 1];
                }
            }

            FeetEffort2Body(LegNumber);

            for(i = 0; i < 6; i++)
            {
                StateSpaceModel->Double_Par[12+LegNumber*6+i] = FeetEffort2BodyMotion[LegNumber][i]; 
            }
        }
        
        if(FootIsOnGround[0]||FootIsOnGround[1]||FootIsOnGround[2]||FootIsOnGround[3])
        {
            for(i = 0; i < 9; i++)
            {
                StateSpaceModel->Matrix_H[i * StateSpaceModel->Nx + i] = 0;
            }

            FootFallPositionRecord(Message);

            if(JointsXYZEnable){
                for(i = 0; i < 3; i++)
                {
                    StateSpaceModel->Matrix_H[(3 * i + 0) * StateSpaceModel->Nx + (3 * i + 0)] = 1;
                }
            }
            if(JointsXYZVelocityEnable){
                for(i = 0; i < 3; i++)
                {
                    StateSpaceModel->Matrix_H[(3 * i + 1) * StateSpaceModel->Nx + (3 * i + 1)] = 1;
                }
            }
            StateSpaceModel_Go2_EstimatorPort(Observation, ObservationTime, StateSpaceModel);

            double p_w[4][2];
            for (LegNumber = 0; LegNumber < 4; ++LegNumber) {
                if (FootIsOnGround[LegNumber]) {
                    p_w[LegNumber][0] = FootfallPositionRecord[LegNumber][0];
                    p_w[LegNumber][1] = FootfallPositionRecord[LegNumber][1];
                } else {
                    p_w[LegNumber][0] = StateSpaceModel->EstimatedState[0] + FootBodyPos_WF[LegNumber][0];
                    p_w[LegNumber][1] = StateSpaceModel->EstimatedState[3] + FootBodyPos_WF[LegNumber][1];
                }
            }

            const double fx = 0.5 * (p_w[0][0] + p_w[1][0]);
            const double fy = 0.5 * (p_w[0][1] + p_w[1][1]);
            const double rx = 0.5 * (p_w[2][0] + p_w[3][0]);
            const double ry = 0.5 * (p_w[2][1] + p_w[3][1]);

            // 四足中心（世界系）
            const double x_mean = 0.5 * (fx + rx);
            const double y_mean = 0.5 * (fy + ry);

            double yaw_ff = std::atan2(fy - ry, fx - rx);

            angle_unwrap(yaw_ff);

            FootfallAveragePosition[0] = x_mean;
            FootfallAveragePosition[1] = y_mean;
            FootfallAveragePosition[2] = yaw_ff; 
        }
    }

    void SensorLegsPos::LoadedWeightCheck(double* Message, double Time) 
    {
        static constexpr int WIN_T = 50;
        static constexpr int STABLE_N = 100;

        static double buf100[WIN_T] = {0.0};
        static int    buf100_i = 0;
        static int    buf100_n = 0;
        static double sum100   = 0.0;

        static int stable_cnt = 0; // 连续四足着地计数（饱和到 STABLE_N，避免溢出）

        const bool all_on_ground = (FootfallProbability[0] + FootfallProbability[1] + FootfallProbability[2] + FootfallProbability[3]) > 2.5;

        if (all_on_ground) {
            if (stable_cnt < STABLE_N) stable_cnt++;
        } else {
            stable_cnt = 0;
            buf100_i = 0; buf100_n = 0; sum100 = 0.0; 
        }

        if (stable_cnt >= STABLE_N) {
            const double fz_sum =
                FootBodyEff_WF[0][2] + FootBodyEff_WF[1][2] +
                FootBodyEff_WF[2][2] + FootBodyEff_WF[3][2];

            if (buf100_n < WIN_T) {
                buf100[buf100_i] = fz_sum;
                sum100 += fz_sum;
                buf100_n++;
            } else {
                sum100 -= buf100[buf100_i];
                buf100[buf100_i] = fz_sum;
                sum100 += fz_sum;
            }
            buf100_i++;
            if (buf100_i >= WIN_T) buf100_i = 0;

            const double mean100 = (buf100_n > 0) ? (sum100 / (double)buf100_n) : 0.0;

            TimelyWeight = - mean100 * 0.1;

            if(TimelyWeight < MinimumWeight) TimelyWeight = MinimumWeight;
        }
    }

    void SensorLegsPos::Joint2HipFoot(double *Message, int LegNumber)
    {
        double s1, s2, s3, c1, c2, c3, c23, s23, dq1, dq2, dq3;
        int SideSign, i;

        if (LegNumber == 0 || LegNumber == 2)
            SideSign = 1;
        else
            SideSign = -1;

        s1 = sin(Message[LegNumber*4+0]);
        s2 = sin(Message[LegNumber*4+1]);
        s3 = sin(Message[LegNumber*4+2]);
        c1 = cos(Message[LegNumber*4+0]);
        c2 = cos(Message[LegNumber*4+1]);
        c3 = cos(Message[LegNumber*4+2]);
        dq1 = Message[16 + LegNumber*4+0];
        dq2 = Message[16 + LegNumber*4+1];
        dq3 = Message[16 + LegNumber*4+2];

        c23 = c2 * c3 - s2 * s3;
        s23 = s2 * c3 + c2 * s3;

        Observation[0] = Par_CalfLength  * s23 + Par_ThighLength * s2;
        Observation[3] = Par_HipLength * SideSign * c1 + (Par_CalfLength + Par_WheelRadius) * (s1 * c23) + Par_ThighLength * c2 * s1;
        Observation[6] = Par_HipLength * SideSign * s1 - Par_CalfLength * (c1 * c23) - Par_ThighLength * c1 * c2 + Par_WheelRadius;

        Observation[1] = (Par_CalfLength *c23 + Par_ThighLength * c2)*dq2 + (Par_CalfLength *c23)*dq3;
        Observation[4] = ((Par_CalfLength + Par_WheelRadius) *c1*c23 + Par_ThighLength * c1*c2 - Par_HipLength*SideSign*s1)*dq1\
        + (-(Par_CalfLength + Par_WheelRadius)  * s1*s23 - Par_ThighLength * s1*s2)*dq2\
        + (-(Par_CalfLength + Par_WheelRadius)  * s1*s23)*dq3;
        Observation[7] = (Par_CalfLength *s1*c23 + Par_ThighLength * c2*s1 + Par_HipLength*SideSign*c1)*dq1\
        + (Par_CalfLength *c1*s23 + Par_ThighLength * c1*s2)*dq2\
        + (Par_CalfLength *c1*s23)*dq3;

        Observation[0] = -Observation[0];
        Observation[1] = -Observation[1];

        FootBodyPos_BF[LegNumber][0] = Observation[0] + SensorPosition[0];
        FootBodyPos_BF[LegNumber][1] = Observation[3] + SensorPosition[1];
        FootBodyPos_BF[LegNumber][2] = Observation[6] + SensorPosition[2];


        // ===== IKVel CKF(1003): 用关节角/角速度观测，对足端速度做估计与滤波 =====
        if (IKVelEnable && IKVelCKF_Inited_)
        {
            // 观测 z = [q1 q2 q3 dq1 dq2 dq3]
            double z_ik[6];
            z_ik[0] = Message[LegNumber*4 + 0];
            z_ik[1] = Message[LegNumber*4 + 1];
            z_ik[2] = Message[LegNumber*4 + 2];
            z_ik[3] = Message[16 + LegNumber*4 + 0];
            z_ik[4] = Message[16 + LegNumber*4 + 1];
            z_ik[5] = Message[16 + LegNumber*4 + 2];

            IKVelCKF_.Double_Par[0] = (double)SideSign;

            // 第一次：用当前正解出来的 (pos, vel) 作为初值
            // 注意：你这里把 Observation[0]/[1] 做了 x 翻转，所以要还原成 IKVel 状态使用的坐标
            if (!IKVelLegInited_[LegNumber])
            {
                IKVelX_[LegNumber][0] = -Observation[0]; // x
                IKVelX_[LegNumber][1] =  Observation[3]; // y
                IKVelX_[LegNumber][2] =  Observation[6]; // z
                IKVelX_[LegNumber][3] = -Observation[1]; // vx
                IKVelX_[LegNumber][4] =  Observation[4]; // vy
                IKVelX_[LegNumber][5] =  Observation[7]; // vz

                // P 初值：直接用 IKVelCKF_ 初始化后的 Matrix_P
                std::memcpy(IKVelP_[LegNumber], IKVelCKF_.Matrix_P, 36 * sizeof(double));

                IKVelLastT_[LegNumber] = ObservationTime;   // 你在 SensorDataHandle 里赋值过 ObservationTime
                IKVelLegInited_[LegNumber] = true;
            }

            // 把该腿缓存装入复用的 IKVelCKF_ 实例
            std::memcpy(IKVelCKF_.EstimatedState, IKVelX_[LegNumber], 6 * sizeof(double));
            std::memcpy(IKVelCKF_.PredictedState, IKVelX_[LegNumber], 6 * sizeof(double));
            std::memcpy(IKVelCKF_.Matrix_P,      IKVelP_[LegNumber], 36 * sizeof(double));
            IKVelCKF_.StateUpdateTimestamp = IKVelLastT_[LegNumber];

            // 调 IKVel CKF
            StateSpaceModel_IKVel_EstimatorPort(z_ik, ObservationTime, &IKVelCKF_);

            // 把结果存回该腿缓存
            std::memcpy(IKVelX_[LegNumber], IKVelCKF_.EstimatedState, 6 * sizeof(double));
            std::memcpy(IKVelP_[LegNumber], IKVelCKF_.Matrix_P,      36 * sizeof(double));
            IKVelLastT_[LegNumber] = IKVelCKF_.StateUpdateTimestamp; // 一般会等于 ObservationTime

            // 用滤波后的速度覆盖你当前的 Observation 速度
            // 仍然保持你原来的“x 轴翻转约定”
            Observation[1] = -IKVelX_[LegNumber][3]; // vx flip back
            Observation[4] =  IKVelX_[LegNumber][4]; // vy
            Observation[7] =  IKVelX_[LegNumber][5]; // vz

            FootBodyVel_CKE[LegNumber][0] = Observation[1];
            FootBodyVel_CKE[LegNumber][1] = Observation[4];
            FootBodyVel_CKE[LegNumber][2] = Observation[7];
        }


        double tau_hip   = Message[32 + LegNumber * 4 + 0];
        double tau_thigh = Message[32 + LegNumber * 4 + 1];
        double tau_knee  = Message[32 + LegNumber * 4 + 2];

        double J[3][3];

        double Jx1_raw = 0.0;
        double Jx2_raw = Par_CalfLength * c23 + Par_ThighLength * c2;
        double Jx3_raw = Par_CalfLength * c23;

        double Jy1 = (Par_CalfLength + Par_WheelRadius) * c1 * c23 + Par_ThighLength * c1 * c2 - Par_HipLength * SideSign * s1;
        double Jy2 = -(Par_CalfLength + Par_WheelRadius) * s1 * s23 - Par_ThighLength * s1 * s2;
        double Jy3 = -(Par_CalfLength + Par_WheelRadius) * s1 * s23;

        double Jz1 = Par_CalfLength * s1 * c23 + Par_ThighLength * c2 * s1 + Par_HipLength * SideSign * c1;
        double Jz2 = Par_CalfLength * c1 * s23 + Par_ThighLength * c1 * s2;
        double Jz3 = Par_CalfLength * c1 * s23;

        J[0][0] = -Jx1_raw;
        J[0][1] = -Jx2_raw;
        J[0][2] = -Jx3_raw;

        J[1][0] =  Jy1;  J[1][1] =  Jy2;  J[1][2] =  Jy3;
        J[2][0] =  Jz1;  J[2][1] =  Jz2;  J[2][2] =  Jz3;


        const double tau[3] = { tau_hip, tau_thigh, tau_knee };
        // w = J * tau
        double w[3];
        mat3_mul_vec(J, tau, w);
        // M = J * J^T
        double M[3][3];
        mat3_mul_mat3T(J, M);
        // f = inv(M) * w
        double Minv[3][3];
        double f[3] = {0,0,0};
        if (mat3_inv(M, Minv))
            mat3_mul_vec(Minv, w, FootBodyEff_BF[LegNumber]);

        quat_rot_vec3(Est_Quaternion, FootBodyEff_BF[LegNumber], FootBodyEff_WF[LegNumber]);

        if(FootBodyEff_WF[LegNumber][2] > 0.3 * FootEffortThreshold)
            FootfallProbability[LegNumber] = 0.0;
        else if(FootBodyEff_WF[LegNumber][2] < 1.3 * FootEffortThreshold)
            FootfallProbability[LegNumber] = 1.0;
        else
            FootfallProbability[LegNumber] = (FootBodyEff_WF[LegNumber][2] - 0.3 * FootEffortThreshold) / (FootEffortThreshold);

        if(FootBodyEff_WF[LegNumber][2] <= FootEffortThreshold)
        {
            FootIsOnGround[LegNumber] = true;
        }
        else
        {
            FootIsOnGround[LegNumber] = false;
        }

        if(FootIsOnGround[LegNumber] && !FootWasOnGround[LegNumber])
        {
            // std::cout << "[Check] L" << LegNumber
            //     << " t=" << ObservationTime
            //     << " FootBodyEff_WF[LegNumber][2]=" << FootBodyEff_WF[LegNumber][2]
            //     << " FootEffortThreshold=" << FootEffortThreshold
            //     << " LastStatus=" << FootWasOnGround[LegNumber]
            //     << " LastMotion=" << FootLastMotion[LegNumber]
            //     << std::endl;
            FootLanding[LegNumber] = true;
            FootLastMotion[LegNumber] = true;
        }
        else
            FootLanding[LegNumber] = false;
        
        if(!FootIsOnGround[LegNumber] && FootWasOnGround[LegNumber])
        {
            FootLastMotion[LegNumber] = false;
        }

        if(LegNumber==3&&!FootIsOnGround[0]&&!FootIsOnGround[1]&&!FootIsOnGround[2]&&Observation[6]>-0.1)
            FootIsOnGround[LegNumber] = true;

        FootWasOnGround[LegNumber] = FootIsOnGround[LegNumber];
    }

    void SensorLegsPos::FootFallPositionRecord(double *Message){

        double p_sum[3] = {0}, v_sum[3] = {0};
        int    leg_cnt = 0;
        static double ShankPitchPrev[4] = {0,0,0,0};
        double body_roll=0.0, body_pitch=0.0, body_yaw=0.0;
        quat_to_eulerZYX(Est_Quaternion, body_roll, body_pitch, body_yaw);

        double move_dir_x = 1.0, move_dir_y = 0.0, move_dir_z = 0.0;
        EstimateGroundPitchAlongHeading(move_dir_x, move_dir_y, move_dir_z);

        for (int LegNumber = 0; LegNumber < 4; ++LegNumber)
        {
            if (!FootIsOnGround[LegNumber])
                continue;
            if(!FootfallPositionRecordIsInitiated[LegNumber])
            {
                FootfallPositionRecordIsInitiated[LegNumber] = true;
                FootLanding[LegNumber]= false;
                FootfallPositionRecord[LegNumber][0] = StateSpaceModel->EstimatedState[0] + FootBodyPos_WF[LegNumber][0];
                FootfallPositionRecord[LegNumber][1] = StateSpaceModel->EstimatedState[3] + FootBodyPos_WF[LegNumber][1];
                FootfallPositionRecord[LegNumber][2] = 0;
                FootfallPositionRecord[LegNumber][3] = ObservationTime;
                WheelAnglePrev[LegNumber] = Message[LegNumber*4 + 3];
                ShankPitchPrev[LegNumber] = body_pitch + Message[LegNumber*4 + 1] + Message[LegNumber*4 + 2];

            }
            else if(FootLanding[LegNumber])
            {
                FootLanding[LegNumber]= false;
                FootfallPositionRecord[LegNumber][0] = StateSpaceModel->EstimatedState[0] + FootBodyPos_WF[LegNumber][0];
                FootfallPositionRecord[LegNumber][1] = StateSpaceModel->EstimatedState[3] + FootBodyPos_WF[LegNumber][1];
                FootfallPositionRecord[LegNumber][2] = StateSpaceModel->EstimatedState[6] + FootBodyPos_WF[LegNumber][2];
                FootfallPositionRecord[LegNumber][3] = ObservationTime;
                WheelAnglePrev[LegNumber] = Message[LegNumber*4 + 3];
                ShankPitchPrev[LegNumber] = body_pitch + Message[LegNumber*4 + 1] + Message[LegNumber*4 + 2];

                static double MapHeightStore[3][1000] = {0};
                static int MapHeightStoreMax = 0;
                int i = 0;
                double Zdifference = 99;
                
                // std::cout << "[LAND] L" << LegNumber
                //     << " t=" << ObservationTime
                //     << " z_in=" << (StateSpaceModel->EstimatedState[6] + Observation[6])
                //     << " scope=" << Environement_Height_Scope
                //     << " fade=" << Data_Fading_Time
                //     << " max=" << MapHeightStoreMax
                //     << std::endl;
            
                for(i = 0; i < (MapHeightStoreMax+1); i++)
                {
                    if(MapHeightStore[2][i] != 0 && std::abs(ObservationTime-MapHeightStore[2][i]) > Data_Fading_Time)
                    {
                        MapHeightStore[0][i] = 0;
                        MapHeightStore[1][i] = 0;
                        MapHeightStore[2][i] = 0;
                        // std::cout <<"One old step cleared" << std::endl;
                }
                }

                for(i = 0; i < (MapHeightStoreMax+1); i++){

                    if(std::abs(MapHeightStore[0][i] - FootfallPositionRecord[LegNumber][2]) <= Environement_Height_Scope)
                    {
                        // std::cout << "[HIT] i=" << i
                        //         << " rec=" << MapHeightStore[0][i]
                        //         << " in="  << FootfallPositionRecord[LegNumber][2]
                        //         << " dz="  << std::abs(MapHeightStore[0][i] - FootfallPositionRecord[LegNumber][2])
                        //         << " conf->" << MapHeightStore[1][i]
                        //         << std::endl;

                        MapHeightStore[1][i] *= exp(- (ObservationTime - MapHeightStore[2][i]) / (10 * Data_Fading_Time));
                        MapHeightStore[1][i] += 1;
                        MapHeightStore[2][i] = ObservationTime;
                        if(std::abs(MapHeightStore[0][i] - FootfallPositionRecord[LegNumber][2]) <= Environement_Height_Scope/10)
                            Zdifference = 0;
                        else
                            Zdifference = FootfallPositionRecord[LegNumber][2] - MapHeightStore[0][i];

                        // std::cout << "[CORR] z_diff=" << Zdifference
                        //     << " z_out=" << (FootfallPositionRecord[LegNumber][2] - Zdifference)
                        //     << std::endl;
                        break;
                    }
                }
                if(Zdifference == 99){
                    Zdifference = 0;
                    for(i = 0; i < (MapHeightStoreMax+1); i++)
                    {
                        if(MapHeightStore[2][i] == 0)
                        {
                            MapHeightStore[0][i] = FootfallPositionRecord[LegNumber][2];
                            MapHeightStore[1][i] = 1;
                            MapHeightStore[2][i] = ObservationTime;
                            // std::cout << "[NEW] i=" << i
                            //     << " h=" << MapHeightStore[0][i]
                            //     << " t=" << MapHeightStore[2][i]
                            //     << " max=" << MapHeightStoreMax
                            //     << std::endl;
                            break;
                        }
                    }
                    if(i >= 999)
                    {
                        // std::cout << "[FULL] store full-ish, overwrite slot0"
                        //     << " t=" << ObservationTime
                        //     << " max=" << MapHeightStoreMax
                        //     << std::endl;

                        for(i = 0; i < (MapHeightStoreMax+1); i++)
                        {
                            if(MapHeightStore[2][i] != 0 && std::abs(ObservationTime-MapHeightStore[2][i]) > 60)
                            {
                                MapHeightStore[0][i] = 0;
                                MapHeightStore[1][i] = 0;
                                MapHeightStore[2][i] = 0;
                            }
                        }
                        i = 0;
                        MapHeightStore[0][i] = FootfallPositionRecord[LegNumber][2];
                        MapHeightStore[1][i] = 1;
                        MapHeightStore[2][i] = ObservationTime;
                    }
                    if(i == MapHeightStoreMax + 1)
                    {
                        MapHeightStoreMax = i;
                        MapHeightStore[0][i] = FootfallPositionRecord[LegNumber][2];
                        MapHeightStore[1][i] = 1;
                        MapHeightStore[2][i] = ObservationTime;
                    }
                    
                } 
                FootfallPositionRecord[LegNumber][2] = FootfallPositionRecord[LegNumber][2] - Zdifference;
            }

             // 轮子转动角度
            double WheelRotationAngle = Message[LegNumber*4 + 3] - WheelAnglePrev[LegNumber];
            if (WheelRotationAngle >  2.0*M_PI) WheelRotationAngle -= 4.0*M_PI;
            if (WheelRotationAngle < -2.0*M_PI) WheelRotationAngle += 4.0*M_PI;
            WheelAnglePrev[LegNumber] = Message[LegNumber*4 + 3];

            // 小腿摆动角
            double ShankPitch = body_pitch + Message[LegNumber*4 + 1] + Message[LegNumber*4 + 2];
            double ShankRotationAngle = ShankPitch - ShankPitchPrev[LegNumber];
            while (WheelRotationAngle >  M_PI) WheelRotationAngle -= 2.0*M_PI;
            while (WheelRotationAngle < -M_PI) WheelRotationAngle += 2.0*M_PI;

            ShankPitchPrev[LegNumber] = ShankPitch;

            // 轮子有效转动角
            const double WheelRotationAngleEff = WheelRotationAngle - ShankRotationAngle;
            
            // 轮子沿地面的实际滚动距离
            const double WheelMove = Par_WheelRadius * WheelRotationAngleEff;

            FootfallPositionRecord[LegNumber][0] += WheelMove * move_dir_x;
            FootfallPositionRecord[LegNumber][1] += WheelMove * move_dir_y;
            FootfallPositionRecord[LegNumber][2] += WheelMove * move_dir_z;

            // 轮对地有效角速度
            const double WheelRotationVelocityEff = Message[16 + LegNumber*4 + 3] - (Message[16 + LegNumber*4 + 1] + Message[16 + LegNumber*4 + 2]); 
            const double WheelVel = Par_WheelRadius * WheelRotationVelocityEff;

            p_sum[0] += FootfallPositionRecord[LegNumber][0] - FootBodyPos_WF[LegNumber][0];
            p_sum[1] += FootfallPositionRecord[LegNumber][1] - FootBodyPos_WF[LegNumber][1];
            p_sum[2] += FootfallPositionRecord[LegNumber][2] - FootBodyPos_WF[LegNumber][2];

            v_sum[0] += FootBodyVel_WF[LegNumber][0] + WheelVel * move_dir_x;
            v_sum[1] += FootBodyVel_WF[LegNumber][1] + WheelVel * move_dir_y;
            v_sum[2] += FootBodyVel_WF[LegNumber][2] + WheelVel * move_dir_z;
            
            leg_cnt++;
        }

        Observation[0] = p_sum[0] / (double)leg_cnt;
        Observation[3] = p_sum[1] / (double)leg_cnt;
        Observation[6] = p_sum[2] / (double)leg_cnt;
        Observation[1] = v_sum[0] / (double)leg_cnt;
        Observation[4] = v_sum[1] / (double)leg_cnt;
        Observation[7] = v_sum[2] / (double)leg_cnt;
    }

    void SensorLegsPos::FeetEffort2Body(int LegNumber)
    {
        // -------- body frame: torque = r x f --------
        const double tau_b[3] = {
            FootBodyPos_BF[LegNumber][1]*FootBodyEff_BF[LegNumber][2] - FootBodyPos_BF[LegNumber][2]*FootBodyEff_BF[LegNumber][1],  // Mx  (roll moment about x)
            FootBodyPos_BF[LegNumber][2]*FootBodyEff_BF[LegNumber][0] - FootBodyPos_BF[LegNumber][0]*FootBodyEff_BF[LegNumber][2],  // My  (pitch moment about y)
            FootBodyPos_BF[LegNumber][0]*FootBodyEff_BF[LegNumber][1] - FootBodyPos_BF[LegNumber][1]*FootBodyEff_BF[LegNumber][0]   // Mz  (yaw moment about z)
        };

        // -------- world frame: rotate both --------
        double tau_w[3];
        quat_rot_vec3(Est_Quaternion, tau_b, tau_w);  // body -> world

        // 3) 写入：XYZ 力 + XYZ 力矩（把它当作 roll/pitch/yaw 的力矩分量）
        FeetEffort2BodyMotion[LegNumber][0] = FootBodyEff_WF[LegNumber][0];
        FeetEffort2BodyMotion[LegNumber][1] = FootBodyEff_WF[LegNumber][1];
        FeetEffort2BodyMotion[LegNumber][2] = FootBodyEff_WF[LegNumber][2];
        FeetEffort2BodyMotion[LegNumber][3] = tau_w[0];
        FeetEffort2BodyMotion[LegNumber][4] = tau_w[1];
        FeetEffort2BodyMotion[LegNumber][5] = tau_w[2];
    }

    void SensorLegsPos::EstimateGroundPitchAlongHeading(double& move_dir_x, double& move_dir_y, double& move_dir_z)
    {
        // 默认输出：平地前进
        move_dir_x = 1.0;
        move_dir_y = 0.0;
        move_dir_z = 0.0;

        // =========================================================
        // 1) 用当前着地足的 FootBodyPos_WF 拟合平面 z = a*x + b*y + c
        // =========================================================
        double sxx = 0.0, sxy = 0.0, syy = 0.0;
        double sx  = 0.0, sy  = 0.0;
        double sxz = 0.0, syz = 0.0, sz  = 0.0;
        int n = 0;

        for (int LegNumber = 0; LegNumber < 4; ++LegNumber)
        {
            if (!FootIsOnGround[LegNumber])
                continue;
                
            if (!FootfallPositionRecordIsInitiated[LegNumber])
                continue;
            
            if (ObservationTime - FootfallPositionRecord[LegNumber][3] < GroundPitchTimeThreshold)
                continue;

            const double x = FootBodyPos_WF[LegNumber][0];
            const double y = FootBodyPos_WF[LegNumber][1];
            const double z = FootBodyPos_WF[LegNumber][2];

            sxx += x * x;
            sxy += x * y;
            syy += y * y;
            sx  += x;
            sy  += y;
            sxz += x * z;
            syz += y * z;
            sz  += z;
            ++n;
        }

        double A[3][3] = {
            { sxx, sxy, sx },
            { sxy, syy, sy },
            { sx , sy , static_cast<double>(n) }
        };
        double rhs[3] = { sxz, syz, sz };
        double Ainv[3][3];
        double abc[3] = {0.0, 0.0, 0.0};

        // =========================================================
        // 2) 计算机身前向在世界系水平面的单位方向 hx, hy
        // =========================================================
        double hx = 1.0 - 2.0 * (Est_Quaternion[2] * Est_Quaternion[2] + Est_Quaternion[3] * Est_Quaternion[3]);
        double hy = 2.0 * (Est_Quaternion[1] * Est_Quaternion[2] + Est_Quaternion[0] * Est_Quaternion[3]);
        double hn = std::sqrt(hx * hx + hy * hy);
        if (hn < 1e-9)
        {
            hx = 1.0;
            hy = 0.0;
            hn = 1.0;
        }
        hx /= hn;
        hy /= hn;

        if (n < 3 || !mat3_inv(A, Ainv))
        {
            move_dir_x = hx;
            move_dir_y = hy;
            move_dir_z = 0.0;
            return;
        }

        mat3_mul_vec(Ainv, rhs, abc);

        const double a = abc[0];
        const double b = abc[1];
        // =========================================================
        // 3) 计算“朝向方向上的坡度”
        //    k = dz / d(水平距离)
        // =========================================================
        const double k = a * hx + b * hy;

        // =========================================================
        // 4) 坡度小：只修正 XY
        // =========================================================
        if (std::fabs(k) <= std::tan(GroundPitchAngleThreshold))
        {
            move_dir_x = hx;
            move_dir_y = hy;
            move_dir_z = 0.0;
            return;
        }

        // =========================================================
        // 5) 坡度大：把轮子滚动分解到 XYZ
        //    hxy_n: 水平分量比例
        //    hz_n : 垂向分量比例
        // =========================================================
        const double hxy_n = 1.0 / std::sqrt(1.0 + k * k);
        const double hz_n  = k * hxy_n;

        move_dir_x = hx * hxy_n;
        move_dir_y = hy * hxy_n;
        move_dir_z = hz_n;
    }

    void SensorLegsOri::SensorDataHandle(double* Message, double Time) 
    {
        if(!JointsRPYEnable)
            return;
        
        double P_body[4][3];
        double P_world[4][3];
        static double TimeRecord = Time;
        int LegNumber, i;

        for (LegNumber = 0; LegNumber < 4; ++LegNumber) {
            for (i = 0; i < 3; i++){
                P_body[LegNumber][i] = legs_pos_ref_->FootBodyPos_BF[LegNumber][i];
                P_world[LegNumber][i] = legs_pos_ref_->FootfallPositionRecord[LegNumber][i];
            }
        }

        int n_ground = 0;
        for (LegNumber = 0; LegNumber < 4; LegNumber++) {
            if (legs_pos_ref_->FootIsOnGround[LegNumber])
                n_ground++;
        }

        if (n_ground < 2) {
            return;
        }
        if (n_ground < 4){
            TimeRecord = Time;
            legori_current_weight = legori_init_weight;
        }
        else{
            legori_current_weight = (Time-TimeRecord) * (1.0 - legori_init_weight) /legori_time_weight + legori_init_weight;
            if(legori_current_weight>1.0)
                legori_current_weight = 1.0;
        }

        const double roll  = StateSpaceModel->EstimatedState[0];
        const double pitch = StateSpaceModel->EstimatedState[3];

        double q_rp[4];
        eulerZYX_to_quat(roll, pitch, 0.0, q_rp);

        double sx = 0.0, sy = 0.0;
        for (i = 0; i < 4; ++i) {
            if(!legs_pos_ref_->FootIsOnGround[i])
                continue;

            for (int j = i + 1; j < 4; ++j) {
                if(!legs_pos_ref_->FootIsOnGround[j])
                    continue;

                double vb_x = P_body[j][0] - P_body[i][0];
                double vb_y = P_body[j][1] - P_body[i][1];
                double vb_z = P_body[j][2] - P_body[i][2];

                double v_body[3] = { vb_x, vb_y, vb_z };
                double v_rp[3];
                quat_rot_vec3(q_rp, v_body, v_rp);


                double vw_x = P_world[j][0] - P_world[i][0];
                double vw_y = P_world[j][1] - P_world[i][1];
                double vw_z = P_world[j][2] - P_world[i][2];

                const double ang_rp = std::atan2(v_rp[1], v_rp[0]);
                const double ang_w  = std::atan2(vw_y,     vw_x);

                const double yaw_ij = angle_wrap(ang_w - ang_rp);
                sx += std::cos(yaw_ij);
                sy += std::sin(yaw_ij);

                int k = i+j;
                if(i==0)
                    k--;

                // printf("%d: ang_rp-%lf; ang_w-%lf; yaw_ij-%lf \n",k,ang_rp,ang_w,yaw_ij);
            }
        }

        if (sx != 0.0 || sy != 0.0) {
            const double yaw_est = std::atan2(sy, sx);
            const double yaw_now = StateSpaceModel->EstimatedState[6];
            const double err     = angle_wrap(yaw_est - yaw_now);
            legori_correct = angle_wrap(yaw_now + legori_current_weight * err);
            UpdateEst_Quaternion();
        }
    }
}