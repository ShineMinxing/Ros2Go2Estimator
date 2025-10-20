#include "Sensor_Legs.h"

namespace DataFusion
{
    void SensorLegsPos::SensorDataHandle(double* Message, double Time) 
    {
        ObservationTime = Time;

        for(int LegNumber = 0; LegNumber<4; LegNumber++)
        {

            for(int i = 0; i < 3; i++)
            {
                Observation[3*i] = Message[LegNumber*3+i];
                Observation[3*i+1] = Message[12+LegNumber*3+i];
            }
            for(int i = 0; i < 3; i++)
            {
                SensorPosition[i] = KinematicParams(LegNumber, i) ;
            }
            LatestFeetEffort = Message[24 + LegNumber];
            
            //Obtain foot hip relative position and velocity
            Joint2HipFoot(LegNumber);

            for(int i = 0; i < 3; i++)
            {
                StateSpaceModel->Double_Par[6 * LegNumber + i] = Observation[3 * i];
                StateSpaceModel->Double_Par[6 * LegNumber + 3 + i] = Observation[3 * i + 1];
            }

            ObservationCorrect_Position();
            ObservationCorrect_Velocity();

            for(int i = 0; i < 3; i++)
            {
                StateSpaceModel->Double_Par[24 + 6 * LegNumber + i] = Observation[3 * i];
                StateSpaceModel->Double_Par[24 + 6 * LegNumber + 3 + i] = Observation[3 * i + 1];
            }

            for(int i = 0; i < StateSpaceModel->Nx * StateSpaceModel->Nz; i++)
            {
                StateSpaceModel->Matrix_H[i] = 0;
            }
            for(int i = 0; i < 3; i++)
            {
                StateSpaceModel->Matrix_H[(3 * i + 0) * StateSpaceModel->Nx + (3 * i + 0)] = 1;
                StateSpaceModel->Matrix_H[(3 * i + 1) * StateSpaceModel->Nx + (3 * i + 1)] = 1;
            }

            if(FootIsOnGround[LegNumber])
            {
                PositionCorrect(LegNumber);
                StateSpaceModel1_EstimatorPort(Observation, ObservationTime, StateSpaceModel);

                for(int i = 0; i < 3; i++){
                    StateSpaceModel->Double_Par[48 + 6 * LegNumber + i] = Observation[3 * i];
                    StateSpaceModel->Double_Par[48 + 6 * LegNumber + 3 + i] = Observation[3 * i + 1];
                }
            }
        }
    }

    void SensorLegsPos::Joint2HipFoot(int LegNumber)
    {
        double s1, s2, s3, c1, c2, c3, c23, s23, dq1, dq2, dq3;
        int SideSign;

        if (LegNumber == 0 || LegNumber == 2)
            SideSign = -1;
        else
            SideSign = 1;

        s1 = sin(Observation[0]);
        s2 = sin(Observation[3]);
        s3 = sin(Observation[6]);
        c1 = cos(Observation[0]);
        c2 = cos(Observation[3]);
        c3 = cos(Observation[6]);
        dq1 = Observation[1];
        dq2 = Observation[4];
        dq3 = Observation[7];
        c23 = c2 * c3 - s2 * s3;
        s23 = s2 * c3 + c2 * s3;

        Observation[0] = (Par_CalfLength + Par_FootLength)  * s23 + Par_ThighLength * s2;
        Observation[3] = Par_HipLength * SideSign * c1 + (Par_CalfLength + Par_FootLength) * (s1 * c23) + Par_ThighLength * c2 * s1;
        Observation[6] = Par_HipLength * SideSign * s1 - (Par_CalfLength + Par_FootLength) * (c1 * c23) - Par_ThighLength * c1 * c2;

        Observation[1] = ((Par_CalfLength + Par_FootLength) *c23 + Par_ThighLength * c2)*dq2 + ((Par_CalfLength + Par_FootLength) *c23)*dq3;
        Observation[4] = ((Par_CalfLength + Par_FootLength) *c1*c23 + Par_ThighLength * c1*c2 - Par_HipLength*SideSign*s1)*dq1\
        + (-(Par_CalfLength + Par_FootLength)  * s1*s23 - Par_ThighLength * s1*s2)*dq2\
        + (-(Par_CalfLength + Par_FootLength)  * s1*s23)*dq3;
        Observation[7] = ((Par_CalfLength + Par_FootLength) *s1*c23 + Par_ThighLength * c2*s1 + Par_HipLength*SideSign*c1)*dq1\
        + ((Par_CalfLength + Par_FootLength) *c1*s23 + Par_ThighLength * c1*s2)*dq2\
        + ((Par_CalfLength + Par_FootLength) *c1*s23)*dq3;

        Observation[0] = -Observation[0];
        Observation[1] = -Observation[1];

        FootBodyPosition[LegNumber][0] = Observation[0] + SensorPosition[0];
        FootBodyPosition[LegNumber][1] = Observation[3] + SensorPosition[1];
        FootBodyPosition[LegNumber][2] = Observation[6] + SensorPosition[2];

        //Detect the moment of foot falling on the ground
        if(LatestFeetEffort >= FootEffortThreshold)
        {
            FootIsOnGround[LegNumber] = true;
        }
        else
        {
            FootIsOnGround[LegNumber] = false;
        }
        if(FootIsOnGround[LegNumber] && !FootWasOnGround[LegNumber])
        {
            FootLanding[LegNumber] = true;
        }
        else
            FootLanding[LegNumber] = false;

        if(LatestFeetEffort>10&&LatestFeetEffort<FootEffortThreshold&&Observation[6]<0&&Observation[6]>-0.12&&Observation[0]>-0.2) //趴着
            FootIsOnGround[LegNumber] = true;

        FootWasOnGround[LegNumber] = FootIsOnGround[LegNumber];

    }

    void SensorLegsPos::PositionCorrect(int LegNumber){

        if(FootLanding[LegNumber]||FootfallPositionRecordIsInitiated[LegNumber]==0)
        {
            FootfallPositionRecordIsInitiated[LegNumber] = 1;
            FootLanding[LegNumber]= 0;
            FootfallPositionRecord[LegNumber][0] = StateSpaceModel->EstimatedState[0] + Observation[0];
            FootfallPositionRecord[LegNumber][1] = StateSpaceModel->EstimatedState[3] + Observation[3];
            FootfallPositionRecord[LegNumber][2] = StateSpaceModel->EstimatedState[6] + Observation[6];

            static double MapHeightStore[3][100] = {0};   
            double Scope = 0.2, DataAvailablePeriod = 60;
            int i = 0;
            double distance = 0, Zdifference = 0, Temp[4] = {0,0,0,0};
            double AngleA = atan(abs(Observation[3]) / abs(Observation[0]));
        
            distance = std::sqrt(std::pow(Observation[0],2) + std::pow(Observation[3],2) + std::pow(Observation[6],2));
        
            // Discard too old record
            for(i = 0; i < 100; i++)
            {
              if(MapHeightStore[2][i] != 0 && abs(ObservationTime-MapHeightStore[2][i]) > DataAvailablePeriod)
              {
                MapHeightStore[0][i] = 0;
                MapHeightStore[1][i] = 0;
                MapHeightStore[2][i] = 0;
                std::cout <<"One old step cleared" << std::endl;
              }
            }
        
            Zdifference = 0;
            // MapHeightStore and claculate z difference
            if(FootfallPositionRecord[LegNumber][2] <= Scope){

              Zdifference = FootfallPositionRecord[LegNumber][2];
            }
            else{

                for(i = 0; i < 100; i++){

                    if(abs(MapHeightStore[0][i] - FootfallPositionRecord[LegNumber][2]) <= Scope)
                    {
                        std::cout <<"Recorded: " << MapHeightStore[0][i];
                        std::cout <<", input: " << FootfallPositionRecord[LegNumber][2] <<" after " << ObservationTime - MapHeightStore[2][i];
                        MapHeightStore[1][i] *= exp(- (ObservationTime - MapHeightStore[2][i]) / (10 * DataAvailablePeriod)); //Confidence fading
                        // MapHeightStore[0][i] = (MapHeightStore[0][i] * MapHeightStore[1][i] + FootfallPositionRecord[LegNumber][2]) / (MapHeightStore[1][i] + 1);
                        MapHeightStore[1][i] += 1;
                        MapHeightStore[2][i] = ObservationTime;
                        Zdifference = FootfallPositionRecord[LegNumber][2] - MapHeightStore[0][i];
                        std::cout <<", height correct to " << MapHeightStore[0][i] << " confidence is " << MapHeightStore[1][i] << std::endl;
                        break;
                    }
                }
                if(Zdifference == 0){

                    for(i = 0; i < 100; i++)
                    {
                        if(MapHeightStore[2][i] == 0)
                        {
                            MapHeightStore[0][i] = FootfallPositionRecord[LegNumber][2];
                            MapHeightStore[1][i] = 1;
                            MapHeightStore[2][i] = ObservationTime;
                            std::cout <<"New height recorded: " << MapHeightStore[0][i] << std::endl;
                            break;
                        }
                    }
                }  
            }
            FootfallPositionRecord[LegNumber][2] = FootfallPositionRecord[LegNumber][2] - Zdifference;
        }

        Observation[0] = FootfallPositionRecord[LegNumber][0] - Observation[0];
        Observation[3] = FootfallPositionRecord[LegNumber][1] - Observation[3];
        Observation[6] = FootfallPositionRecord[LegNumber][2] - Observation[6];
    }

   void SensorLegsOri::SensorDataHandle(double* Message, double Time) 
    {
        ObservationTime = Time;

        // 1) 读取足在身体系/世界系的位置（由 SensorLegsPos 写入 Double_Par）
        double P_body[4][3];   // 身体系足点位置
        double P_world[4][3];  // 世界系足点位置

        for (int leg = 0; leg < 4; ++leg) {
            for (int i = 0; i < 3; i++){
                P_body[leg][i] = legs_pos_ref_->FootBodyPosition[leg][i];
                P_world[leg][i] = legs_pos_ref_->FootfallPositionRecord[leg][i];
            }
        }

        // 2) 统计在地足（用固定数组，不用 <vector>）
        int n_ground = 0;
        for (int leg = 0; leg < 4; leg++) {
            if (legs_pos_ref_->FootIsOnGround[leg])
                n_ground++;
        }

        // 不足两足在地：返回
        if (n_ground < 2) {
            return;
        }

        // 3) 仅保留 roll/pitch（去掉 yaw），把身体系差向量旋到“去 yaw”的世界平面
        const double roll  = StateSpaceModel->EstimatedState[0];
        const double pitch = StateSpaceModel->EstimatedState[3];
        const double yaw   = StateSpaceModel->EstimatedState[6];

        Eigen::Quaterniond Rp =
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll , Eigen::Vector3d::UnitX());

        // 角度归一化
        auto angle_wrap = [](double a) -> double {
            while (a >  M_PI) a -= 2.0 * M_PI;
            while (a < -M_PI) a += 2.0 * M_PI;
            return a;
        };

        for(int i=0; i<100; i++)
            StateSpaceModel->Double_Par[i] = 0;

        // 4) 基于两两配对估计 yaw（固定大小累加，圆统计）
        double sx = 0.0, sy = 0.0; // 累加 cos/sin
        for (int i = 0; i < 4; ++i) {
            if(!legs_pos_ref_->FootIsOnGround[i])
                continue;

            for (int j = i + 1; j < 4; ++j) {
                if(!legs_pos_ref_->FootIsOnGround[j])
                    continue;

                // 身体系差向量
                double vb_x = P_body[j][0] - P_body[i][0];
                double vb_y = P_body[j][1] - P_body[i][1];
                double vb_z = P_body[j][2] - P_body[i][2];

                // 用仅含 roll/pitch 的旋转（去 yaw）
                Eigen::Vector3d v_body(vb_x, vb_y, vb_z);
                Eigen::Vector3d v_rp = Rp * v_body;

                // 世界系真实差向量
                double vw_x = P_world[j][0] - P_world[i][0];
                double vw_y = P_world[j][1] - P_world[i][1];
                double vw_z = P_world[j][2] - P_world[i][2];

                // 平面方位角
                const double ang_rp = std::atan2(v_rp.y(), v_rp.x());
                const double ang_w  = std::atan2(vw_y,     vw_x);

                const double yaw_ij = angle_wrap(ang_w - ang_rp);
                sx += std::cos(yaw_ij);
                sy += std::sin(yaw_ij);

                int k = i+j;
                if(i==0)
                    k--;
                StateSpaceModel->Double_Par[k*8+0] = vb_x;
                StateSpaceModel->Double_Par[k*8+1] = vb_y;
                StateSpaceModel->Double_Par[k*8+2] = vw_x;
                StateSpaceModel->Double_Par[k*8+3] = vw_y;
                StateSpaceModel->Double_Par[k*8+4] = ang_rp;
                StateSpaceModel->Double_Par[k*8+5] = yaw_ij;
                StateSpaceModel->Double_Par[k*8+6] = std::cos(yaw_ij);
                StateSpaceModel->Double_Par[k*8+7] = std::sin(yaw_ij);

            }
        }

        // 5) 与当前估计做指数平滑融合，避免跳变
        if (sx != 0.0 || sy != 0.0) {
            const double yaw_est = std::atan2(sy, sx);
            const double yaw_now = StateSpaceModel->EstimatedState[6];
            const double err     = angle_wrap(yaw_est - yaw_now);
            const double alpha   = 0.001;
            StateSpaceModel->Double_Par[99] = angle_wrap(yaw_now + alpha * err);

            StateSpaceModel->Double_Par[49] = yaw_est;
            StateSpaceModel->Double_Par[50] = yaw_now;
            StateSpaceModel->Double_Par[51] = err;
            StateSpaceModel->Double_Par[52] = StateSpaceModel->EstimatedState[6];
        }
    }
}