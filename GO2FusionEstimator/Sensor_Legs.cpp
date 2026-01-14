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
                SensorPosition[i] = KinematicParams(LegNumber, i) ;
            }
            
            Joint2HipFoot(Message,LegNumber);

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
                StateSpaceModel_Go2_EstimatorPort(Observation, ObservationTime, StateSpaceModel);

                for(int i = 0; i < 3; i++){
                    StateSpaceModel->Double_Par[48 + 6 * LegNumber + i] = Observation[3 * i];
                    StateSpaceModel->Double_Par[48 + 6 * LegNumber + 3 + i] = Observation[3 * i + 1];
                }
            }
        }
    }

    void SensorLegsPos::Joint2HipFoot(double *Message, int LegNumber)
    {
        double s1, s2, s3, c1, c2, c3, c23, s23, dq1, dq2, dq3;
        int SideSign;

        if (LegNumber == 0 || LegNumber == 2)
            SideSign = 1;
        else
            SideSign = -1;


        s1 = sin(Message[LegNumber*3+0]);
        s2 = sin(Message[LegNumber*3+1]);
        s3 = sin(Message[LegNumber*3+2]);
        c1 = cos(Message[LegNumber*3+0]);
        c2 = cos(Message[LegNumber*3+1]);
        c3 = cos(Message[LegNumber*3+2]);
        dq1 = Message[12 + LegNumber*3+0];
        dq2 = Message[12 + LegNumber*3+1];
        dq3 = Message[12 + LegNumber*3+2];

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


        double tau_hip   = Message[24 + LegNumber * 3 + 0];
        double tau_thigh = Message[24 + LegNumber * 3 + 1];
        double tau_knee  = Message[24 + LegNumber * 3 + 2];

        Eigen::Matrix3d J;

        double Jx1_raw = 0.0;
        double Jx2_raw = (Par_CalfLength + Par_FootLength) * c23 + Par_ThighLength * c2;
        double Jx3_raw = (Par_CalfLength + Par_FootLength) * c23;

        double Jy1 = (Par_CalfLength + Par_FootLength) * c1 * c23 + Par_ThighLength * c1 * c2 - Par_HipLength * SideSign * s1;
        double Jy2 = -(Par_CalfLength + Par_FootLength) * s1 * s23 - Par_ThighLength * s1 * s2;
        double Jy3 = -(Par_CalfLength + Par_FootLength) * s1 * s23;

        double Jz1 = (Par_CalfLength + Par_FootLength) * s1 * c23 + Par_ThighLength * c2 * s1 + Par_HipLength * SideSign * c1;
        double Jz2 = (Par_CalfLength + Par_FootLength) * c1 * s23 + Par_ThighLength * c1 * s2;
        double Jz3 = (Par_CalfLength + Par_FootLength) * c1 * s23;

        J(0,0) = -Jx1_raw;
        J(0,1) = -Jx2_raw;
        J(0,2) = -Jx3_raw;

        J(1,0) = Jy1;  J(1,1) = Jy2;  J(1,2) = Jy3;
        J(2,0) = Jz1;  J(2,1) = Jz2;  J(2,2) = Jz3;

        Eigen::Vector3d tau_vec(tau_hip, tau_thigh, tau_knee);

        Eigen::Matrix3d M = J * J.transpose();
        Eigen::Vector3d f_vec = M.inverse() * (J * tau_vec);

        if (M.determinant() > 1e-6)
        {
            LatestFeetEffort = std::fabs(f_vec(2));
        }
        else
        {
            LatestFeetEffort = std::sqrt(
                tau_hip   * tau_hip +
                tau_thigh * tau_thigh +
                tau_knee  * tau_knee
            );
        }
        StateSpaceModel->Double_Par[84 + 3* LegNumber + 0] = f_vec(0);
        StateSpaceModel->Double_Par[84 + 3* LegNumber + 1] = f_vec(1);
        StateSpaceModel->Double_Par[84 + 3* LegNumber + 2] = f_vec(2);

        LatestFeetEffort = Message[24 + LegNumber * 3 + 2];

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

        // if(LatestFeetEffort>10&&LatestFeetEffort<FootEffortThreshold&&Observation[6]<0&&Observation[6]>-0.12&&Observation[0]>-0.2)
        //     FootIsOnGround[LegNumber] = true;

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

            static double MapHeightStore[3][999] = {0};   
            int i = 0;
            double distance = 0, Zdifference = 0, Temp[4] = {0,0,0,0};
            double AngleA = atan(abs(Observation[3]) / abs(Observation[0]));
        
            distance = std::sqrt(std::pow(Observation[0],2) + std::pow(Observation[3],2) + std::pow(Observation[6],2));
        
            for(i = 0; i < 999; i++)
            {
              if(MapHeightStore[2][i] != 0 && abs(ObservationTime-MapHeightStore[2][i]) > Data_Fading_Time)
              {
                MapHeightStore[0][i] = 0;
                MapHeightStore[1][i] = 0;
                MapHeightStore[2][i] = 0;
                std::cout <<"One old step cleared" << std::endl;
              }
            }
        
            Zdifference = 0;
            if(FootfallPositionRecord[LegNumber][2] <= Environement_Height_Scope){

              Zdifference = FootfallPositionRecord[LegNumber][2];
            }
            else{

                for(i = 0; i < 999; i++){

                    if(abs(MapHeightStore[0][i] - FootfallPositionRecord[LegNumber][2]) <= Environement_Height_Scope)
                    {
                        std::cout <<"Recorded: " << MapHeightStore[0][i];
                        std::cout <<", input: " << FootfallPositionRecord[LegNumber][2] <<" after " << ObservationTime - MapHeightStore[2][i];
                        MapHeightStore[1][i] *= exp(- (ObservationTime - MapHeightStore[2][i]) / (10 * Data_Fading_Time));
                        MapHeightStore[1][i] += 1;
                        MapHeightStore[2][i] = ObservationTime;
                        Zdifference = FootfallPositionRecord[LegNumber][2] - MapHeightStore[0][i];
                        std::cout <<", height correct to " << MapHeightStore[0][i] << " confidence is " << MapHeightStore[1][i] << std::endl;
                        break;
                    }
                }
                if(Zdifference == 0){

                    for(i = 0; i < 999; i++)
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
        double P_body[4][3];
        double P_world[4][3];
        static double TimeRecord = Time;

        for (int leg = 0; leg < 4; ++leg) {
            for (int i = 0; i < 3; i++){
                P_body[leg][i] = legs_pos_ref_->FootBodyPosition[leg][i];
                P_world[leg][i] = legs_pos_ref_->FootfallPositionRecord[leg][i];
            }
        }

        int n_ground = 0;
        for (int leg = 0; leg < 4; leg++) {
            if (legs_pos_ref_->FootIsOnGround[leg])
                n_ground++;
        }

        if (n_ground < 2) {
            return;
        }
        if (n_ground < 4){
            TimeRecord = Time;
            StateSpaceModel->Double_Par[98] = legori_init_weight;
        }
        else{
            StateSpaceModel->Double_Par[98] = (Time-TimeRecord) * (1.0 - legori_init_weight) /legori_time_weight + legori_init_weight;
            if(StateSpaceModel->Double_Par[98]>1.0)
                StateSpaceModel->Double_Par[98] = 1.0;
        }

        const double roll  = StateSpaceModel->EstimatedState[0];
        const double pitch = StateSpaceModel->EstimatedState[3];
        const double yaw   = StateSpaceModel->EstimatedState[6];

        Eigen::Quaterniond Rp =
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll , Eigen::Vector3d::UnitX());

        auto angle_wrap = [](double a) -> double {
            while (a >  M_PI) a -= 2.0 * M_PI;
            while (a < -M_PI) a += 2.0 * M_PI;
            return a;
        };

        double sx = 0.0, sy = 0.0;
        for (int i = 0; i < 4; ++i) {
            if(!legs_pos_ref_->FootIsOnGround[i])
                continue;

            for (int j = i + 1; j < 4; ++j) {
                if(!legs_pos_ref_->FootIsOnGround[j])
                    continue;

                double vb_x = P_body[j][0] - P_body[i][0];
                double vb_y = P_body[j][1] - P_body[i][1];
                double vb_z = P_body[j][2] - P_body[i][2];

                Eigen::Vector3d v_body(vb_x, vb_y, vb_z);
                Eigen::Vector3d v_rp = Rp * v_body;

                double vw_x = P_world[j][0] - P_world[i][0];
                double vw_y = P_world[j][1] - P_world[i][1];
                double vw_z = P_world[j][2] - P_world[i][2];

                const double ang_rp = std::atan2(v_rp.y(), v_rp.x());
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
            StateSpaceModel->Double_Par[99] = angle_wrap(yaw_now + StateSpaceModel->Double_Par[98] * err);
            UpdateEst_Quaternion();
        }
    }
}