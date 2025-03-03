#include "Sensor_Legs.h"

namespace DataFusion
{
    void SensorLegs::SensorDataHandle(double* Message, double Time) 
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
            }

            for(int i = 0; i < 3; i++){
                StateSpaceModel->Double_Par[24 + 6 * LegNumber + i] = Observation[3 * i];
                StateSpaceModel->Double_Par[24 + 6 * LegNumber + 3 + i] = Observation[3 * i + 1];
            }
        }
    }

    void SensorLegs::Joint2HipFoot(int LegNumber)
    {
        double s1, s2, s3, c1, c2, c3, c23, s23, dq1, dq2, dq3;
        int SideSign;

        if (LegNumber == 1 || LegNumber == 2)
            SideSign = 1;
        else
            SideSign = -1;

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
            std::cout << FootLanding[LegNumber] <<" leg" << LegNumber << std::endl;
        }
        else
            FootLanding[LegNumber] = false;

        if(LatestFeetEffort>10&&LatestFeetEffort<FootEffortThreshold&&Observation[6]<0&&Observation[6]>-0.12) //趴着
            FootIsOnGround[LegNumber] = true;

        FootWasOnGround[LegNumber] = FootIsOnGround[LegNumber];

    }

    void SensorLegs::PositionCorrect(int LegNumber){

        if(FootLanding[LegNumber]||!FootfallPositionRecordIsInitiated)
        {
            FootfallPositionRecordIsInitiated = 1;
            FootLanding[LegNumber]= 0;
            FootfallPositionRecord[LegNumber][0] = StateSpaceModel->EstimatedState[0] + Observation[0];
            FootfallPositionRecord[LegNumber][1] = StateSpaceModel->EstimatedState[3] + Observation[3];
            FootfallPositionRecord[LegNumber][2] = StateSpaceModel->EstimatedState[6] + Observation[6];

            static double MapHeightStore[3][100] = {0};   
            double Scope = 0.1, DataAvailablePeriod = 60;
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
                        MapHeightStore[1][i] *= exp(- (ObservationTime - MapHeightStore[2][i]) / (DataAvailablePeriod / 3)); //Confidence fading
                        MapHeightStore[0][i] = (MapHeightStore[0][i] * MapHeightStore[1][i] + FootfallPositionRecord[LegNumber][2]) / (MapHeightStore[1][i] + 1);
                        MapHeightStore[1][i] += 1;
                        MapHeightStore[2][i] = ObservationTime;
                        Zdifference = FootfallPositionRecord[LegNumber][2] - MapHeightStore[0][i];
                        std::cout <<"New height confidence is " << MapHeightStore[1][i] << std::endl;
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
                            break;
                        }
                    }
                }  
            }
        }

        Observation[0] = FootfallPositionRecord[LegNumber][0] - Observation[0];
        Observation[3] = FootfallPositionRecord[LegNumber][1] - Observation[3];
        Observation[6] = FootfallPositionRecord[LegNumber][2] - Observation[6];
    }
}