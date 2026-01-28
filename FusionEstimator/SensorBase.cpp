#include "SensorBase.h"

namespace DataFusion
{
    double Est_Quaternion[4]    = {1,0,0,0};
    double Est_QuaternionInv[4] = {1,0,0,0};

    void Sensors::UpdateEst_Quaternion(){
        eulerZYX_to_quat(StateSpaceModel->EstimatedState[0],
                        StateSpaceModel->EstimatedState[3],
                        StateSpaceModel->EstimatedState[6],
                        Est_Quaternion);
        quat_conj(Est_Quaternion, Est_QuaternionInv);
    }

    void Sensors::ObservationCorrect_Position(){

        //Data Quaternion Adjust
        Est_Vector3dTemp1[0] = Observation[0];
        Est_Vector3dTemp1[1] = Observation[3];
        Est_Vector3dTemp1[2] = Observation[6];

        //Data Quaternion in Body Frame with Sensor Quaternion Modify
        quat_rot_vec3(SensorQuaternion, Est_Vector3dTemp1, Est_Vector3dTemp1);

        //Data Quaternion in World Frame with Body Quaternion Modify
        quat_rot_vec3(Est_Quaternion, Est_Vector3dTemp1, Est_Vector3dTemp1);

        //1. Calculate Sensor Position Compared to Body in World Frame
        Est_Vector3dTemp2[0] = SensorPosition[0];
        Est_Vector3dTemp2[1] = SensorPosition[1];
        Est_Vector3dTemp2[2] = SensorPosition[2];
        quat_rot_vec3(Est_Quaternion, Est_Vector3dTemp2, Est_Vector3dTemp2);

        //2. Calculate Foot Position Compared to Body in World Frame
        Observation[0] = Est_Vector3dTemp1[0] + Est_Vector3dTemp2[0];
        Observation[3] = Est_Vector3dTemp1[1] + Est_Vector3dTemp2[1];
        Observation[6] = Est_Vector3dTemp1[2] + Est_Vector3dTemp2[2];
    }

    void Sensors::ObservationCorrect_Velocity(){

        //Data Orientation Adjust
        Est_Vector3dTemp1[0] = Observation[1];
        Est_Vector3dTemp1[1] = Observation[4];
        Est_Vector3dTemp1[2] = Observation[7];

        //Sensor Quaternion in Body Frame
        quat_rot_vec3(SensorQuaternion, Est_Vector3dTemp1, Est_Vector3dTemp1);

        //Adjust with Sensor Angular Velocity in World Frame
        quat_rot_vec3(Est_Quaternion, Est_Vector3dTemp1, Est_Vector3dTemp1);

        //1. Calculate Sensor to Body Distance in World Frame
        Est_SensorWorldPosition[0] = SensorPosition[0];
        Est_SensorWorldPosition[1] = SensorPosition[1];
        Est_SensorWorldPosition[2] = SensorPosition[2];
        quat_rot_vec3(Est_Quaternion, Est_SensorWorldPosition, Est_SensorWorldPosition);

        //2. Calculate Sensor Angular Velocity in World Frame
        Est_BodyAngleVel[0] = Observation[1];
        Est_BodyAngleVel[1] = Observation[4];
        Est_BodyAngleVel[2] = Observation[7];
        cross3(Est_BodyAngleVel, Est_SensorWorldPosition, Est_SensorWorldVelocity);


        //3. Calculate Body Velocity with  Angular Velocity  Compensation
        Est_Vector3dTemp1[0] -= Est_SensorWorldVelocity[0];
        Est_Vector3dTemp1[1] -= Est_SensorWorldVelocity[1];
        Est_Vector3dTemp1[2] -= Est_SensorWorldVelocity[2];

        Observation[1] = -Est_Vector3dTemp1[0];
        Observation[4] = -Est_Vector3dTemp1[1];
        Observation[7] = -Est_Vector3dTemp1[2];
    }

    void Sensors::ObservationCorrect_Acceleration(){

        //Data Orientation Adjust
        Est_Vector3dTemp1[0] = Observation[2];
        Est_Vector3dTemp1[1] = Observation[5];
        Est_Vector3dTemp1[2] = Observation[8];

        //Sensor Quaternion in Body Frame
        quat_rot_vec3(SensorQuaternion, Est_Vector3dTemp1, Est_Vector3dTemp1);

        //Sensor acceleration in World Frame
        quat_rot_vec3(Est_Quaternion, Est_Vector3dTemp1, Est_Vector3dTemp1);

        //1. Calculate Centrifugal Acceleration
        Est_SensorPosition[0] = SensorPosition[0];
        Est_SensorPosition[1] = SensorPosition[1];
        Est_SensorPosition[2] = SensorPosition[2];
        Est_Vector3dTemp2[0] = Observation[1];
        Est_Vector3dTemp2[1] = Observation[4];
        Est_Vector3dTemp2[2] = Observation[7];
        cross3(Est_Vector3dTemp2, Est_SensorPosition, Est_BodyAngleVel);      // reuse Est_BodyAngleVel as temp = w×r
        cross3(Est_Vector3dTemp2, Est_BodyAngleVel, Est_BodyAngleVel);        // now Est_BodyAngleVel = w×(w×r)


        //3. Calculate Body Acceleration with Centrifugal Acceleration Compensation
        Est_Vector3dTemp1[0] -= Est_BodyAngleVel[0];
        Est_Vector3dTemp1[1] -= Est_BodyAngleVel[1];
        Est_Vector3dTemp1[2] -= Est_BodyAngleVel[2];

        Observation[2] = Est_Vector3dTemp1[0];
        Observation[5] = Est_Vector3dTemp1[1];
        Observation[8] = Est_Vector3dTemp1[2];
    }
    
    void Sensors::ObservationCorrect_Orientation(){

        //Obtain Sensor Quaternion in World Frame
        eulerZYX_to_quat(Observation[0], Observation[3], Observation[6], Est_QuaternionTemp1);

        //Obtain Body Quaternion in World Frame
        quat_mul(Est_QuaternionTemp1, SensorQuaternionInv, Est_QuaternionTemp2);
        quat_normalize(Est_QuaternionTemp2);

        //Obtain Body Orientation in World Frame
        const double qw = Est_QuaternionTemp2[0];
        const double qx = Est_QuaternionTemp2[1];
        const double qy = Est_QuaternionTemp2[2];
        const double qz = Est_QuaternionTemp2[3];

        Observation[0] = std::atan2(2.0*(qw*qx + qy*qz), 1.0 - 2.0*(qx*qx + qy*qy));
        double s = 2.0*(qw*qy - qz*qx);
        if (s >  1.0) s =  1.0;
        if (s < -1.0) s = -1.0;
        Observation[3] = std::asin(s);
        Observation[6] = std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
    }

    void Sensors::ObservationCorrect_AngularVelocity(){

        //Data Orientation Adjust
        Est_Vector3dTemp1[0] = Observation[1];
        Est_Vector3dTemp1[1] = Observation[4];
        Est_Vector3dTemp1[2] = Observation[7];

        //Sensor Quaternion in Body Frame
        quat_rot_vec3(SensorQuaternion, Est_Vector3dTemp1, Est_Vector3dTemp1);

        //Sensor Quaternion in World Frame
        quat_rot_vec3(Est_Quaternion,   Est_Vector3dTemp1, Est_Vector3dTemp1);

        Observation[1] = Est_Vector3dTemp1[0];
        Observation[4] = Est_Vector3dTemp1[1];
        Observation[7] = Est_Vector3dTemp1[2];
    }

    void Sensors::ObservationCorrect_AngularAcceleration(){

        //Data Orientation Adjust
        Est_Vector3dTemp1[0] = Observation[2];
        Est_Vector3dTemp1[1] = Observation[5];
        Est_Vector3dTemp1[2] = Observation[8];
  
        //Sensor Quaternion in Body Frame
        quat_rot_vec3(SensorQuaternion, Est_Vector3dTemp1, Est_Vector3dTemp1);
  
        //Sensor Quaternion in World Frame
        quat_rot_vec3(Est_Quaternion,   Est_Vector3dTemp1, Est_Vector3dTemp1);
  
        Observation[2] = Est_Vector3dTemp1[0];
        Observation[5] = Est_Vector3dTemp1[1];
        Observation[8] = Est_Vector3dTemp1[2];
    }
}
