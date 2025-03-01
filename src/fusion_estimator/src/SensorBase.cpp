#include "SensorBase.h"

using namespace Eigen;

namespace DataFusion
{
    Eigen::MatrixXd Est_StateXYZ;         // Estimated States [Xposition Xvelocity Xacceleration Yp Yv Ya Zp Zv Za]
    Eigen::MatrixXd Est_StateRPY;         // Estimated States [ROLLorientation ROLLvelocity ROLLacceleration Po Pv Pa Yo Yv Ya]
    Eigen::Quaterniond Est_Quaternion;    // Body Orientation Quaternion
    Eigen::Quaterniond Est_QuaternionInv; // Body Orientation Quaternion Inverse


    void Sensors::UpdateEst_Quaternion(Eigen::MatrixXd* StateRPY){

        Est_Quaternion = Eigen::AngleAxisd((*StateRPY)(0, 0), Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd((*StateRPY)(3, 0), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd((*StateRPY)(6, 0), Eigen::Vector3d::UnitZ());
        Est_QuaternionInv = Est_Quaternion.inverse();

    }

    void Sensors::ObservationCorrect_Acceleration(double* Observation){

        //Data Orientation Adjust
        Est_QuaternionTemp1 = Eigen::Quaterniond(0, Observation[2], Observation[5], Observation[8]);

        //Sensor Quaternion in Body Frame
        Est_QuaternionTemp1 = SensorQuaternion * Est_QuaternionTemp1 * SensorQuaternionInv;

        //Sensor acceleration in World Frame
        Est_QuaternionTemp1 = Est_Quaternion * Est_QuaternionTemp1 * Est_QuaternionInv;

        //1. Calculate Centrifugal Acceleration
        Est_SensorPosition << SensorPosition[0], SensorPosition[1], SensorPosition[2];
        Est_Vector3dTemp1 << Est_StateRPY(1, 0), Est_StateRPY(4, 0), Est_StateRPY(7, 0);
        Est_Vector3dTemp2 = Est_Vector3dTemp1.cross(Est_SensorPosition);
        Est_Vector3dTemp1 = Est_Vector3dTemp1.cross(Est_Vector3dTemp2);

        //3. Calculate Body Acceleration with Centrifugal Acceleration Compensation
        Est_QuaternionTemp1.x() -= Est_Vector3dTemp1(0);
        Est_QuaternionTemp1.y() -= Est_Vector3dTemp1(1);
        Est_QuaternionTemp1.z() -= Est_Vector3dTemp1(2);

        Observation[2] = Est_QuaternionTemp1.x();
        Observation[5] = Est_QuaternionTemp1.y();
        Observation[8] = Est_QuaternionTemp1.z();

    }
    void Sensors::ObservationCorrect_Orientation(double* Observation){

        double Q[4];
        //Obtain Sensor Quaternion in World Frame
        Est_QuaternionTemp1 = Eigen::AngleAxisd(Observation[6], Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(Observation[3], Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(Observation[0], Eigen::Vector3d::UnitX());
        Est_QuaternionTemp2 = Est_Quaternion.inverse();

        //Obtain Body Quaternion in World Frame
        Est_QuaternionTemp2 = Est_QuaternionTemp1 * SensorQuaternionInv;

        //Obtain Body Orientation in World Frame
        Q[0] = Est_QuaternionTemp2.w();
        Q[1] = Est_QuaternionTemp2.x();
        Q[2] = Est_QuaternionTemp2.y();
        Q[3] = Est_QuaternionTemp2.z();

        Observation[0] = atan2(2 * (Q[0] * Q[1] + Q[2] * Q[3]), (1 - 2 * (Q[1] * Q[1] + Q[2] * Q[2])));
        Observation[3] = asin(2 * (Q[0] * Q[2] - Q[3] * Q[1]));
        Observation[6] = atan2(2 * (Q[0] * Q[3] + Q[1] * Q[2]), (1 - 2 * (Q[2] * Q[2] + Q[3] * Q[3])));

    }

    void Sensors::ObservationCorrect_AngularVelocity(double* Observation){

        //Data Orientation Adjust
        Est_QuaternionTemp1 = Eigen::Quaterniond(0, Observation[1], Observation[4], Observation[7]);

        //Sensor Quaternion in Body Frame
        Est_QuaternionTemp1 = SensorQuaternion * Est_QuaternionTemp1 * SensorQuaternionInv;

        //Sensor Quaternion in World Frame
        Est_QuaternionTemp1 = Est_Quaternion * Est_QuaternionTemp1 * Est_QuaternionInv;

        Observation[1] = Est_QuaternionTemp1.x();
        Observation[4] = Est_QuaternionTemp1.y();
        Observation[7] = Est_QuaternionTemp1.z();

    }
}
