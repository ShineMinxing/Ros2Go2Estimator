#include "Sensor_Legs.h"

namespace DataFusion
{
    void SensorLegs::SensorDataHandle(const unitree_go::msg::dds_::LowState_& low_state)
    {

        rclcpp::Clock ros_clock(RCL_SYSTEM_TIME);
        CurrentTime = ros_clock.now();
        CurrentTimestamp = CurrentTime.seconds();

        for(int LegNumber = 0; LegNumber<4; LegNumber++)
        {
            for(int i = 0; i < 3; i++)
            {
                Observation[3*i] = low_state.motor_state()[LegNumber*3+i].q();
                Observation[3*i+1] = low_state.motor_state()[LegNumber*3+i].dq();
            }

            Joint2HipFoot(LegNumber, Observation);

            for(int i = 0; i < 9; i++)
            {
                StateSpaceModel->Double_Par[9*LegNumber+i] = Observation[i];
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
        }
    }

    void SensorLegs::ObtainUrdfParameter()
    {

        KinematicParams << 
        0.1934,  0.0465,  0.000,   0.0,  0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022,
        0.1934, -0.0465,  0.000,   0.0, -0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022,
        -0.1934,  0.0465,  0.000,   0.0,  0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022,
        -0.1934, -0.0465,  0.000,   0.0, -0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022;

        std::string urdf_path = "/home/smx/unitree_ros2_250221/unitree_sdk2_ws/src/fusion_estimator/cfg/go2_description.urdf";
        std::ifstream urdf_file(urdf_path);
        if (!urdf_file.is_open())
        {
            std::cout << "无法打开文件: " << urdf_path << "，使用默认值。" << std::endl;
            return;
        }

        // 读入文件内容
        std::string urdf_xml((std::istreambuf_iterator<char>(urdf_file)), std::istreambuf_iterator<char>());
        urdf_file.close();

        urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_xml);
        if (!model)
        {
            std::cout << "解析URDF失败: " << urdf_path << "，使用默认值。" << std::endl;
            return;
        }

        // 设置输出格式：固定小数点，保留四位小数
        std::cout << std::fixed << std::setprecision(4);

        // 定义腿名称顺序，与 KinematicParams 的行对应
        std::vector<std::string> legs = {"FL", "FR", "RL", "RR"};

        // 定义关节映射结构，每个关节在 13 维向量中的起始列号（每个关节占 3 列）
        struct JointMapping {
            std::string suffix; // 关节后缀，如 "hip_joint"
            int col;            // 起始列号
        };

        // 对每条腿，映射 hip, thigh, calf, foot_joint 对应的参数
        std::vector<JointMapping> jointMappings = {
            { "hip_joint",   0 },
            { "thigh_joint", 3 },
            { "calf_joint",  6 },
            { "foot_joint",  9 }
        };

        // 对每条腿更新各关节参数
        for (size_t i = 0; i < legs.size(); i++)
        {
            const std::string& leg = legs[i];
            for (const auto& jm : jointMappings)
            {
                // 拼接完整关节名称，如 "FL_hip_joint"
                std::string jointName = leg + "_" + jm.suffix;
                urdf::JointConstSharedPtr joint = model->getJoint(jointName);
                if (!joint)
                {
                    std::cout << "未找到关节: " << jointName << " (" << leg << ")，使用默认值: ";
                    std::cout << KinematicParams.row(i).segment(jm.col, 3) << std::endl;
                }
                else
                {
                    urdf::Vector3 pos = joint->parent_to_joint_origin_transform.position;
                    KinematicParams(i, jm.col)     = pos.x;
                    KinematicParams(i, jm.col + 1) = pos.y;
                    KinematicParams(i, jm.col + 2) = pos.z;
                    std::cout << "Obtained KinematicPar for " << jointName << ": ";
                    std::cout << KinematicParams.row(i).segment(jm.col, 3) << std::endl;
                }
            }

            // 更新该腿 foot 连杆 collision 的球半径（存储在列 12）
            std::string footLinkName = leg + "_foot";
            urdf::LinkConstSharedPtr footLink = model->getLink(footLinkName);
            if (!footLink)
            {
                std::cout << "未找到连杆: " << footLinkName << " (" << leg << ")，使用默认值: " << KinematicParams(i, 12) << std::endl;
            }
            else
            {
                if (footLink->collision && footLink->collision->geometry &&
                    footLink->collision->geometry->type == urdf::Geometry::SPHERE)
                {
                    urdf::Sphere* sphere = dynamic_cast<urdf::Sphere*>(footLink->collision->geometry.get());
                    if (sphere)
                    {
                        KinematicParams(i, 12) = sphere->radius;
                        std::cout << "Obtained KinematicPar for " << footLinkName << ": " << KinematicParams(i, 12) << std::endl;
                    }
                }
            }

            Par_HipLength = std::sqrt(KinematicParams(0, 0)*KinematicParams(0, 0) + KinematicParams(0, 1)*KinematicParams(0, 1) + KinematicParams(0, 2)*KinematicParams(0, 2));
            Par_ThighLength = std::sqrt(KinematicParams(0, 3)*KinematicParams(0, 3) + KinematicParams(0, 4)*KinematicParams(0, 4) + KinematicParams(0, 5)*KinematicParams(0, 5));
            Par_CalfLength = std::sqrt(KinematicParams(0, 6)*KinematicParams(0, 6) + KinematicParams(0, 7)*KinematicParams(0, 7) + KinematicParams(0, 8)*KinematicParams(0, 8));
            Par_FootLength = abs(KinematicParams(0, 9));
        }
    }

    void SensorLegs::Joint2HipFoot(int LegNumber, double *Observation)
    {
        double s1, s2, s3, c1, c2, c3, c23, s23, dq1, dq2, dq3;
        int SideSign;

        if (LegNumber == 1 || LegNumber == 3)
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
    }
}