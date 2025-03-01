#include "Sensor_Legs.h"

namespace DataFusion
{
    void SensorLegs::SensorDataHandle(const unitree_go::msg::dds_::LowState_& low_state)
    {
    }

    void SensorLegs::ObtainUrdfParameter()
    {
        std::string urdf_path = "/home/smx/unitree_ros2_250221/unitree_sdk2_ws/src/fusion_estimator/cfg/go2_description.urdf";
        std::string FLHipJoint_name = "FL_hip_joint";

        std::ifstream urdf_file(urdf_path);
        if (!urdf_file.is_open()) {
            std::cout << "无法打开文件: " << urdf_path << "，使用默认值。" << std::endl;
            return;
        }

        // 读入文件内容
        std::string urdf_xml((std::istreambuf_iterator<char>(urdf_file)), std::istreambuf_iterator<char>());
        urdf_file.close();
        urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_xml);
        if (!model) {
            std::cout << "解析URDF失败: " << urdf_path << "，使用默认值。" << std::endl;
            return;
        }

        // 定义关节名称与对应参数变量的映射
        struct JointParam {
            std::string joint_name;
            Eigen::Vector3d* param;
        };
        std::vector<JointParam> jointParams = {
            {"FL_hip_joint",   &KinematicPar_FLHipJoint},
            {"FL_thigh_joint", &KinematicPar_FLThighJoint},
            {"FL_calf_joint",  &KinematicPar_FLCalfJoint},
            {"FL_foot_joint",  &KinematicPar_FLFootJoint}
        };
    
        for (const auto& jp : jointParams) {
            urdf::JointConstSharedPtr joint = model->getJoint(jp.joint_name);
            if (!joint) {
                std::cout << "未找到关节: " << jp.joint_name << "，使用默认值。" << std::endl;
                std::cout << "KinematicPar for " << jp.joint_name << ": " << jp.param->transpose() << std::endl;
            }else            {
                urdf::Vector3 pos = joint->parent_to_joint_origin_transform.position;
                *(jp.param) << pos.x, pos.y, pos.z;
                std::cout << "Obtained KinematicPar for " << jp.joint_name << ": " << jp.param->transpose() << std::endl;
            }
        }
    
        std::string footLinkName = "FL_foot";
        urdf::LinkConstSharedPtr footLink = model->getLink(footLinkName);
        if (!footLink) {
            std::cout << "未找到连杆: " << footLinkName << "，使用默认值。" << std::endl;
            std::cout << "KinematicPar for " << footLinkName << ": " << KinematicPar__FLFoot << std::endl;
        } else {
            if (footLink->collision && footLink->collision->geometry &&
                footLink->collision->geometry->type == urdf::Geometry::SPHERE) {
                // 动态转换为 Sphere 类型
                urdf::Sphere* sphere = dynamic_cast<urdf::Sphere*>(footLink->collision->geometry.get());
                if (sphere){
                    KinematicPar__FLFoot = sphere->radius;
                    std::cout << "Obtained KinematicPar for " << footLinkName << ": " << KinematicPar__FLFoot << std::endl;
                }
            }
        }
    }
}