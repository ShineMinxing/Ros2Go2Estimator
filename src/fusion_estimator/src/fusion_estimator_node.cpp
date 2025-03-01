#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/idl/go2/LowState_.hpp"
#include "Estimator/EstimatorPortN.h"
#include "fusion_estimator/msg/fusion_estimator_test.hpp" 
#include "Sensor_Legs.h" 
#include "Sensor_IMU.h" 
#include "unitree/idl/go2/LowState_.hpp"
#include <urdf_parser/urdf_parser.h>

using namespace DataFusion;

class FusionEstimatorNode : public rclcpp::Node
{
public:
    FusionEstimatorNode() 
    : Node("fusion_estimator_node")
    {
        // 通过参数获取网络接口名称，设置默认值为 "enxc8a3627ff10b" 或其他有效接口
        this->declare_parameter<std::string>("network_interface", "enxc8a3627ff10b");
        std::string network_interface = this->get_parameter("network_interface").as_string();

        // 初始化Unitree通道工厂
        unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

        // 初始化消息接收与发布
        lowstate_subscriber_.reset(
            new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>("rt/lowstate"));
        lowstate_subscriber_->InitChannel(
            std::bind(&FusionEstimatorNode::LowStateCallback, this, std::placeholders::_1), 1);

        publisher_ = this->create_publisher<fusion_estimator::msg::FusionEstimatorTest>(
            "fusion_estimator_data", 10);

        for(int i = 0; i < 2; i++)
        {
            EstimatorPortN* StateSpaceModel1_SensorsPtrs = new EstimatorPortN; // 创建新的结构体实例
            StateSpaceModel1_Initialization(StateSpaceModel1_SensorsPtrs);     // 调用初始化函数
            StateSpaceModel1_Sensors.push_back(StateSpaceModel1_SensorsPtrs);  // 将指针添加到容器中
        }

        Sensor_IMUAcc = std::make_shared<DataFusion::SensorIMUAcc>(StateSpaceModel1_Sensors[0]);
        Sensor_IMUMagGyro = std::make_shared<DataFusion::SensorIMUMagGyro>(StateSpaceModel1_Sensors[1]);
        Sensor_Legs = std::make_shared<DataFusion::SensorLegs>(StateSpaceModel1_Sensors[0]);

        RCLCPP_INFO(this->get_logger(), "Fusion Estimator Node Initialized");
    }

    // 传感器状态空间模型创建
    std::vector<EstimatorPortN*> StateSpaceModel1_Sensors = {};// 容器声明
    std::vector<EstimatorPortN*> StateSpaceModel2_Sensors = {};// 容器声明

    void Init()
    {
        Sensor_Legs->KinematicParams  << 
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
                    std::cout << Sensor_Legs->KinematicParams.row(i).segment(jm.col, 3) << std::endl;
                }
                else
                {
                    urdf::Vector3 pos = joint->parent_to_joint_origin_transform.position;
                    Sensor_Legs->KinematicParams(i, jm.col)     = pos.x;
                    Sensor_Legs->KinematicParams(i, jm.col + 1) = pos.y;
                    Sensor_Legs->KinematicParams(i, jm.col + 2) = pos.z;
                    std::cout << "Obtained KinematicPar for " << jointName << ": ";
                    std::cout << Sensor_Legs->KinematicParams.row(i).segment(jm.col, 3) << std::endl;
                }
            }
    
            // 更新该腿 foot 连杆 collision 的球半径（存储在列 12）
            std::string footLinkName = leg + "_foot";
            urdf::LinkConstSharedPtr footLink = model->getLink(footLinkName);
            if (!footLink)
            {
                std::cout << "未找到连杆: " << footLinkName << " (" << leg << ")，使用默认值: " << Sensor_Legs->KinematicParams(i, 12) << std::endl;
            }
            else
            {
                if (footLink->collision && footLink->collision->geometry &&
                    footLink->collision->geometry->type == urdf::Geometry::SPHERE)
                {
                    urdf::Sphere* sphere = dynamic_cast<urdf::Sphere*>(footLink->collision->geometry.get());
                    if (sphere)
                    {
                        Sensor_Legs->KinematicParams(i, 12) = sphere->radius;
                        std::cout << "Obtained KinematicPar for " << footLinkName << ": " << Sensor_Legs->KinematicParams(i, 12) << std::endl;
                    }
                }
            }
    
            Sensor_Legs->Par_HipLength = std::sqrt(Sensor_Legs->KinematicParams(0, 0)*Sensor_Legs->KinematicParams(0, 0) + Sensor_Legs->KinematicParams(0, 1)*Sensor_Legs->KinematicParams(0, 1) + Sensor_Legs->KinematicParams(0, 2)*Sensor_Legs->KinematicParams(0, 2));
            Sensor_Legs->Par_ThighLength = std::sqrt(Sensor_Legs->KinematicParams(0, 3)*Sensor_Legs->KinematicParams(0, 3) + Sensor_Legs->KinematicParams(0, 4)*Sensor_Legs->KinematicParams(0, 4) + Sensor_Legs->KinematicParams(0, 5)*Sensor_Legs->KinematicParams(0, 5));
            Sensor_Legs->Par_CalfLength = std::sqrt(Sensor_Legs->KinematicParams(0, 6)*Sensor_Legs->KinematicParams(0, 6) + Sensor_Legs->KinematicParams(0, 7)*Sensor_Legs->KinematicParams(0, 7) + Sensor_Legs->KinematicParams(0, 8)*Sensor_Legs->KinematicParams(0, 8));
            Sensor_Legs->Par_FootLength = abs(Sensor_Legs->KinematicParams(0, 9));
        }
    }

private:

    rclcpp::Publisher<fusion_estimator::msg::FusionEstimatorTest>::SharedPtr publisher_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber_;
    std::shared_ptr<DataFusion::SensorIMUAcc> Sensor_IMUAcc; 
    std::shared_ptr<DataFusion::SensorIMUMagGyro> Sensor_IMUMagGyro; 
    std::shared_ptr<DataFusion::SensorLegs> Sensor_Legs; 

    void LowStateCallback(const void* message)
    {
        // 接收LowState数据
        const auto& low_state = *(unitree_go::msg::dds_::LowState_*)message;

        // 创建自定义消息对象
        auto fusion_msg = fusion_estimator::msg::FusionEstimatorTest();
        
        fusion_msg.stamp = this->get_clock()->now();


        for(int i=0; i<3; i++){
            fusion_msg.data_check_a[0+i] = low_state.imu_state().accelerometer()[i];
            fusion_msg.data_check_a[3+i] = low_state.imu_state().rpy()[i];
            fusion_msg.data_check_a[6+i] = low_state.imu_state().gyroscope()[i];
            fusion_msg.data_check_a[9+i] = low_state.foot_force()[i];;
        }

        // Start Estimation
        double Message[100]={0};

        rclcpp::Clock ros_clock(RCL_SYSTEM_TIME);
        rclcpp::Time CurrentTime = ros_clock.now();
        double CurrentTimestamp =  CurrentTime.seconds();

        for(int i = 0; i < 3; i++)
        {
            Message[3*i+2] = low_state.imu_state().accelerometer()[i];
        }
        Sensor_IMUAcc->SensorDataHandle(Message, CurrentTimestamp);
        for(int i=0; i<9; i++){
            fusion_msg.data_check_b[i] = StateSpaceModel1_Sensors[0]->EstimatedState[i];
        }

        for(int i = 0; i < 3; i++)
        {
            Message[3*i] = low_state.imu_state().rpy()[i];
        }
        for(int i = 0; i < 3; i++)
        {
            Message[3*i+1] = low_state.imu_state().gyroscope()[i];
        }
        Sensor_IMUMagGyro->SensorDataHandle(Message, CurrentTimestamp);
        for(int i=0; i<9; i++){
            fusion_msg.data_check_c[i] = StateSpaceModel1_Sensors[1]->EstimatedState[i];
        }

        for(int LegNumber = 0; LegNumber<4; LegNumber++)
        {
            for(int i = 0; i < 3; i++)
            {
                Message[LegNumber*3+i] = low_state.motor_state()[LegNumber*3+i].q();
                Message[12+ LegNumber*3+i] = low_state.motor_state()[LegNumber*3+i].dq();
            }
            Message[24 + LegNumber] = low_state.foot_force()[LegNumber];
        }
        Sensor_Legs->SensorDataHandle(Message, CurrentTimestamp);
        for(int i=0; i<4; i++){
            for(int j=0; j<3; j++){
                fusion_msg.data_check_d[3 * i + j] = StateSpaceModel1_Sensors[0]->Double_Par[6 * i + j];
                fusion_msg.data_check_e[3 * i + j] = StateSpaceModel1_Sensors[0]->Double_Par[6 * i + j + 3];
                fusion_msg.feet_based_position[3 * i + j] = StateSpaceModel1_Sensors[0]->Double_Par[24 + 6 * i + j];
                fusion_msg.feet_based_velocity[3 * i + j] = StateSpaceModel1_Sensors[0]->Double_Par[24 + 6 * i + j + 3];
            }
        }

        publisher_->publish(fusion_msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FusionEstimatorNode>();
    node->Init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
