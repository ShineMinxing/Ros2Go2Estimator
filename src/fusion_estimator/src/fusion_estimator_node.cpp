#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>
#include <filesystem>
#include <urdf_parser/urdf_parser.h>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/imu.hpp>

#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/idl/go2/LowState_.hpp"

#include "fusion_estimator/msg/fusion_estimator_test.hpp" 
#include "GO2FusionEstimator/Estimator/EstimatorPortN.h"
#include "GO2FusionEstimator/Sensor_Legs.h" 
#include "GO2FusionEstimator/Sensor_IMU.h" 

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
        Lowstate_subscriber.reset(
            new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>("rt/lowstate"));
        Lowstate_subscriber->InitChannel(
            std::bind(&FusionEstimatorNode::LowStateCallback, this, std::placeholders::_1), 1);

        FETest_publisher = this->create_publisher<fusion_estimator::msg::FusionEstimatorTest>(
            "SMXFE/Estimation", 10);

        SMXFE_publisher = this->create_publisher<nav_msgs::msg::Odometry>("SMXFE/Odom", 10);

        for(int i = 0; i < 2; i++)
        {
            EstimatorPortN* StateSpaceModel1_SensorsPtrs = new EstimatorPortN; // 创建新的结构体实例
            StateSpaceModel1_Initialization(StateSpaceModel1_SensorsPtrs);     // 调用初始化函数
            StateSpaceModel1_Sensors.push_back(StateSpaceModel1_SensorsPtrs);  // 将指针添加到容器中
        }

        Sensor_IMUAcc = std::make_shared<DataFusion::SensorIMUAcc>(StateSpaceModel1_Sensors[0]);
        Sensor_IMUMagGyro = std::make_shared<DataFusion::SensorIMUMagGyro>(StateSpaceModel1_Sensors[1]);
        Sensor_Legs = std::make_shared<DataFusion::SensorLegs>(StateSpaceModel1_Sensors[0]);

        this->declare_parameter<double>("Modify_Par_1", 0.0);
        this->declare_parameter<double>("Modify_Par_2", 0.0);
        this->declare_parameter<double>("Modify_Par_3", 0.0);
        ParameterCorrectCallback = this->add_on_set_parameters_callback(
            std::bind(&FusionEstimatorNode::Modify_Par_Fun, this, std::placeholders::_1)
          );

        RCLCPP_INFO(this->get_logger(), "FusionEstimatorNode 已启动");
    }

    // 传感器状态空间模型创建
    std::vector<EstimatorPortN*> StateSpaceModel1_Sensors = {};// 容器声明
    std::vector<EstimatorPortN*> StateSpaceModel2_Sensors = {};// 容器声明

    void ObtainParameter();

private:

    rclcpp::Publisher<fusion_estimator::msg::FusionEstimatorTest>::SharedPtr FETest_publisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr SMXFE_publisher;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> Lowstate_subscriber;
    std::shared_ptr<DataFusion::SensorIMUAcc> Sensor_IMUAcc; 
    std::shared_ptr<DataFusion::SensorIMUMagGyro> Sensor_IMUMagGyro; 
    std::shared_ptr<DataFusion::SensorLegs> Sensor_Legs; 

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr ParameterCorrectCallback;
    rcl_interfaces::msg::SetParametersResult Modify_Par_Fun(
    const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "Sensor_IMUMagGyro->SensorQuaternion is corrected.";
        double AngleCorrect[3] = {0};

        for (const auto & param : parameters)
        {
            if (param.get_name() == "Modify_Par_1")
                AngleCorrect[0] = param.as_double() * M_PI / 180.0;
            if (param.get_name() == "Modify_Par_2")
                AngleCorrect[1] = param.as_double() * M_PI / 180.0;
            if (param.get_name() == "Modify_Par_3")
                AngleCorrect[2] = param.as_double() * M_PI / 180.0;
        }

        Eigen::AngleAxisd rollAngle(AngleCorrect[2], Eigen::Vector3d::UnitZ());  // 绕 X 轴旋转
        Eigen::AngleAxisd pitchAngle(AngleCorrect[1], Eigen::Vector3d::UnitY()); // 绕 Y 轴旋转
        Eigen::AngleAxisd yawAngle(AngleCorrect[0], Eigen::Vector3d::UnitX());   // 绕 Z 轴旋转

        Sensor_IMUMagGyro->SensorQuaternion = yawAngle * pitchAngle * rollAngle;
        Sensor_IMUMagGyro->SensorQuaternionInv = Sensor_IMUMagGyro->SensorQuaternion.inverse();
        std::cout <<"Sensor_IMUMagGyro->SensorQuaternion: " << Sensor_IMUMagGyro->SensorQuaternion.coeffs().transpose() << std::endl;

        return result;
    }

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
        }

        for(int LegNumber = 0; LegNumber<4; LegNumber++)
        {
            for(int i = 0; i < 3; i++)
            {
                fusion_msg.data_check_b[LegNumber*3+i] = low_state.motor_state()[LegNumber*3+i].q();
                fusion_msg.data_check_c[LegNumber*3+i] = low_state.motor_state()[LegNumber*3+i].dq();
            }
        }

        // Start Estimation
        double LatestMessage[3][100]={0};
        static double LastMessage[3][100]={0};

        rclcpp::Clock ros_clock(RCL_SYSTEM_TIME);
        rclcpp::Time CurrentTime = ros_clock.now();
        double CurrentTimestamp =  CurrentTime.seconds();

        for(int i = 0; i < 3; i++)
        {
            LatestMessage[0][3*i+2] = low_state.imu_state().accelerometer()[i];
        }
        for(int i = 0; i < 9; i++)
        {
            if(LastMessage[0][i] != LatestMessage[0][i])
            {
                Sensor_IMUAcc->SensorDataHandle(LatestMessage[0], CurrentTimestamp);
                for(int j = 0; j < 9; j++)
                {
                    LastMessage[0][j] = LatestMessage[0][j];
                }
                break;
            }
        }
        for(int i=0; i<9; i++){
            fusion_msg.estimated_xyz[i] = StateSpaceModel1_Sensors[0]->EstimatedState[i];
        }

        for(int i = 0; i < 3; i++)
        {
            LatestMessage[1][3*i] = low_state.imu_state().rpy()[i];
        }
        for(int i = 0; i < 3; i++)
        {
            LatestMessage[1][3*i+1] = low_state.imu_state().gyroscope()[i];
        }
        for(int i = 0; i < 9; i++)
        {
            if(LastMessage[1][i] != LatestMessage[1][i])
            {
                Sensor_IMUMagGyro->SensorDataHandle(LatestMessage[1], CurrentTimestamp);
                for(int j = 0; j < 9; j++)
                {
                    LastMessage[1][j] = LatestMessage[1][j];
                }
                break;
            }
        }
        for(int i=0; i<9; i++){
            fusion_msg.estimated_rpy[i] = StateSpaceModel1_Sensors[1]->EstimatedState[i];
        }

        for(int LegNumber = 0; LegNumber<4; LegNumber++)
        {
            for(int i = 0; i < 3; i++)
            {
                LatestMessage[2][LegNumber*3+i] = low_state.motor_state()[LegNumber*3+i].q();
                LatestMessage[2][12+ LegNumber*3+i] = low_state.motor_state()[LegNumber*3+i].dq();
            }
            LatestMessage[2][24 + LegNumber] = low_state.foot_force()[LegNumber];
            fusion_msg.others[LegNumber] = low_state.foot_force()[LegNumber];
            fusion_msg.others[LegNumber] = fusion_msg.others[LegNumber];
        }
        for(int i = 0; i < 28; i++)
        {
            if(LastMessage[2][i] != LatestMessage[2][i])
            {
                Sensor_Legs->SensorDataHandle(LatestMessage[2], CurrentTimestamp);
                for(int j = 0; j < 28; j++)
                {
                    LastMessage[2][j] = LatestMessage[2][j];
                }
                break;
            }
        }
        for(int i=0; i<4; i++){
            for(int j=0; j<3; j++){
                fusion_msg.data_check_d[3 * i + j] = StateSpaceModel1_Sensors[0]->Double_Par[6 * i + j];
                fusion_msg.data_check_e[3 * i + j] = StateSpaceModel1_Sensors[0]->Double_Par[24 + 6 * i + j];
                fusion_msg.feet_based_position[3 * i + j] = StateSpaceModel1_Sensors[0]->Double_Par[48 + 6 * i + j];
                fusion_msg.feet_based_velocity[3 * i + j] = StateSpaceModel1_Sensors[0]->Double_Par[48 + 6 * i + j + 3];
            }
        }

        FETest_publisher->publish(fusion_msg);

        // 新增：构造标准 odometry 消息，并发布
        nav_msgs::msg::Odometry SMXFE_odom;
        SMXFE_odom.header.stamp = fusion_msg.stamp;
        SMXFE_odom.header.frame_id = "odom";
        SMXFE_odom.child_frame_id = "base_link";

        // 使用 fusion_msg.estimated_xyz 的前 3 个元素作为位置
        SMXFE_odom.pose.pose.position.x = fusion_msg.estimated_xyz[0];
        SMXFE_odom.pose.pose.position.y = fusion_msg.estimated_xyz[3];
        SMXFE_odom.pose.pose.position.z = fusion_msg.estimated_xyz[6];

        // 使用 fusion_msg.estimated_rpy 的前 3 个元素（roll, pitch, yaw）转换为四元数
        tf2::Quaternion q;
        q.setRPY(fusion_msg.estimated_rpy[0], fusion_msg.estimated_rpy[3], fusion_msg.estimated_rpy[6]);
        SMXFE_odom.pose.pose.orientation = tf2::toMsg(q);

        // 线速度：使用 fusion_msg.estimated_xyz 的索引 1, 4, 7
        SMXFE_odom.twist.twist.linear.x = fusion_msg.estimated_xyz[1];
        SMXFE_odom.twist.twist.linear.y = fusion_msg.estimated_xyz[4];
        SMXFE_odom.twist.twist.linear.z = fusion_msg.estimated_xyz[7];

        // 角速度：使用 fusion_msg.estimated_rpy 的索引 1, 4, 7
        SMXFE_odom.twist.twist.angular.x = fusion_msg.estimated_rpy[1];
        SMXFE_odom.twist.twist.angular.y = fusion_msg.estimated_rpy[4];
        SMXFE_odom.twist.twist.angular.z = fusion_msg.estimated_rpy[7];

        // 设置位姿（pose）协方差：6x6 矩阵（行优先排列）
        // 初始化全部置零
        for (int i = 0; i < 36; ++i) {
            SMXFE_odom.pose.covariance[i] = 0.0;
        }
        // 位置 (x, y, z) 的协方差设为 0.01
        SMXFE_odom.pose.covariance[0]  = 0.01;   // x
        SMXFE_odom.pose.covariance[7]  = 0.01;   // y
        SMXFE_odom.pose.covariance[14] = 0.01;   // z
        // 姿态（roll, pitch, yaw）的协方差设为 0.0001
        SMXFE_odom.pose.covariance[21] = 0.0001; // roll
        SMXFE_odom.pose.covariance[28] = 0.0001; // pitch
        SMXFE_odom.pose.covariance[35] = 0.0001; // yaw

        // 设置 twist 协方差：6x6 矩阵（行优先排列）
        for (int i = 0; i < 36; ++i) {
            SMXFE_odom.twist.covariance[i] = 0.0;
        }
        // 线速度 (x, y, z) 的协方差设为 0.1
        SMXFE_odom.twist.covariance[0]  = 0.1;   // linear x
        SMXFE_odom.twist.covariance[7]  = 0.1;   // linear y
        SMXFE_odom.twist.covariance[14] = 0.1;   // linear z
        // 角速度 (x, y, z) 的协方差设为 0.01
        SMXFE_odom.twist.covariance[21] = 0.01;  // angular x
        SMXFE_odom.twist.covariance[28] = 0.01;  // angular y
        SMXFE_odom.twist.covariance[35] = 0.01;  // angular z

        // 发布 odometry 消息
        SMXFE_publisher->publish(SMXFE_odom);
        }
};

void FusionEstimatorNode::ObtainParameter()
{
    // 获得机器狗运动学参数
    Sensor_Legs->KinematicParams  << 
    0.1934,  0.0465,  0.000,   0.0,  0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022,
    0.1934, -0.0465,  0.000,   0.0, -0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022,
    -0.1934, -0.0465,  0.000,   0.0, -0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022,
    -0.1934,  0.0465,  0.000,   0.0,  0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022;
    std::filesystem::path current_file(__FILE__);
    std::filesystem::path package_dir = current_file.parent_path().parent_path();
    std::filesystem::path urdf_path = package_dir / "cfg" / "go2_description.urdf";
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

        Sensor_Legs->Par_HipLength = std::sqrt(Sensor_Legs->KinematicParams(0, 3)*Sensor_Legs->KinematicParams(0, 3) + Sensor_Legs->KinematicParams(0, 4)*Sensor_Legs->KinematicParams(0, 4) + Sensor_Legs->KinematicParams(0, 5)*Sensor_Legs->KinematicParams(0, 5));
        Sensor_Legs->Par_ThighLength = std::sqrt(Sensor_Legs->KinematicParams(0, 6)*Sensor_Legs->KinematicParams(0, 6) + Sensor_Legs->KinematicParams(0, 7)*Sensor_Legs->KinematicParams(0, 7) + Sensor_Legs->KinematicParams(0, 8)*Sensor_Legs->KinematicParams(0, 8));
        Sensor_Legs->Par_CalfLength = std::sqrt(Sensor_Legs->KinematicParams(0, 9)*Sensor_Legs->KinematicParams(0, 9) + Sensor_Legs->KinematicParams(0, 10)*Sensor_Legs->KinematicParams(0, 10) + Sensor_Legs->KinematicParams(0, 11)*Sensor_Legs->KinematicParams(0, 11));
        Sensor_Legs->Par_FootLength = abs(Sensor_Legs->KinematicParams(0, 12));
    }

    // 获得IMU安装位置
    Eigen::Vector3d IMUPosition(-0.02557, 0, 0.04232);
    std::string jointName = "imu_joint";
    urdf::JointConstSharedPtr joint = model->getJoint(jointName);
    if (!joint)
    {
        std::cout << "未找到关节: " << jointName << ", 使用默认值： ";
        std::cout << IMUPosition.transpose() << std::endl;
    }
    else
    {
        urdf::Vector3 pos = joint->parent_to_joint_origin_transform.position;
        std::cout << "Obtained Position for " << jointName << ": ";
        std::cout << IMUPosition.transpose() << std::endl;
    }
    Sensor_IMUAcc->SensorPosition[0] = IMUPosition(0);
    Sensor_IMUAcc->SensorPosition[1] = IMUPosition(1);
    Sensor_IMUAcc->SensorPosition[2] = IMUPosition(2);
    Sensor_IMUMagGyro->SensorPosition[0] = IMUPosition(0);
    Sensor_IMUMagGyro->SensorPosition[1] = IMUPosition(1);
    Sensor_IMUMagGyro->SensorPosition[2] = IMUPosition(2);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FusionEstimatorNode>();
    node->ObtainParameter();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
