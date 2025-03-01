#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/idl/go2/LowState_.hpp"
#include "Estimator/EstimatorPortN.h"
#include "fusion_estimator/msg/fusion_estimator_test.hpp" 
#include "Sensor_IMU.h" 
#include "Sensor_Legs.h" 

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

private:

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

        // Start Estimation
 
        Sensor_IMUAcc->SensorDataHandle(low_state);
        for(int i=0; i<9; i++){
            fusion_msg.data_check_b[i] = StateSpaceModel1_Sensors[0]->EstimatedState[i];
        }
        Sensor_IMUMagGyro->SensorDataHandle(low_state);
        for(int i=0; i<9; i++){
            fusion_msg.data_check_c[i] = StateSpaceModel1_Sensors[1]->EstimatedState[i];
        }

        publisher_->publish(fusion_msg);
    }

    rclcpp::Publisher<fusion_estimator::msg::FusionEstimatorTest>::SharedPtr publisher_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber_;
    std::shared_ptr<DataFusion::SensorIMUAcc> Sensor_IMUAcc; 
    std::shared_ptr<DataFusion::SensorIMUMagGyro> Sensor_IMUMagGyro; 
    std::shared_ptr<DataFusion::SensorLegs> Sensor_Legs; 
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FusionEstimatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
