#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <iomanip>

#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
//#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> doesn't work on Ubuntu20.04
#include "std_msgs/msg/float64_multi_array.hpp"
#include <sensor_msgs/msg/imu.hpp>

#include "fusion_estimator/msg/fusion_estimator_test.hpp"
#include "GO2FusionEstimator/fusion_estimator.h"

class FusionEstimatorNode : public rclcpp::Node
{
public:
    FusionEstimatorNode(const rclcpp::NodeOptions &options)
    : Node("fusion_estimator_node", options)
    {        
        for (int i = 0; i < 9; ++i) {
            position_correct[i] = 0;
            orientation_correct[i] = 0;
        }

        /* ────────────── 读取参数 ────────────── */
        std::string sub_imu_topic;
        this->get_parameter_or("sub_imu_topic", sub_imu_topic, std::string("NoYamlRead/Go2IMU"));
        std::string sub_joint_topic;
        this->get_parameter_or("sub_joint_topic", sub_joint_topic, std::string("NoYamlRead/Go2Joint"));
        std::string sub_mode_topic;
        this->get_parameter_or("sub_mode_topic", sub_mode_topic, std::string("NoYamlRead/SportCmd"));
        std::string pub_estimation_topic;
        this->get_parameter_or("pub_estimation_topic", pub_estimation_topic, std::string("NoYamlRead/Estimation"));
        std::string pub_odom_topic;
        this->get_parameter_or("pub_odom_topic", pub_odom_topic, std::string("NoYamlRead/Odom"));
        std::string pub_odom_2d_topic;
        this->get_parameter_or("pub_odom2d_topic", pub_odom_2d_topic, std::string("NoYamlRead/Odom_2D"));
        this->get_parameter_or("odom_frame", odom_frame_id, std::string("odom"));
        this->get_parameter_or("base_frame", child_frame_id, std::string("base_link"));
        this->get_parameter_or("base_frame_2d", child_frame_2d_id, std::string("base_link_2D"));

        this->get_parameter_or("imu_data_enable", imu_data_enable, true);
        this->get_parameter_or("leg_pos_enable", leg_pos_enable, true);
        this->get_parameter_or("leg_vel_enable", leg_vel_enable, true);
        this->get_parameter_or("foot_force_threshold", foot_force_threshold, 20.0);
        this->get_parameter_or("min_stair_height", min_stair_height, 0.08);
        this->get_parameter_or("stair_height_fogotten", stair_height_fogotten, 60.0);

        this->get_parameter_or("leg_ori_enable", leg_ori_enable, true);
        this->get_parameter_or("leg_ori_init_weight", leg_ori_init_weight, 0.001);
        this->get_parameter_or("leg_ori_time_wight", leg_ori_time_wight, 100.0);
        if(!(leg_ori_init_weight>-1.0 && leg_ori_init_weight<=1.0))
        {
            leg_ori_init_weight = 0.001;
            RCLCPP_INFO(get_logger(), "leg_ori_init_weight to %lf",leg_ori_init_weight);
        }
        if(!(leg_ori_time_wight>0.0 && leg_ori_time_wight<=1000000.0))
        {
            leg_ori_time_wight = 100.0;
            RCLCPP_INFO(get_logger(), "leg_ori_time_wight to %lf", leg_ori_time_wight);
        }


        /* ────────────── 设置参数 ────────────── */
        double status[200] = {0};
        status[IndexInOrOut] = 1;
        // enable 开关
        status[IndexIMUAccEnable]         = imu_data_enable ? 1.0 : 0.0;
        status[IndexIMUQuaternionEnable]  = imu_data_enable ? 1.0 : 0.0;
        status[IndexIMUGyroEnable]        = imu_data_enable ? 1.0 : 0.0;
        status[IndexJointsXYZEnable]          = leg_pos_enable ? 1.0 : 0.0;
        status[IndexJointsVelocityXYZEnable]  = leg_vel_enable ? 1.0 : 0.0;
        status[IndexJointsRPYEnable]          = leg_ori_enable ? 1.0 : 0.0;
        // 阈值/权重
        status[IndexLegFootForceThreshold]       = -1.0;
        status[IndexLegMinStairHeight]           = min_stair_height;
        status[IndexLegOrientationInitialWeight] = leg_ori_init_weight;
        status[IndexLegOrientationTimeWeight]    = leg_ori_time_wight;

        fe_.fusion_estimator_status(status);

        /* ────────────── 数据通信 ────────────── */
        go2_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(sub_imu_topic, 10, std::bind(&FusionEstimatorNode::imu_callback, this, std::placeholders::_1));
        go2_joint_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(sub_joint_topic, 10, std::bind(&FusionEstimatorNode::joint_callback, this, std::placeholders::_1));
        joystick_cmd_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            sub_mode_topic, 10,
            std::bind(&FusionEstimatorNode::joystick_cmd_callback, this, std::placeholders::_1));
        parameter_sub = this->add_on_set_parameters_callback(std::bind(&FusionEstimatorNode::Modify_Par_Fun, this, std::placeholders::_1));

        FETest_publisher = this->create_publisher<fusion_estimator::msg::FusionEstimatorTest>(
        pub_estimation_topic, 10);
        SMXFE_publisher   = this->create_publisher<nav_msgs::msg::Odometry>(pub_odom_topic, 10);
        SMXFE_2D_publisher= this->create_publisher<nav_msgs::msg::Odometry>(pub_odom_2d_topic, 10);
    }

private:
    FusionEstimatorCore fe_;     // 核心融合器（已封装好）
    FE_LowlevelState st_;        // “静态”输入缓存（节点生命周期内一直保留）
    FE_Odometer odom_;           // “静态”输出缓存（仅 joint 回调更新）

    rclcpp::Publisher<fusion_estimator::msg::FusionEstimatorTest>::SharedPtr FETest_publisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr SMXFE_publisher, SMXFE_2D_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr go2_imu_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr go2_joint_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joystick_cmd_sub;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_sub;

    fusion_estimator::msg::FusionEstimatorTest fusion_msg;

    std::string odom_frame_id, child_frame_id, child_frame_2d_id;
    bool imu_data_enable, leg_pos_enable, leg_vel_enable, leg_ori_enable;
    bool msg_received[2] = {0,0};
    double foot_force_threshold, min_stair_height, stair_height_fogotten;

    double leg_ori_init_weight, leg_ori_time_wight;
    double position_correct[9];
    double orientation_correct[9];

    rcl_interfaces::msg::SetParametersResult Modify_Par_Fun(
    const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto & param : parameters)
        {
            if (param.get_name() == "Modify_Par_X")
                position_correct[0] = param.as_double();
            if (param.get_name() == "Modify_Par_Y")
                position_correct[3] = param.as_double();
            if (param.get_name() == "Modify_Par_Z")
                position_correct[6] = param.as_double();
            if (param.get_name() == "Modify_Par_roll")
                orientation_correct[0] = param.as_double() * M_PI / 180.0;
            if (param.get_name() == "Modify_Par_pitch")
                orientation_correct[3] = param.as_double() * M_PI / 180.0;
            if (param.get_name() == "Modify_Par_yaw")
                orientation_correct[6] = param.as_double() * M_PI / 180.0;
        }

        return result;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        rclcpp::Clock ros_clock(RCL_SYSTEM_TIME);
        rclcpp::Time now = ros_clock.now();
        rclcpp::Time msg_time(msg->header.stamp, now.get_clock_type());

        if (now > msg_time) fusion_msg.stamp = now;
        else                fusion_msg.stamp = msg_time + rclcpp::Duration(0, 1);

        st_.timestamp = msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;

        st_.imu_acc[0] = msg->linear_acceleration.x;
        st_.imu_acc[1] = msg->linear_acceleration.y;
        st_.imu_acc[2] = msg->linear_acceleration.z;

        st_.imu_gyro[0] = msg->angular_velocity.x;
        st_.imu_gyro[1] = msg->angular_velocity.y;
        st_.imu_gyro[2] = msg->angular_velocity.z;

        st_.imu_quat[0] = msg->orientation.w;
        st_.imu_quat[1] = msg->orientation.x;
        st_.imu_quat[2] = msg->orientation.y;
        st_.imu_quat[3] = msg->orientation.z;

        msg_received[0] = 1;
    }

    void joint_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (st_.timestamp <= 0.0) return; 
        const auto& arr = msg->data;
        if (arr.size() < 28) return;

        // joint topic layout
        // data[ 0..11] : 12×q
        // data[12..23] : 12×dq
        // data[24..27] : 4×foot_force

        // 12 个关节对应的电机编号（你原始代码里用的那套）
        static const int desired_joints[] = {0,1,2, 4,5,6, 8,9,10, 12,13,14};

        // 写入 st_
        for (int i = 0; i < 12; ++i) {
            const int mid = desired_joints[i];
            st_.motor_q[mid]  = arr[i];
            st_.motor_dq[mid] = arr[12 + i];
            st_.motor_tau[mid] = 0.0;
        }

        // 将足段触地1/0等效为小腿关节大力矩
        static const int tau_idx[4] = {2, 6, 10, 14};
        for (int leg = 0; leg < 4; ++leg) {
            const bool on = (arr[24 + leg] >= foot_force_threshold);
            st_.motor_tau[tau_idx[leg]] = on ? 100.0 : 0.0;
        }

        // 只有 joint_callback 调用融合更新
        odom_ = fe_.fusion_estimator(st_);

        // 回填 fusion_msg（沿用你原来的 9 维布局：x,v,a / y,v,a / z,v,a）
        fusion_msg.estimated_xyz[0] = odom_.pos[0] + position_correct[0];
        fusion_msg.estimated_xyz[1] = odom_.vel[0] + position_correct[1];
        fusion_msg.estimated_xyz[2] = odom_.acc[0] + position_correct[2];

        fusion_msg.estimated_xyz[3] = odom_.pos[1] + position_correct[3];
        fusion_msg.estimated_xyz[4] = odom_.vel[1] + position_correct[4];
        fusion_msg.estimated_xyz[5] = odom_.acc[1] + position_correct[5];

        fusion_msg.estimated_xyz[6] = odom_.pos[2] + position_correct[6];
        fusion_msg.estimated_xyz[7] = odom_.vel[2] + position_correct[7];
        fusion_msg.estimated_xyz[8] = odom_.acc[2] + position_correct[8];

        fusion_msg.estimated_rpy[0] = odom_.rpy[0] + orientation_correct[0];
        fusion_msg.estimated_rpy[1] = odom_.rpy_rate[0] + orientation_correct[1];
        fusion_msg.estimated_rpy[2] = odom_.rpy_acc[0] + orientation_correct[2];

        fusion_msg.estimated_rpy[3] = odom_.rpy[1] + orientation_correct[3];
        fusion_msg.estimated_rpy[4] = odom_.rpy_rate[1] + orientation_correct[4];
        fusion_msg.estimated_rpy[5] = odom_.rpy_acc[1] + orientation_correct[5];

        fusion_msg.estimated_rpy[6] = odom_.rpy[2] + orientation_correct[6];
        fusion_msg.estimated_rpy[7] = odom_.rpy_rate[2] + orientation_correct[7];
        fusion_msg.estimated_rpy[8] = odom_.rpy_acc[2] + orientation_correct[8];

        msg_received[1] = 1;
        Msg_Publish();   // 只在 joint 回调里 publish
    }


    void Msg_Publish()
    {
        if (!msg_received[0] || !msg_received[1])
            return;
        msg_received[0] = 0;
        msg_received[1] = 0;

        FETest_publisher->publish(fusion_msg);

        // 构造标准 3D odometry 消息，并发布
        nav_msgs::msg::Odometry SMXFE_odom;
        SMXFE_odom.header.stamp = fusion_msg.stamp;
        SMXFE_odom.header.frame_id = odom_frame_id;
        SMXFE_odom.child_frame_id = child_frame_id;

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
        SMXFE_odom.pose.covariance[0]  = 0.1;   // x
        SMXFE_odom.pose.covariance[7]  = 0.1;   // y
        SMXFE_odom.pose.covariance[14] = 0.1;   // z
        // 姿态（roll, pitch, yaw）的协方差设为 0.0001
        SMXFE_odom.pose.covariance[21] = 0.1; // roll
        SMXFE_odom.pose.covariance[28] = 0.1; // pitch
        SMXFE_odom.pose.covariance[35] = 0.1; // yaw

        // 设置 twist 协方差：6x6 矩阵（行优先排列）
        for (int i = 0; i < 36; ++i) {
            SMXFE_odom.twist.covariance[i] = 0.0;
        }
        // 线速度 (x, y, z) 的协方差设为 0.1
        SMXFE_odom.twist.covariance[0]  = 0.1;   // linear x
        SMXFE_odom.twist.covariance[7]  = 0.1;   // linear y
        SMXFE_odom.twist.covariance[14] = 0.1;   // linear z
        // 角速度 (x, y, z) 的协方差设为 0.01
        SMXFE_odom.twist.covariance[21] = 0.1;  // angular x
        SMXFE_odom.twist.covariance[28] = 0.1;  // angular y
        SMXFE_odom.twist.covariance[35] = 0.1;  // angular z

        // 发布 odometry 消息
        SMXFE_publisher->publish(SMXFE_odom);

        // 构造标准 2D odometry 消息，并发布
        nav_msgs::msg::Odometry SMXFE_odom_2D;
        SMXFE_odom_2D.header.stamp = fusion_msg.stamp;
        SMXFE_odom_2D.header.frame_id = odom_frame_id;
        SMXFE_odom_2D.child_frame_id = child_frame_2d_id;

        // 使用 fusion_msg.estimated_xyz 的前 3 个元素作为位置
        SMXFE_odom_2D.pose.pose.position.x = fusion_msg.estimated_xyz[0];
        SMXFE_odom_2D.pose.pose.position.y = fusion_msg.estimated_xyz[3];
        SMXFE_odom_2D.pose.pose.position.z = 0;

        // 使用 fusion_msg.estimated_rpy 的前 3 个元素（roll, pitch, yaw）转换为四元数
        q.setRPY(0, 0, fusion_msg.estimated_rpy[6]);
        SMXFE_odom_2D.pose.pose.orientation = tf2::toMsg(q);

        // 线速度：使用 fusion_msg.estimated_xyz 的索引 1, 4, 7
        SMXFE_odom_2D.twist.twist.linear.x = fusion_msg.estimated_xyz[1];
        SMXFE_odom_2D.twist.twist.linear.y = fusion_msg.estimated_xyz[4];
        SMXFE_odom_2D.twist.twist.linear.z = fusion_msg.estimated_xyz[7];

        // 角速度：使用 fusion_msg.estimated_rpy 的索引 1, 4, 7
        SMXFE_odom_2D.twist.twist.angular.x = fusion_msg.estimated_rpy[1];
        SMXFE_odom_2D.twist.twist.angular.y = fusion_msg.estimated_rpy[4];
        SMXFE_odom_2D.twist.twist.angular.z = fusion_msg.estimated_rpy[7];

        // 设置位姿（pose）协方差：6x6 矩阵（行优先排列）
        // 初始化全部置零
        for (int i = 0; i < 36; ++i) {
            SMXFE_odom_2D.pose.covariance[i] = 0.0;
        }
        // 位置 (x, y, z) 的协方差设为 0.01
        SMXFE_odom_2D.pose.covariance[0]  = 0.1;   // x
        SMXFE_odom_2D.pose.covariance[7]  = 0.1;   // y
        SMXFE_odom_2D.pose.covariance[14] = 0.1;   // z
        // 姿态（roll, pitch, yaw）的协方差设为 0.0001
        SMXFE_odom_2D.pose.covariance[21] = 0.1; // roll
        SMXFE_odom_2D.pose.covariance[28] = 0.1; // pitch
        SMXFE_odom_2D.pose.covariance[35] = 0.1; // yaw

        // 设置 twist 协方差：6x6 矩阵（行优先排列）
        for (int i = 0; i < 36; ++i) {
            SMXFE_odom_2D.twist.covariance[i] = 0.0;
        }
        // 线速度 (x, y, z) 的协方差设为 0.1
        SMXFE_odom_2D.twist.covariance[0]  = 0.1;   // linear x
        SMXFE_odom_2D.twist.covariance[7]  = 0.1;   // linear y
        SMXFE_odom_2D.twist.covariance[14] = 0.1;   // linear z
        // 角速度 (x, y, z) 的协方差设为 0.01
        SMXFE_odom_2D.twist.covariance[21] = 0.1;  // angular x
        SMXFE_odom_2D.twist.covariance[28] = 0.1;  // angular y
        SMXFE_odom_2D.twist.covariance[35] = 0.1;  // angular z

        SMXFE_2D_publisher->publish(SMXFE_odom_2D);
    }

    // 回调函数，处理SportCmd消息
    void joystick_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data[0] == 25140000)
        {
            double status[200] = {0};
            status[IndexInOrOut] = 3;
            fe_.fusion_estimator_status(status);
        }
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)
    .arguments({
        "--ros-args",
        "--params-file", "/home/shine/SMX/ros2_ioe/src/Ros2Go2Estimator/config.yaml"
    });

  auto node = std::make_shared<FusionEstimatorNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}