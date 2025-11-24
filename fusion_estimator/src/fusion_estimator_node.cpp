#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <iomanip>

#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <sensor_msgs/msg/imu.hpp>

#include "fusion_estimator/msg/fusion_estimator_test.hpp"
#include "GO2FusionEstimator/Estimator/EstimatorPortN.h"
#include "GO2FusionEstimator/Sensor_Legs.h"
#include "GO2FusionEstimator/Sensor_IMU.h"
#include "GO2FusionEstimator/Sensor_IMU.h"

using namespace DataFusion;

class FusionEstimatorNode : public rclcpp::Node
{
public:
    FusionEstimatorNode(const rclcpp::NodeOptions &options)
    : Node("fusion_estimator_node", options)
    {
        /* ────────────── 创建融合器对象 ────────────── */
        for (int i = 0; i < 2; ++i) {
        auto * ptr = new EstimatorPortN;
        StateSpaceModel1_Initialization(ptr);
        StateSpaceModel1_Sensors.emplace_back(ptr);
        }
        Sensor_IMUAcc      = std::make_shared<SensorIMUAcc>     (StateSpaceModel1_Sensors[0]);
        Sensor_IMUMagGyro  = std::make_shared<SensorIMUMagGyro> (StateSpaceModel1_Sensors[1]);
        Sensor_LegsPos     = std::make_shared<SensorLegsPos>    (StateSpaceModel1_Sensors[0]);
        Sensor_LegsOri     = std::make_shared<SensorLegsOri>    (StateSpaceModel1_Sensors[1]);
        Sensor_LegsOri->SetLegsPosRef(Sensor_LegsPos.get());
        
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

        rclcpp::Parameter kin_param;
        if (this->get_parameter("kinematic_params", kin_param)) 
        {
            auto v = kin_param.as_double_array();  // std::vector<double>
            if (v.size() == 4u * 13u) {
                // 用 Eigen::Map 把一维数组映射成 4×13 矩阵（行优先）
                Eigen::Map<const Eigen::Matrix<double, 4, 13, Eigen::RowMajor>> kin_map(v.data());
                Sensor_LegsPos->KinematicParams = kin_map;

                // 用新的 KinematicParams 更新几何长度（若你想保持默认值可以直接删掉下面4行）
                Sensor_LegsPos->Par_HipLength   = Sensor_LegsPos->KinematicParams.block<1,3>(0,3).norm();
                Sensor_LegsPos->Par_ThighLength = Sensor_LegsPos->KinematicParams.block<1,3>(0,6).norm();
                Sensor_LegsPos->Par_CalfLength  = Sensor_LegsPos->KinematicParams.block<1,3>(0,9).norm();
                Sensor_LegsPos->Par_FootLength  = std::abs(Sensor_LegsPos->KinematicParams(0,12));
                RCLCPP_INFO(this->get_logger(), "Kinematic Params Sucessfully Read.");
            }
        }

        rclcpp::Parameter imu_param;
        if (this->get_parameter("imu_params", imu_param)) {
            auto v = imu_param.as_double_array();   // 期望 6 个
            if (v.size() == 6u) {
                Sensor_IMUAcc->SensorPosition[0]     = v[0];
                Sensor_IMUAcc->SensorPosition[1]     = v[1];
                Sensor_IMUAcc->SensorPosition[2]     = v[2];
                Sensor_IMUMagGyro->SensorPosition[0] = v[0];
                Sensor_IMUMagGyro->SensorPosition[1] = v[1];
                Sensor_IMUMagGyro->SensorPosition[2] = v[2];
                double r = v[3] * M_PI / 180.0;
                double p = v[4] * M_PI / 180.0;
                double y = v[5] * M_PI / 180.0;
                Eigen::AngleAxisd rollAngle (y, Eigen::Vector3d::UnitZ());
                Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
                Eigen::AngleAxisd yawAngle  (r, Eigen::Vector3d::UnitX());
                Sensor_IMUAcc->SensorQuaternion = yawAngle * pitchAngle * rollAngle;
                Sensor_IMUAcc->SensorQuaternionInv = Sensor_IMUAcc->SensorQuaternion.inverse();
                Sensor_IMUMagGyro->SensorQuaternion = yawAngle * pitchAngle * rollAngle;
                Sensor_IMUMagGyro->SensorQuaternionInv = Sensor_IMUMagGyro->SensorQuaternion.inverse();
                RCLCPP_INFO(this->get_logger(), "IMU Params Sucessfully Read.");
            }
        }

        /* ────────────── 数据通信 ────────────── */
        if(imu_data_enable)
            go2_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(sub_imu_topic, 10, std::bind(&FusionEstimatorNode::imu_callback, this, std::placeholders::_1));
        if(leg_pos_enable || leg_ori_enable)
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

    std::vector<EstimatorPortN*> StateSpaceModel1_Sensors;

    rclcpp::Publisher<fusion_estimator::msg::FusionEstimatorTest>::SharedPtr FETest_publisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr SMXFE_publisher, SMXFE_2D_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr go2_imu_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr go2_joint_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joystick_cmd_sub;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_sub;

    std::shared_ptr<SensorIMUAcc>     Sensor_IMUAcc;
    std::shared_ptr<SensorIMUMagGyro> Sensor_IMUMagGyro;
    std::shared_ptr<SensorLegsPos>    Sensor_LegsPos;
    std::shared_ptr<SensorLegsOri>    Sensor_LegsOri;

    fusion_estimator::msg::FusionEstimatorTest fusion_msg;

    std::string odom_frame_id, child_frame_id, child_frame_2d_id;
    bool imu_data_enable, leg_pos_enable, leg_ori_enable;
    double leg_ori_init_weight, leg_ori_time_wight;
    double position_correct[9];
    double orientation_correct[9];

    rcl_interfaces::msg::SetParametersResult Modify_Par_Fun(
    const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "Sensor_IMUMagGyro->SensorQuaternion is corrected.";

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
        rclcpp::Time CurrentTime = ros_clock.now();
        double CurrentTimestamp =  CurrentTime.seconds();

        double LatestMessage[3][100]={0};
        static double LastMessage[3][100]={0};

        fusion_msg.stamp = this->get_clock()->now();

        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        LatestMessage[0][3*0+2] = msg->linear_acceleration.x;
        LatestMessage[0][3*1+2] = msg->linear_acceleration.y;
        LatestMessage[0][3*2+2] = msg->linear_acceleration.z;

        LatestMessage[1][3*0] = roll + orientation_correct[0];
        LatestMessage[1][3*1] = pitch + orientation_correct[3];
        LatestMessage[1][3*2] = yaw  + orientation_correct[6];
        LatestMessage[1][3*0+1] = msg->angular_velocity.x;
        LatestMessage[1][3*1+1] = msg->angular_velocity.y;
        LatestMessage[1][3*2+1] = msg->angular_velocity.z;
        
        for(int i = 0; i < 9; i++)
        {
            if(LastMessage[0][i] != LatestMessage[0][i])
                break;
            if(LastMessage[1][i] != LatestMessage[1][i])
                break;
            if(i==8)
                return;  //数据未变则视为重复发送，舍弃数据
        }
        for(int j = 0; j < 9; j++)
        {
            LastMessage[0][j] = LatestMessage[0][j];
            LastMessage[1][j] = LatestMessage[1][j];
        }

        Sensor_IMUAcc->SensorDataHandle(LatestMessage[0], CurrentTimestamp);
        Sensor_IMUMagGyro->SensorDataHandle(LatestMessage[1], CurrentTimestamp);

        for(int i=0; i<9; i++){
            fusion_msg.estimated_xyz[i] = StateSpaceModel1_Sensors[0]->EstimatedState[i] + position_correct[i];
            fusion_msg.estimated_rpy[i] = StateSpaceModel1_Sensors[1]->EstimatedState[i];
        }

        Msg_Publish();
    }

    void joint_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        rclcpp::Clock ros_clock(RCL_SYSTEM_TIME);
        rclcpp::Time CurrentTime = ros_clock.now();
        double CurrentTimestamp =  CurrentTime.seconds();

        double LatestMessage[3][100]={0};
        static double LastMessage[3][100]={0};

        fusion_msg.stamp = this->get_clock()->now();

        const auto& arr = msg->data;
        if (arr.size() < 28) return;
        /* 数据布置：
        data[ 0..11] : 12× 关节位置  (q)
        data[12..23] : 12× 关节速度  (dq)
        data[24..27] : 4 × 足端力    (foot_force)
        */

        for(int LegNumber = 0; LegNumber<4; LegNumber++)
        {
            for(int i = 0; i < 3; i++)
            {
                LatestMessage[2][LegNumber*3+i] = arr[LegNumber*3+i];
                LatestMessage[2][12+LegNumber*3+i] = arr[12+LegNumber*3+i];
            }
            LatestMessage[2][24 + LegNumber] = arr[24 + LegNumber];
        }

        for(int i = 0; i < 28; i++)
        {
            if(LastMessage[2][i] != LatestMessage[2][i])
                break;
            if(i==27)
                return;  //数据未变则视为重复发送，舍弃数据
        }
        for(int j = 0; j < 28; j++)
        {
            LastMessage[2][j] = LatestMessage[2][j];
        }

        if(leg_pos_enable)
        {
            Sensor_LegsPos->SensorDataHandle(LatestMessage[2], CurrentTimestamp);
            
            for(int i=0; i<9; i++){
                fusion_msg.estimated_xyz[i] = StateSpaceModel1_Sensors[0]->EstimatedState[i] + position_correct[i];
            }

            for(int i=0; i<4; i++){
                for(int j=0; j<3; j++){
                    fusion_msg.feet_based_position[3 * i + j] = StateSpaceModel1_Sensors[0]->Double_Par[48 + 6 * i + j];
                    fusion_msg.feet_based_velocity[3 * i + j] = StateSpaceModel1_Sensors[0]->Double_Par[48 + 6 * i + j + 3];
                }
            }
            
            for(int LegNumber=0; LegNumber<4; LegNumber++){
                for(int i=0; i<3; i++){
                    fusion_msg.data_check_a[3 * LegNumber + i] = StateSpaceModel1_Sensors[0]->Double_Par[6 * LegNumber + i];      //身体坐标系的足身相对位置
                    fusion_msg.data_check_b[3 * LegNumber + i] = StateSpaceModel1_Sensors[0]->Double_Par[24 + 6 * LegNumber + i]; //世界坐标系的足身相对位置
                }
            }
        }
        if(leg_ori_enable)
        {
            double Last_Yaw = StateSpaceModel1_Sensors[1]->EstimatedState[6] - orientation_correct[6];

            StateSpaceModel1_Sensors[1]->Double_Par[97] = leg_ori_init_weight;
            StateSpaceModel1_Sensors[1]->Double_Par[98] = leg_ori_time_wight;
            Sensor_LegsOri->SensorDataHandle(LatestMessage[2], CurrentTimestamp);

            orientation_correct[6] = StateSpaceModel1_Sensors[1]->Double_Par[99] - Last_Yaw;
            if(!imu_data_enable)
                StateSpaceModel1_Sensors[1]->EstimatedState[6] = StateSpaceModel1_Sensors[1]->Double_Par[99];

            for(int i=0; i<9; i++){
                fusion_msg.estimated_rpy[i] = StateSpaceModel1_Sensors[1]->EstimatedState[i];
            }
        }

        Msg_Publish();
    }

    void Msg_Publish()
    {
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
            for(int i=0; i<4; i++)
            {
                Sensor_LegsPos->FootfallPositionRecordIsInitiated[i] = 0;
                Sensor_LegsPos->FootWasOnGround[i] = 0;
                Sensor_LegsPos->FootIsOnGround[i] = 0;
                Sensor_LegsPos->FootLanding[i] = 0;
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Former Px:%.3lf,Py:%.3lf,Pz:%.3lf,Yaw:%.3lf,Cx:%.3lf,Cy:%.3lf,Cz:%.3lf,Cyaw:%.3lf",StateSpaceModel1_Sensors[0]->EstimatedState[0],StateSpaceModel1_Sensors[0]->EstimatedState[3],StateSpaceModel1_Sensors[0]->EstimatedState[6],StateSpaceModel1_Sensors[1]->EstimatedState[6],position_correct[0],position_correct[3],position_correct[6],orientation_correct[6]);

            position_correct[0] = -StateSpaceModel1_Sensors[0]->EstimatedState[0];
            position_correct[3] = -StateSpaceModel1_Sensors[0]->EstimatedState[3];
            position_correct[6] = 0.34 -StateSpaceModel1_Sensors[0]->EstimatedState[6];
            orientation_correct[6] = -StateSpaceModel1_Sensors[1]->EstimatedState[6] + orientation_correct[6];

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received Estimator Position and Yaw Reset Command");
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
        "--params-file", "/home/unitree/ros2_ws/LeggedRobot/src/Ros2Go2Estimator/config.yaml"
    });

  auto node = std::make_shared<FusionEstimatorNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}