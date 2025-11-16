#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <iomanip>

#include <urdf_parser/urdf_parser.h>
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
        /* ────────────── ① 读取所有可能的参数 ────────────── */
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
        std::string odom_frame;
        this->get_parameter_or("odom_frame", odom_frame, std::string("odom"));
        std::string base_frame;
        this->get_parameter_or("base_frame", base_frame, std::string("base_link"));
        std::string base_frame_2d;
        this->get_parameter_or("base_frame_2d", base_frame_2d, std::string("base_link_2D"));
        std::string urdf_path_cfg;
        this->get_parameter_or("urdf_file", urdf_path_cfg, std::string("cfg/go2_description.urdf"));
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


        go2_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            sub_imu_topic, 10,
            std::bind(&FusionEstimatorNode::imu_callback, this, std::placeholders::_1));
        go2_joint_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            sub_joint_topic, 10,
            std::bind(&FusionEstimatorNode::joint_callback, this, std::placeholders::_1));
        joystick_cmd_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            sub_mode_topic, 10,
            std::bind(&FusionEstimatorNode::joystick_cmd_callback, this, std::placeholders::_1));

        FETest_publisher = this->create_publisher<fusion_estimator::msg::FusionEstimatorTest>(
        pub_estimation_topic, 10);
        SMXFE_publisher   = this->create_publisher<nav_msgs::msg::Odometry>(pub_odom_topic, 10);
        SMXFE_2D_publisher= this->create_publisher<nav_msgs::msg::Odometry>(pub_odom_2d_topic, 10);

        odom_frame_id_     = odom_frame;
        child_frame_id_    = base_frame;
        child_frame_id_2d_ = base_frame_2d;

        /* ────────────── ③ 创建融合器对象 ────────────── */
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

        /* 可在线调三轴角度补偿 */
        ParameterCorrectCallback = this->add_on_set_parameters_callback(
        std::bind(&FusionEstimatorNode::Modify_Par_Fun, this, std::placeholders::_1));

        /* ────────────── ④ 读取 URDF 并填充腿部参数 ────────────── */
        urdf_path_cfg_ = urdf_path_cfg;   // 保存到成员变量，ObtainParameter() 会用
        RCLCPP_INFO(get_logger(), "FusionEstimatorNode started, URDF=%s", urdf_path_cfg_.c_str());

        
        for (int i = 0; i < 9; ++i) {
            position_correct[i] = 0;
            orientation_correct[i] = 0;
        }
    }

    /* —— 在 main() 里会显式调用 —— */
    void ObtainParameter();

private:

    std::vector<EstimatorPortN*> StateSpaceModel1_Sensors;

    rclcpp::Publisher<fusion_estimator::msg::FusionEstimatorTest>::SharedPtr FETest_publisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr SMXFE_publisher, SMXFE_2D_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr go2_imu_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr go2_joint_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joystick_cmd_sub;

    std::shared_ptr<SensorIMUAcc>     Sensor_IMUAcc;
    std::shared_ptr<SensorIMUMagGyro> Sensor_IMUMagGyro;
    std::shared_ptr<SensorLegsPos>    Sensor_LegsPos;
    std::shared_ptr<SensorLegsOri>    Sensor_LegsOri;

    fusion_estimator::msg::FusionEstimatorTest fusion_msg;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr ParameterCorrectCallback;

    std::string odom_frame_id_, child_frame_id_, child_frame_id_2d_;
    std::string urdf_path_cfg_;
    bool leg_pos_enable, leg_ori_enable;
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
            if (param.get_name() == "Modify_Par_1")
                orientation_correct[0] = param.as_double() * M_PI / 180.0;
            if (param.get_name() == "Modify_Par_2")
                orientation_correct[1] = param.as_double() * M_PI / 180.0;
            if (param.get_name() == "Modify_Par_3")
                orientation_correct[2] = param.as_double() * M_PI / 180.0;
        }

        Eigen::AngleAxisd rollAngle(orientation_correct[0], Eigen::Vector3d::UnitZ());  // 绕 X 轴旋转
        Eigen::AngleAxisd pitchAngle(orientation_correct[1], Eigen::Vector3d::UnitY()); // 绕 Y 轴旋转
        Eigen::AngleAxisd yawAngle(orientation_correct[2], Eigen::Vector3d::UnitX());   // 绕 Z 轴旋转

        Sensor_IMUMagGyro->SensorQuaternion = yawAngle * pitchAngle * rollAngle;
        Sensor_IMUMagGyro->SensorQuaternionInv = Sensor_IMUMagGyro->SensorQuaternion.inverse();
        std::cout <<"Sensor_IMUMagGyro->SensorQuaternion: " << Sensor_IMUMagGyro->SensorQuaternion.coeffs().transpose() << std::endl;

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

        LatestMessage[1][3*0] = roll + orientation_correct[0];
        LatestMessage[1][3*1] = pitch + orientation_correct[3];
        LatestMessage[1][3*2] = yaw  + orientation_correct[6];
        LatestMessage[1][3*0+1] = msg->angular_velocity.x;
        LatestMessage[1][3*1+1] = msg->angular_velocity.y;
        LatestMessage[1][3*2+1] = msg->angular_velocity.z;

        // only fot test
        // LatestMessage[1][3*0] = orientation_correct[0];
        // LatestMessage[1][3*1] = orientation_correct[3];
        // LatestMessage[1][3*2] = orientation_correct[6];
        
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

        if(leg_pos_enable)
        {
            double Last_Yaw = StateSpaceModel1_Sensors[1]->EstimatedState[6] - orientation_correct[6];
            for(int i = 0; i < 28; i++)
            {
                if(LastMessage[2][i] != LatestMessage[2][i])
                {
                    Sensor_LegsPos->SensorDataHandle(LatestMessage[2], CurrentTimestamp);
                    if(leg_ori_enable){
                        StateSpaceModel1_Sensors[1]->Double_Par[97] = leg_ori_init_weight;
                        StateSpaceModel1_Sensors[1]->Double_Par[98] = leg_ori_time_wight;
                        Sensor_LegsOri->SensorDataHandle(LatestMessage[2], CurrentTimestamp);
                    }
                    for(int j = 0; j < 28; j++)
                    {
                        LastMessage[2][j] = LatestMessage[2][j];
                    }
                    break;
                }
            }

            if(leg_ori_enable)
                orientation_correct[6] = StateSpaceModel1_Sensors[1]->Double_Par[99] - Last_Yaw;
            
            for(int i=0; i<9; i++){
                fusion_msg.estimated_xyz[i] = StateSpaceModel1_Sensors[0]->EstimatedState[i] + position_correct[i];
                fusion_msg.estimated_rpy[i] = StateSpaceModel1_Sensors[1]->EstimatedState[i];
            }
            
            for(int LegNumber=0; LegNumber<4; LegNumber++){
                for(int i=0; i<3; i++){
                    fusion_msg.data_check_a[3 * LegNumber + i] = StateSpaceModel1_Sensors[0]->Double_Par[6 * LegNumber + i];
                    fusion_msg.data_check_b[3 * LegNumber + i] = StateSpaceModel1_Sensors[0]->Double_Par[6 * LegNumber + 3 + i];
                    fusion_msg.data_check_c[3 * LegNumber + i] = StateSpaceModel1_Sensors[0]->Double_Par[24 + 6 * LegNumber + i];
                    fusion_msg.data_check_d[3 * LegNumber + i] = StateSpaceModel1_Sensors[0]->Double_Par[24 + 6 * LegNumber + 3 + i];
                }
            }

            for(int i=0; i<4; i++){
                for(int j=0; j<3; j++){
                    fusion_msg.feet_based_position[3 * i + j] = StateSpaceModel1_Sensors[0]->Double_Par[48 + 6 * i + j];
                    fusion_msg.feet_based_velocity[3 * i + j] = StateSpaceModel1_Sensors[0]->Double_Par[48 + 6 * i + j + 3];
                }
            }


            for(int i=0; i<48; i++)
                fusion_msg.others[i] = StateSpaceModel1_Sensors[1]->Double_Par[i];
        }

        Msg_Publish();
    }

    void Msg_Publish()
    {
        FETest_publisher->publish(fusion_msg);

        // 构造标准 3D odometry 消息，并发布
        nav_msgs::msg::Odometry SMXFE_odom;
        SMXFE_odom.header.stamp = fusion_msg.stamp;
        SMXFE_odom.header.frame_id = odom_frame_id_;
        SMXFE_odom.child_frame_id = child_frame_id_;

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
        SMXFE_odom_2D.header.frame_id = odom_frame_id_;
        SMXFE_odom_2D.child_frame_id = child_frame_id_2d_;

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

void FusionEstimatorNode::ObtainParameter()
{
    // 获得机器狗运动学参数
    Sensor_LegsPos->KinematicParams  << 
    0.1934, -0.0465,  0.000,   0.0, -0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022,
    0.1934,  0.0465,  0.000,   0.0,  0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022,
    -0.1934, -0.0465,  0.000,   0.0, -0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022,
    -0.1934,  0.0465,  0.000,   0.0,  0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022;
    std::filesystem::path current_file(__FILE__);
    std::filesystem::path package_dir = current_file.parent_path().parent_path();
    std::filesystem::path urdf_path = package_dir / urdf_path_cfg_;
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
    std::vector<std::string> legs = {"FR", "FL", "RR", "RL"};
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
                std::cout << Sensor_LegsPos->KinematicParams.row(i).segment(jm.col, 3) << std::endl;
            }
            else
            {
                urdf::Vector3 pos = joint->parent_to_joint_origin_transform.position;
                Sensor_LegsPos->KinematicParams(i, jm.col)     = pos.x;
                Sensor_LegsPos->KinematicParams(i, jm.col + 1) = pos.y;
                Sensor_LegsPos->KinematicParams(i, jm.col + 2) = pos.z;
                std::cout << "Obtained KinematicPar for " << jointName << ": ";
                std::cout << Sensor_LegsPos->KinematicParams.row(i).segment(jm.col, 3) << std::endl;
            }
        }
        // 更新该腿 foot 连杆 collision 的球半径（存储在列 12）
        std::string footLinkName = leg + "_foot";
        urdf::LinkConstSharedPtr footLink = model->getLink(footLinkName);
        if (!footLink)
        {
            std::cout << "未找到连杆: " << footLinkName << " (" << leg << ")，使用默认值: " << Sensor_LegsPos->KinematicParams(i, 12) << std::endl;
        }
        else
        {
            if (footLink->collision && footLink->collision->geometry &&
                footLink->collision->geometry->type == urdf::Geometry::SPHERE)
            {
                urdf::Sphere* sphere = dynamic_cast<urdf::Sphere*>(footLink->collision->geometry.get());
                if (sphere)
                {
                    Sensor_LegsPos->KinematicParams(i, 12) = sphere->radius;
                    std::cout << "Obtained KinematicPar for " << footLinkName << ": " << Sensor_LegsPos->KinematicParams(i, 12) << std::endl;
                }
            }
        }

        Sensor_LegsPos->Par_HipLength = std::sqrt(Sensor_LegsPos->KinematicParams(0, 3)*Sensor_LegsPos->KinematicParams(0, 3) + Sensor_LegsPos->KinematicParams(0, 4)*Sensor_LegsPos->KinematicParams(0, 4) + Sensor_LegsPos->KinematicParams(0, 5)*Sensor_LegsPos->KinematicParams(0, 5));
        Sensor_LegsPos->Par_ThighLength = std::sqrt(Sensor_LegsPos->KinematicParams(0, 6)*Sensor_LegsPos->KinematicParams(0, 6) + Sensor_LegsPos->KinematicParams(0, 7)*Sensor_LegsPos->KinematicParams(0, 7) + Sensor_LegsPos->KinematicParams(0, 8)*Sensor_LegsPos->KinematicParams(0, 8));
        Sensor_LegsPos->Par_CalfLength = std::sqrt(Sensor_LegsPos->KinematicParams(0, 9)*Sensor_LegsPos->KinematicParams(0, 9) + Sensor_LegsPos->KinematicParams(0, 10)*Sensor_LegsPos->KinematicParams(0, 10) + Sensor_LegsPos->KinematicParams(0, 11)*Sensor_LegsPos->KinematicParams(0, 11));
        Sensor_LegsPos->Par_FootLength = abs(Sensor_LegsPos->KinematicParams(0, 12));
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
  node->ObtainParameter();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}