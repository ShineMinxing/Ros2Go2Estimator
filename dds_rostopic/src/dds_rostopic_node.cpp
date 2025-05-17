#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


// Unitree DDS
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>

// OpenCV & GStreamer
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

class DDSToRosNode : public rclcpp::Node
{
public:
  DDSToRosNode(const rclcpp::NodeOptions &options)
  : Node("dds_rostopic_node", options)
  {
    // 读取参数，如果 YAML 里没配置，就用第二个参数里的默认值
    std::string network_if;
    this->get_parameter_or<std::string>(
      "network_interface", network_if, std::string("enxf8e43b808e06"));

    std::string pub_joint_topic;
    this->get_parameter_or<std::string>(
      "pub_joint_topic", pub_joint_topic, std::string("NoYamlRead/Go2Joint"));

    std::string pub_imu_topic;
    this->get_parameter_or<std::string>(
      "pub_imu_topic", pub_imu_topic, std::string("NoYamlRead/Go2IMU"));

    std::string pub_cloud_topic;
    this->get_parameter_or<std::string>(
      "pub_cloud_topic", pub_cloud_topic, std::string("NoYamlRead/Go2Lidar"));

    std::string pub_camera_topic;
    this->get_parameter_or<std::string>(
      "pub_camera_topic", pub_camera_topic, std::string("NoYamlRead/Go2Camera"));

    std::string dds_lowstate_topic;
      this->get_parameter_or("dds_lowstate_topic", dds_lowstate_topic, std::string("rt/lowstate"));

    std::string dds_pointcloud_topic;
    this->get_parameter_or<std::string>(
      "dds_pointcloud_topic", dds_pointcloud_topic, std::string("rt/utlidar/cloud"));

    std::string gst_pipeline;
    this->get_parameter_or<std::string>(
      "gst_pipeline", gst_pipeline,
      std::string(
        "udpsrc address=230.1.1.1 port=1720 multicast-iface=enxf8e43b808e06 ! "
        "application/x-rtp,media=video,encoding-name=H264 ! rtph264depay ! "
        "h264parse ! nvv4l2decoder enable-max-performance=1 ! "
        "nvvidconv output-buffers=1 ! "
        "video/x-raw,format=BGRx,width=1280,height=720 ! "
        "videoconvert ! video/x-raw,format=BGR ! appsink drop=1 sync=false"
      )
    );

    // 1) 初始化 DDS
    unitree::robot::ChannelFactory::Instance()->Init(0, network_if.c_str());

    // 2) ROS2 发布者
    pub_cloud_  = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_cloud_topic,  10);
    pub_camera_ = this->create_publisher<sensor_msgs::msg::Image>(pub_camera_topic, 10);
    pub_imu_    = this->create_publisher<sensor_msgs::msg::Imu>(pub_imu_topic, 10);
    pub_joint_  = this->create_publisher<std_msgs::msg::Float64MultiArray>(pub_joint_topic, 10);


    // 3) DDS 订阅
    Pointcloud_subscriber = std::make_unique<
      unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>>(dds_pointcloud_topic);
    Pointcloud_subscriber->InitChannel(
      std::bind(&DDSToRosNode::CbPointCloud, this, std::placeholders::_1));

    Lowstate_subscriber = std::make_unique<
      unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(dds_lowstate_topic);
    Lowstate_subscriber->InitChannel(
      std::bind(&DDSToRosNode::LowStateCallback, this, std::placeholders::_1), 1);

    // 4) OpenCV 拉流
    cap_ = std::make_shared<cv::VideoCapture>(gst_pipeline, cv::CAP_GSTREAMER);
    if (!cap_->isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开视频流");
      rclcpp::shutdown();
      return;
    }

    // 5) 定时发布视频
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(30),
      std::bind(&DDSToRosNode::timer_callback, this)
    );
  }

private:
  void CbPointCloud(const void* message)
  {
    auto dds_msg = static_cast<const sensor_msgs::msg::dds_::PointCloud2_*>(message);
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp.sec     = dds_msg->header().stamp().sec();
    cloud.header.stamp.nanosec = dds_msg->header().stamp().nanosec();
    cloud.header.frame_id      = dds_msg->header().frame_id();
    cloud.height = dds_msg->height();
    cloud.width  = dds_msg->width();
    cloud.fields.resize(dds_msg->fields().size());
    for (size_t i = 0; i < dds_msg->fields().size(); i++) {
      auto &f_dds = dds_msg->fields()[i];
      auto &f_ros = cloud.fields[i];
      f_ros.name     = f_dds.name();
      f_ros.offset   = f_dds.offset();
      f_ros.datatype = f_dds.datatype();
      f_ros.count    = f_dds.count();
    }
    cloud.is_bigendian = dds_msg->is_bigendian();
    cloud.point_step   = dds_msg->point_step();
    cloud.row_step     = dds_msg->row_step();
    cloud.data.resize(dds_msg->data().size());
    memcpy(cloud.data.data(), dds_msg->data().data(), cloud.data.size());
    cloud.is_dense = dds_msg->is_dense();
    pub_cloud_->publish(cloud);
  }

  void LowStateCallback(const void* message)
  {
    const auto& low_state = *static_cast<const unitree_go::msg::dds_::LowState_*>(message);
    rclcpp::Time stamp = this->get_clock()->now();

    /* ---------- ① IMU -> sensor_msgs/Imu ---------- */
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp    = stamp;
    imu_msg.header.frame_id = "base_imu";

    /* RPY → Quaternion */
    tf2::Quaternion q_tf;
    double roll  =  low_state.imu_state().rpy()[0];
    double pitch =  low_state.imu_state().rpy()[1];
    double yaw   =  low_state.imu_state().rpy()[2];
    q_tf.setRPY(roll, pitch, yaw);
    imu_msg.orientation             = tf2::toMsg(q_tf);
    imu_msg.orientation_covariance  = {0.0};

    /* 角速度（rad/s） */
    imu_msg.angular_velocity.x = low_state.imu_state().gyroscope()[0];
    imu_msg.angular_velocity.y = low_state.imu_state().gyroscope()[1];
    imu_msg.angular_velocity.z = low_state.imu_state().gyroscope()[2];

    /* 线加速度（m/s²） */
    imu_msg.linear_acceleration.x = low_state.imu_state().accelerometer()[0];
    imu_msg.linear_acceleration.y = low_state.imu_state().accelerometer()[1];
    imu_msg.linear_acceleration.z = low_state.imu_state().accelerometer()[2];

    pub_imu_->publish(imu_msg);

    /* ---------- ② 关节/足端 -> Float64MultiArray ---------- */
    std_msgs::msg::Float64MultiArray joint_msg;
    /* 数据布置：
      data[ 0..11] : 12× 关节位置  (q)
      data[12..23] : 12× 关节速度  (dq)
      data[24..27] : 4 × 足端力    (foot_force)
    */
    joint_msg.data.resize(28);
    // 电机位置 & 速度
    for (int leg = 0; leg < 4; ++leg) {
        for (int i = 0; i < 3; ++i) {
            int idx = leg * 3 + i;
            joint_msg.data[idx]       = low_state.motor_state()[idx].q();      // 位置
            joint_msg.data[12 + idx]  = low_state.motor_state()[idx].dq();     // 速度
        }
        joint_msg.data[24 + leg] = low_state.foot_force()[leg];                // 足端力
    }
    pub_joint_->publish(joint_msg);
  }

  void timer_callback()
  {
    cv::Mat frame;
    if (cap_->read(frame)) {
      sensor_msgs::msg::Image img;
      img.header.stamp = this->get_clock()->now();
      img.header.frame_id = "camera";
      img.height = frame.rows;
      img.width  = frame.cols;
      img.encoding     = "bgr8";
      img.is_bigendian = false;
      img.step         = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
      img.data.assign(frame.datastart, frame.dataend);
      pub_camera_->publish(img);
    } else {
      RCLCPP_WARN(this->get_logger(), "采集视频帧失败");
    }
  }


  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_camera_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr   pub_imu_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_joint_;
  std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>> Lowstate_subscriber;
  std::unique_ptr<unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>> Pointcloud_subscriber;
  std::shared_ptr<cv::VideoCapture> cap_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)
    .arguments({
      "--ros-args",
      "--params-file", "/home/unitree/ros2_ws/LeggedRobot/src/Ros2Go2Estimator/config.yaml"
    });

  auto node = std::make_shared<DDSToRosNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
