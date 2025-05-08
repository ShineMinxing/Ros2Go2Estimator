#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>

// Unitree DDS
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
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
      "network_interface", network_if, std::string("br0"));

    std::string pub_cloud_topic;
    this->get_parameter_or<std::string>(
      "pub_cloud_topic", pub_cloud_topic, std::string("TEST/Go2Lidar"));

    std::string pub_camera_topic;
    this->get_parameter_or<std::string>(
      "pub_camera_topic", pub_camera_topic, std::string("TEST/Go2Camera"));

    std::string dds_topic_in;
    this->get_parameter_or<std::string>(
      "dds_topic", dds_topic_in, std::string("rt/utlidar/cloud"));

    std::string gst_pipeline;
    this->get_parameter_or<std::string>(
      "gst_pipeline", gst_pipeline,
      std::string(
        "udpsrc address=230.1.1.1 port=1720 multicast-iface=br0 ! "
        "application/x-rtp, media=video, encoding-name=H264 ! rtph264depay ! "
        "h264parse ! avdec_h264 ! videoconvert ! video/x-raw,width=1280,height=720,format=BGR ! "
        "appsink drop=1 sync=false"
      )
    );

    // 1) 初始化 DDS
    unitree::robot::ChannelFactory::Instance()->Init(0, network_if.c_str());

    // 2) ROS2 发布者
    pub_cloud_  = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_cloud_topic,  10);
    pub_camera_ = this->create_publisher<sensor_msgs::msg::Image>     (pub_camera_topic, 10);

    // 3) DDS 订阅
    subscriber_ = std::make_unique<
      unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>
    >(dds_topic_in);
    subscriber_->InitChannel(
      std::bind(&DDSToRosNode::CbPointCloud, this, std::placeholders::_1)
    );

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
  rclcpp::Publisher<sensor_msgs::msg::Image>     ::SharedPtr pub_camera_;
  std::unique_ptr<
    unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>
  > subscriber_;
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
      "--params-file", "/home/smx/ros2_ws/LeggedRobot/src/Ros2Go2Estimator/config.yaml"
    });

  auto node = std::make_shared<DDSToRosNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
