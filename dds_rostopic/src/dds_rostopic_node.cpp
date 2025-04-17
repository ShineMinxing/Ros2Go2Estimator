#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>  // 导入图像消息类型

// Unitree DDS 库头文件
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>

// C++ 标准库
#include <string>
#include <memory>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <opencv2/videoio.hpp>

#define TOPIC_CLOUD "rt/utlidar/cloud"

// 建立一个封装类，从 rclcpp::Node 继承
class DDSToRosNode : public rclcpp::Node
{
public:
  DDSToRosNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
  : Node("dds_rostopic_node", options)
  {
    // 声明一个 ROS 参数，用于指定网络接口
    this->declare_parameter<std::string>("network_interface", "enx00e04c8d0eff");
    // 获取参数
    std::string network_if = this->get_parameter("network_interface").as_string();
    RCLCPP_INFO(this->get_logger(), "Using network interface: %s", network_if.c_str());

    // 1. 初始化 Unitree DDS
    unitree::robot::ChannelFactory::Instance()->Init(0, network_if.c_str());

    // 2. 创建 ROS2 Publisher（发布标准的 PointCloud2）
    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("SMXFE/Cloud", 10);
    
    // 3. 创建 ROS2 Publisher（发布视频流）
    pub_camera_ = this->create_publisher<sensor_msgs::msg::Image>("SMXFE/Go2Camera", 10);

    // 4. 创建 DDS 订阅者
    subscriber_ = std::make_unique<unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>>(TOPIC_CLOUD);
    subscriber_->InitChannel(std::bind(&DDSToRosNode::CbPointCloud, this, std::placeholders::_1));

    // 5. 创建 OpenCV VideoCapture 对象用于拉取视频流
    std::string pipeline = "udpsrc address=230.1.1.1 port=1720 multicast-iface=enx00e04c8d0eff ! "
                           "application/x-rtp, media=video, encoding-name=H264 ! rtph264depay ! "
                           "h264parse ! avdec_h264 ! videoconvert ! video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1 sync=false";
    cap_ = std::make_shared<cv::VideoCapture>(pipeline, cv::CAP_GSTREAMER);

    if (!cap_->isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open video stream.");
      rclcpp::shutdown();
    }

    // 定时器，定期发布视频帧
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(30), std::bind(&DDSToRosNode::timer_callback, this));
  }

private:
  // 回调函数发布 PointCloud2 数据
  void CbPointCloud(const void* message)
  {
    const sensor_msgs::msg::dds_::PointCloud2_* cloud_msg_dds
       = static_cast<const sensor_msgs::msg::dds_::PointCloud2_*>(message);

    sensor_msgs::msg::PointCloud2 cloud_ros;
    cloud_ros.header.stamp.sec = cloud_msg_dds->header().stamp().sec();
    cloud_ros.header.stamp.nanosec = cloud_msg_dds->header().stamp().nanosec();
    cloud_ros.header.frame_id = cloud_msg_dds->header().frame_id();
    cloud_ros.height = cloud_msg_dds->height();
    cloud_ros.width  = cloud_msg_dds->width();
    cloud_ros.fields.resize(cloud_msg_dds->fields().size());
    for (size_t i = 0; i < cloud_msg_dds->fields().size(); i++) {
      auto& field_dds = cloud_msg_dds->fields()[i];
      auto& field_ros = cloud_ros.fields[i];
      field_ros.name = field_dds.name();
      field_ros.offset = field_dds.offset();
      field_ros.datatype = field_dds.datatype();
      field_ros.count = field_dds.count();
    }
    cloud_ros.is_bigendian = cloud_msg_dds->is_bigendian();
    cloud_ros.point_step = cloud_msg_dds->point_step();
    cloud_ros.row_step   = cloud_msg_dds->row_step();
    size_t data_size = cloud_msg_dds->data().size();
    cloud_ros.data.resize(data_size);
    memcpy(cloud_ros.data.data(), cloud_msg_dds->data().data(), data_size);
    cloud_ros.is_dense = cloud_msg_dds->is_dense();
    pub_cloud_->publish(cloud_ros);
  }

  // 定时器回调函数用于定期发布视频帧
  void timer_callback() {
    cv::Mat frame;
    if (cap_->read(frame)) {
      // 将 OpenCV 的 Mat 转换为 ROS2 图像消息
      auto msg = sensor_msgs::msg::Image();
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "camera";
      msg.height = frame.rows;
      msg.width = frame.cols;
      msg.encoding = "bgr8";
      msg.is_bigendian = false;
      msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
      msg.data.assign(frame.datastart, frame.dataend);

      // 发布视频帧到 ROS2 话题
      pub_camera_->publish(msg);
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to capture frame.");
    }
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_camera_;  // 发布视频流
  std::shared_ptr<cv::VideoCapture> cap_;  // OpenCV VideoCapture 用于拉流
  std::shared_ptr<unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>> subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// main 函数
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DDSToRosNode>());
  rclcpp::shutdown();
  return 0;
}
