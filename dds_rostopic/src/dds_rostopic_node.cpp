#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

// Unitree DDS 库头文件
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>

// C++ 标准库
#include <string>
#include <memory>

// 订阅的 DDS 话题名称
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

    // 3. 创建 DDS 订阅者
    //   使用回调函数 CbPointCloud 来处理订阅到的数据
    subscriber_ = std::make_unique<unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>>(TOPIC_CLOUD);
    subscriber_->InitChannel(std::bind(&DDSToRosNode::CbPointCloud, this, std::placeholders::_1));
  }

private:
  // 回调函数
  void CbPointCloud(const void* message)
  {
    // 将 void* 强转为我们需要的类型
    const sensor_msgs::msg::dds_::PointCloud2_* cloud_msg_dds
       = static_cast<const sensor_msgs::msg::dds_::PointCloud2_*>(message);

    // 构造一个 ROS2 的 sensor_msgs::msg::PointCloud2
    sensor_msgs::msg::PointCloud2 cloud_ros;
    // 1. header
    cloud_ros.header.stamp.sec = cloud_msg_dds->header().stamp().sec();
    cloud_ros.header.stamp.nanosec = cloud_msg_dds->header().stamp().nanosec();
    cloud_ros.header.frame_id = cloud_msg_dds->header().frame_id();

    // 2. height / width
    cloud_ros.height = cloud_msg_dds->height();
    cloud_ros.width  = cloud_msg_dds->width();

    // 3. fields
    cloud_ros.fields.resize(cloud_msg_dds->fields().size());
    for (size_t i = 0; i < cloud_msg_dds->fields().size(); i++)
    {
      auto& field_dds = cloud_msg_dds->fields()[i];
      auto& field_ros = cloud_ros.fields[i];
      field_ros.name = field_dds.name();
      field_ros.offset = field_dds.offset();
      field_ros.datatype = field_dds.datatype();
      field_ros.count = field_dds.count();
    }

    // 4. is_bigendian
    cloud_ros.is_bigendian = cloud_msg_dds->is_bigendian();

    // 5. point_step / row_step
    cloud_ros.point_step = cloud_msg_dds->point_step();
    cloud_ros.row_step   = cloud_msg_dds->row_step();

    // 6. data
    //   DDS 中 data 是 sequence<octet>
    //   ROS 中 data 是 std::vector<uint8_t>
    size_t data_size = cloud_msg_dds->data().size();
    cloud_ros.data.resize(data_size);
    memcpy(cloud_ros.data.data(), cloud_msg_dds->data().data(), data_size);

    // 7. is_dense
    cloud_ros.is_dense = cloud_msg_dds->is_dense();

    // 打印调试信息（可选）
    RCLCPP_DEBUG(this->get_logger(), "Received DDS cloud: width=%d, height=%d, frame_id=%s",
                 cloud_ros.width, cloud_ros.height, cloud_ros.header.frame_id.c_str());

    // 发布到 ROS 话题
    pub_cloud_->publish(cloud_ros);
  }

private:
  // ROS2 Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;

  // Unitree DDS Subscriber
  std::unique_ptr<unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>> subscriber_;
};

// main 函数
int main(int argc, char** argv)
{
  // 初始化 ROS2
  rclcpp::init(argc, argv);
  // 创建节点并运行
  rclcpp::spin(std::make_shared<DDSToRosNode>());
  rclcpp::shutdown();
  return 0;
}
