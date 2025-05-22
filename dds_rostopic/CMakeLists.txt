cmake_minimum_required(VERSION 3.5)
project(dds_rostopic)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(compressed_image_transport REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
include_directories(${GSTREAMER_INCLUDE_DIRS})
link_directories(${GSTREAMER_LIBRARY_DIRS})

add_executable(dds_rostopic_node src/dds_rostopic_node.cpp)

target_include_directories(dds_rostopic_node
  PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${unitree_sdk2_INCLUDE_DIRS} 
)

target_link_libraries(dds_rostopic_node
  ${ament_LIBRARIES}
  unitree_sdk2
  ddsc ddscxx
  tf2_ros::tf2_ros
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(dds_rostopic_node
  rclcpp
  sensor_msgs
  tf2_geometry_msgs
  unitree_sdk2
  OpenCV
  cv_bridge
  image_transport
  compressed_image_transport
)

install(TARGETS
  dds_rostopic_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()