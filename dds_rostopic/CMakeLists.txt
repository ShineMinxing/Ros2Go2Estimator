cmake_minimum_required(VERSION 3.5)
project(dds_rostopic)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_sdk2 REQUIRED)

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
)

ament_target_dependencies(dds_rostopic_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  dds_rostopic_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()