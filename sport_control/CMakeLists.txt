cmake_minimum_required(VERSION 3.5)
project(sport_control)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(sport_control_node src/sport_control_node.cpp)

target_include_directories(sport_control_node
  PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${unitree_sdk2_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(sport_control_node
  ${ament_LIBRARIES}
  unitree_sdk2
  ddsc ddscxx
)

ament_target_dependencies(sport_control_node
  rclcpp
  sensor_msgs
  rclcpp_action
  nav2_msgs
  geometry_msgs
  tf2
  tf2_ros
)

install(TARGETS
  sport_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()