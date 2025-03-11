cmake_minimum_required(VERSION 3.5)
project(joystick_control)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_sdk2 REQUIRED)

add_executable(joystick_control_node src/joystick_control_node.cpp)

target_include_directories(joystick_control_node
  PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${unitree_sdk2_INCLUDE_DIRS} 
)

target_link_libraries(joystick_control_node
  ${ament_LIBRARIES}
  unitree_sdk2
  ddsc ddscxx
)

ament_target_dependencies(joystick_control_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  joystick_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()