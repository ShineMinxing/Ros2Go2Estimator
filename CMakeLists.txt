cmake_minimum_required(VERSION 3.5)
project(fusion_estimator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

file(GLOB_RECURSE ESTIMATOR_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/FusionEstimator/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/FusionEstimator/*.c"
)

add_executable(fusion_estimator_node
  fusion_estimator_node.cpp
  ${ESTIMATOR_SOURCES}
)

target_include_directories(fusion_estimator_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/FusionEstimator
)

ament_target_dependencies(fusion_estimator_node
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS fusion_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()