cmake_minimum_required(VERSION 3.5)
project(fusion_estimator)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 生成消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FusionEstimatorTest.msg"
  DEPENDENCIES builtin_interfaces
)

# 获取 src/Estimator 目录下的所有 .cpp
file(GLOB_RECURSE ESTIMATOR_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
)

# 获取C++类型支持目标
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 创建可执行文件
add_executable(fusion_estimator_node ${ESTIMATOR_SOURCES})

# 包含目录
target_include_directories(fusion_estimator_node PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${unitree_sdk2_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# 声明依赖
ament_target_dependencies(fusion_estimator_node
  rclcpp
  sensor_msgs
)

# 链接库（关键修改）
target_link_libraries(fusion_estimator_node
  ${ament_LIBRARIES}
  unitree_sdk2
  ddsc ddscxx
  ${cpp_typesupport_target}
)

# 安装目标
install(TARGETS fusion_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(rosidl_default_runtime builtin_interfaces)
ament_package()