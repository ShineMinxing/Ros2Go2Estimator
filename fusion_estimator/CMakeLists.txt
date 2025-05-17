cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(fusion_estimator)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(urdfdom REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# —— 后备实现 rosidl_get_typesupport_target —— 
if(NOT COMMAND rosidl_get_typesupport_target)
  function(rosidl_get_typesupport_target var generate_interfaces_target typesupport_name)
    # 检查基础目标是否存在
    if(NOT TARGET ${generate_interfaces_target})
      message(FATAL_ERROR
        "${generate_interfaces_target} is not a CMake target. "
        "Did you run rosidl_generate_interfaces() on the correct target?")
    endif()
    # typesupport 目标名就是 <pkg>__<typesupport_name>
    set(output_target "${generate_interfaces_target}__${typesupport_name}")
    if(NOT TARGET ${output_target})
      message(FATAL_ERROR
        "Cannot find typesupport target: ${output_target}")
    endif()
    # 返回给调用者
    set(${var} "${output_target}" PARENT_SCOPE)
  endfunction()
endif()

# 生成消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FusionEstimatorTest.msg"
  DEPENDENCIES builtin_interfaces
)

# 获取 src/ 下的所有 .cpp 和 .c 源文件
file(GLOB_RECURSE ESTIMATOR_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
)

# 获取 C++ 类型支持目标
rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

# 创建可执行文件
add_executable(fusion_estimator_node ${ESTIMATOR_SOURCES})

# 包含目录
target_include_directories(fusion_estimator_node PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${urdfdom_headers_INCLUDE_DIRS}
  ${urdfdom_INCLUDE_DIRS}
)

# 声明依赖
ament_target_dependencies(fusion_estimator_node
  rclcpp
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

# 链接库
target_link_libraries(fusion_estimator_node
  ${ament_LIBRARIES}
  ${cpp_typesupport_target}
  ${urdfdom_LIBRARIES}
)

# 安装目标
install(TARGETS fusion_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(rosidl_default_runtime builtin_interfaces)
ament_package()
