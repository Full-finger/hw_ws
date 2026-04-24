cmake_minimum_required(VERSION 3.8)
project(yaml_reader_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(yaml_reader_node src/yaml_reader_node.cpp)
ament_target_dependencies(yaml_reader_node rclcpp)

install(TARGETS
  yaml_reader_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 config 目录
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# 安装 launch 目录
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
