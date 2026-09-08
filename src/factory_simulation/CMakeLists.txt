cmake_minimum_required(VERSION 3.8)

project(factory_simulation)

# 查找ROS 2的ament_cmake构建系统
find_package(ament_cmake REQUIRED)

# 安装Launch文件
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装Gazebo World文件
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# 安装Gazebo模型
install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

# 安装辅助脚本
install(
  PROGRAMS
    scripts/prepare_factory_world.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
