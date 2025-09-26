cmake_minimum_required(VERSION 3.8)
project(fc_comm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FCCommand.msg"
  "msg/FCStatus.msg"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 安装Python脚本
install(PROGRAMS
  src/fc_comm_node.py
  src/guidance_simulator.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装消息定义
install(DIRECTORY
  msg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
