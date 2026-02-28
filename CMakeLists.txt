cmake_minimum_required(VERSION 3.8)
project(server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ar_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

# Tell compiler where to find your headers
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include  # <- points to robotlibpc folder
)

# Add executable for PID server node
add_executable(sick_server_node
  src/servers/sick_pid.cpp
  include/robotlibpc/cpp/controller/pid.cpp 
)

add_executable(odom_server_node
  src/servers/odom_pid.cpp
  include/robotlibpc/cpp/controller/pid.cpp
)

add_executable(align_server_node
  src/servers/align.cpp
  include/robotlibpc/cpp/controller/pid.cpp
)

add_executable(test_yaw_node
  src/TestingClient/test_yaw.cpp        # your ROS 2 node file # if your node uses PID helper code
)

ament_target_dependencies(odom_server_node
  rclcpp
  rclcpp_action
  std_msgs
  ar_interfaces
  geometry_msgs
)

ament_target_dependencies(sick_server_node
  rclcpp
  rclcpp_action
  std_msgs
  ar_interfaces
  geometry_msgs
)

ament_target_dependencies(align_server_node
  rclcpp
  rclcpp_action
  std_msgs
  ar_interfaces
  geometry_msgs
)

ament_target_dependencies(test_yaw_node
  rclcpp
  std_msgs
)

install(TARGETS
  sick_server_node
  odom_server_node
  align_server_node
  test_yaw_node
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
