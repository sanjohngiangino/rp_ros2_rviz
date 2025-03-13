cmake_minimum_required(VERSION 3.8)
project(rp_ros2_rviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_executable(control_robot src/control_robot.cpp)
ament_target_dependencies(control_robot rclcpp OpenCV std_msgs geometry_msgs)

add_executable(world_node src/world_node.cpp src/world.cpp src/grid_map.cpp src/grid.cpp src/canvas.cpp src/differential_drive_robot.cpp src/lidar.cpp )
ament_target_dependencies(world_node rclcpp geometry_msgs Eigen3 OpenCV std_msgs)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/msg
)

install(TARGETS world_node control_robot
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
