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
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Define the executable and link dependencies
#add_executable(dmap_node src/dmap_node.cpp src/dmap_planner.cpp src/dmap.cpp src/robot.cpp)
add_executable(world_node src/world_node.cpp src/dmap_planner.cpp src/dmap.cpp src/robot.cpp)
add_executable(pathplanner_node src/pathplanner_node.cpp src/dmap_planner.cpp src/dmap.cpp src/robot.cpp)

# Link the necessary ROS 2 packages with the target

ament_target_dependencies(world_node 
  rclcpp 
  geometry_msgs 
  Eigen3 
  OpenCV 
  std_msgs 
  sensor_msgs 
  cv_bridge
)
ament_target_dependencies(pathplanner_node 
  rclcpp 
  geometry_msgs 
  Eigen3 
  OpenCV 
  std_msgs 
  sensor_msgs 
  cv_bridge
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

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)
include_directories($ENV{ROS2_INSTALL_PATH}/include)

install(TARGETS  world_node pathplanner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
