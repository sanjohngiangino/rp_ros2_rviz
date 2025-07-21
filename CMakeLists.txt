cmake_minimum_required(VERSION 3.8)
project(rp_ros2_rviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake          REQUIRED)
find_package(rclcpp               REQUIRED)
find_package(std_msgs             REQUIRED)
find_package(sensor_msgs          REQUIRED)
find_package(geometry_msgs        REQUIRED)
find_package(nav_msgs             REQUIRED)

find_package(tf2                  REQUIRED)
find_package(tf2_ros             REQUIRED)
find_package(tf2_geometry_msgs    REQUIRED)

find_package(cv_bridge            REQUIRED)
find_package(image_transport      REQUIRED)

find_package(OpenCV               REQUIRED)
find_package(Eigen3               REQUIRED)


include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)


add_executable(controller_node
  src/controller_node.cpp
  src/dmap_planner.cpp
  src/dmap.cpp
  src/robot.cpp
)

add_executable(world_node
  src/world_node.cpp
  src/dmap_planner.cpp
  src/dmap.cpp
  src/robot.cpp
)

add_executable(pathplanner_node
  src/pathplanner_node.cpp
  src/dmap_planner.cpp
  src/dmap.cpp
  src/robot.cpp
)

add_executable(mapserver
  src/mapserver.cpp
  src/dmap_planner.cpp
  src/dmap.cpp
  src/robot.cpp
)

add_executable(particle_node_filter
  src/particle_node_filter.cpp
  src/dmap_planner.cpp
  src/dmap.cpp
  src/robot.cpp
)

set(COMMON_DEPS
  rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs
  tf2 tf2_ros tf2_geometry_msgs
  cv_bridge image_transport
)

foreach(target controller_node world_node pathplanner_node mapserver particle_node_filter)
  ament_target_dependencies(${target} ${COMMON_DEPS})
  target_link_libraries(${target} ${OpenCV_LIBRARIES} Eigen3::Eigen)
endforeach()

install(TARGETS controller_node world_node pathplanner_node mapserver particle_node_filter
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  # rimuovi quando aggiungi i copyright
  set(ament_cmake_cpplint_FOUND TRUE)    # rimuovi se lavori in repo git
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
