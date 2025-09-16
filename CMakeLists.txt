cmake_minimum_required(VERSION 3.22)
project(lsm_localization)

# C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(csm REQUIRED csm)
find_package(Eigen3 REQUIRED)

# Include and link directories
include_directories(
  include
  ${csm_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(${csm_LIBRARY_DIRS})

# Library
add_library(lsm_localization
  src/laser_scan_matcher.cpp
  src/scan_constructor.cpp
  src/segment_tree.cpp
)

target_link_libraries(lsm_localization
  ${csm_LIBRARIES}
  potrace  # Assuming potrace is available system-wide
)

ament_target_dependencies(lsm_localization
  rclcpp tf2 tf2_ros std_msgs sensor_msgs geometry_msgs nav_msgs tf2_geometry_msgs
)

# Executable
add_executable(lsm_localization_node src/laser_scan_matcher_node.cpp)
target_link_libraries(lsm_localization_node
    lsm_localization
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
)

ament_target_dependencies(lsm_localization_node
  rclcpp
  tf2
  tf2_ros
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
)

# Install targets
install(TARGETS
  lsm_localization
  lsm_localization_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/laser_scan_matcher/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Export package
ament_export_include_directories(include)
ament_export_dependencies(rclcpp tf2 tf2_ros std_msgs sensor_msgs geometry_msgs nav_msgs)
ament_export_libraries(lsm_localization)

ament_package()
