cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(lsm_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

find_package(PkgConfig)
find_package(Eigen3 REQUIRED)
set(potrace_LIBRARIES "potrace")
pkg_check_modules(csm REQUIRED csm)
pkg_search_module(Eigen3 REQUIRED eigen3)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lsm_engine
  CATKIN_DEPENDS roscpp message_runtime std_msgs geometry_msgs sensor_msgs nav_msgs tf
)

include_directories(include ${catkin_INCLUDE_DIRS} ${csm_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
link_directories(${csm_LIBRARY_DIRS})

add_library(lsm_engine src/laser_scan_matcher.cpp src/scan_constructor.cpp src/segment_tree.cpp)
target_link_libraries(lsm_engine ${catkin_LIBRARIES} ${csm_LIBRARIES} ${potrace_LIBRARIES})
add_dependencies(lsm_engine ${csm_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(lsm_engine_node src/laser_scan_matcher_node.cpp)
target_link_libraries(lsm_engine_node lsm_engine )

install(TARGETS lsm_engine
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/laser_scan_matcher/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS lsm_engine_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
