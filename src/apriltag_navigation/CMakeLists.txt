cmake_minimum_required(VERSION 3.0.2)

project(apriltag_navigation)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  apriltag_ros
  tf2
  tf2_geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    apriltag_ros
    tf2
    tf2_geometry_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(
  apriltag_line_nav
  src/apriltag_line_nav.cpp
)


target_link_libraries(
  apriltag_line_nav
  ${catkin_LIBRARIES}
)


add_dependencies(
  apriltag_line_nav
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)