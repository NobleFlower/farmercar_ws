cmake_minimum_required(VERSION 3.0.2)

project(apriltag_navigation)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  geometry_msgs
  apriltag_ros
  tricycle_controller
  tf2
  tf2_geometry_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    roslib
    std_msgs
    geometry_msgs
    apriltag_ros
    tricycle_controller
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

catkin_install_python(
  PROGRAMS scripts/voice_apriltag_navigation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_voice_navigation.py)
  find_package(rostest REQUIRED)
  add_rostest(test/voice_navigation.test)
endif()
