cmake_minimum_required(VERSION 2.8.3)
project(ros_whill)

find_package(
  catkin REQUIRED
  COMPONENTS message_generation
             roscpp
             rospy
             std_msgs
             tf
             serial
)

add_subdirectory(./src/whill)
add_message_files(FILES SpeedPack.msg)

add_service_files(FILES SetSpeedProfile.srv)
generate_messages(DEPENDENCIES std_msgs ros_whill)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
add_executable(
  ros_whill
  src/ros_whill.cpp
  src/odom.cpp
  src/utils/rotation_tools.cpp
  src/utils/unit_convert.cpp
)
add_dependencies(
  ros_whill
  ${PROJECT_NAME}_generate_messages_cpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ros_whill
  lib_whill
  ${catkin_LIBRARIES}
)
