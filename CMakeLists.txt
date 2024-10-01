cmake_minimum_required(VERSION 3.0.2)
project(test_stewart)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ik src/ik.cpp)
target_link_libraries(ik ${catkin_LIBRARIES})
# add_library(joint_controller SHARED ~/catkin_ws/src/stewart_platform_learning/src/stewart_platform/plugin/joint_controller.cc)
# target_link_libraries(joint_controller ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
