cmake_minimum_required(VERSION 3.0.2)
project(haptic_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

catkin_package()

install(PROGRAMS
  scripts/haptic_bridge.py
  scripts/sitl_launcher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
