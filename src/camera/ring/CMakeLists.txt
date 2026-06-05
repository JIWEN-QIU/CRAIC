cmake_minimum_required(VERSION 3.0.2)
project(ring_detector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/ring_detector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
