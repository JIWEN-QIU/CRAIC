cmake_minimum_required(VERSION 3.0.2)
project(special_target_detection_n150)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/special_target_detector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY special_target_detection_n150/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/special_target_detection_n150
)
