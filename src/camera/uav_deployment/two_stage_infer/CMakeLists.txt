cmake_minimum_required(VERSION 3.0.2)
project(two_stage_infer)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
)


catkin_package()

# 安装 Python 脚本
catkin_install_python(PROGRAMS
  scripts/infer_node.py
  scripts/windows_udp_cam.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
