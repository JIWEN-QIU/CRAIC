cmake_minimum_required(VERSION 3.10.0)
project(electromagnet_control)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  mavros_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES electromagnet_control
  CATKIN_DEPENDS roscpp std_msgs std_srvs mavros_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(electromagnet_control
  src/electromagnet_controller.cpp
)

target_link_libraries(electromagnet_control
  ${catkin_LIBRARIES}
)

add_executable(electromagnet_test_node
  src/electromagnet_test_node.cpp
)

target_link_libraries(electromagnet_test_node
  electromagnet_control
  ${catkin_LIBRARIES}
)

add_executable(electromagnet_service_node
  src/electromagnet_service_node.cpp
)

target_link_libraries(electromagnet_service_node
  electromagnet_control
  ${catkin_LIBRARIES}
)
