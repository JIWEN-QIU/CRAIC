cmake_minimum_required(VERSION 2.8.3)
project(mission_planner)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        pcl_ros
        geometry_msgs
        nav_msgs
        message_generation
        quadrotor_msgs
        visualization_msgs
        message_filters
        mars_planning_utils
        )
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES
        CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

include_directories(
        SYSTEM
        include
        ../../../devel/include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

set(THIRD_PARTY
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        -ldw
        )


add_executable(star_mission
        Apps/star_mission.cpp
        )
target_link_libraries(star_mission
        ${THIRD_PARTY}
        )


add_executable(waypoint_mission
        Apps/waypoint_mission.cpp
        )
target_link_libraries(waypoint_mission
        ${THIRD_PARTY}
        )


add_executable(tracking_mission
        Apps/tracking_mission.cpp
        )
target_link_libraries(tracking_mission
        ${THIRD_PARTY}
        )


add_executable(path_mission
        Apps/path_mission.cpp
        )
target_link_libraries(path_mission
        ${THIRD_PARTY}
        )

#
#add_executable(minco_debug
#        test/minco_debug.cpp
#        )
#target_link_libraries(minco_debug
#        ${THIRD_PARTY}
#        )
#
