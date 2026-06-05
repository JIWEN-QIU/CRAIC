cmake_minimum_required(VERSION 2.8.3)
project(super_planner)

add_compile_options(-Werror=return-type)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")


find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        std_srvs
        pcl_ros
        geometry_msgs
        nav_msgs
        message_generation
        quadrotor_msgs
        visualization_msgs
        message_filters
        mars_planning_utils
        rog_map
        rosfmt
        )

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost 1.70.0 COMPONENTS system filesystem serialization REQUIRED)
catkin_package(
        INCLUDE_DIRS
        LIBRARIES super_planner
        CATKIN_DEPENDS roscpp rospy std_msgs std_srvs
        #  DEPENDS system_lib
)

include_directories(
        SYSTEM
        include
        ../../../../devel/include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)


add_subdirectory(submodule/traj_opt)
add_subdirectory(submodule/super_core)
add_subdirectory(submodule/planner_fsm)

add_library(super_planner INTERFACE)

target_include_directories(super_planner INTERFACE
        traj_opt
        super_core
        planner_fsm
        )
target_link_libraries(super_planner INTERFACE
        traj_opt
        super_core
        planner_fsm
        ${CATKIN_LIBRARIES}
        )

set(THIRD_PARTY
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        super_planner
        -ldw
        )


add_executable(fsm_node
        Apps/fsm_node.cpp
        )
target_link_libraries(fsm_node
        ${THIRD_PARTY}
        )




add_executable(offline_traj_opt
        Apps/offline_traj_opt.cpp
)
target_link_libraries(offline_traj_opt
        ${THIRD_PARTY}
)


add_executable(gen_random_sfc
        Apps/gen_random_sfc.cpp
        )
target_link_libraries(gen_random_sfc
        ${THIRD_PARTY}
        )

add_executable(super_bench_node
        Apps/bench_node.cpp
        )
target_link_libraries(super_bench_node
        ${THIRD_PARTY}
        )

add_executable(plan_from_odom
        Apps/plan_from_odom_test.cpp
        )
target_link_libraries(plan_from_odom
        ${THIRD_PARTY}
        )
