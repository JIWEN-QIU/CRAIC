//
// Created by yunfan on 2021/8/29.
//

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
/*
 * Test code:
 *      roslaunch simulator test_env.launch
 * */
#define BACKWARD_HAS_DW 1
#include "debug_utils/backward.hpp"
namespace backward{
    backward::SignalHandling sh;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "fsm_node");
    ros::NodeHandle nh("~");
    /* Publisher and subcriber */
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();
    ros::waitForShutdown();
    return 0;
}

