//
// Created by yunfan on 2021/8/29.
//


#include <planner_fsm/fsm.h>

/*
 * Test code:
 *      roslaunch simulator test_env.launch
 * */
#define BACKWARD_HAS_DW 1

#include "debug_utils/backward.hpp"

namespace backward {
    backward::SignalHandling sh;
}


using namespace planner_fsm;
using namespace std;
FSM::Ptr fsm_ptr;

#include <ros/console.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "fsm_node");
    ros::NodeHandle nh("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    cout << GREEN << " -- [FSM-Test] Begin." << RESET << endl;

    fsm_ptr.reset(new FSM);
    fsm_ptr->init(nh);

    /* Publisher and subcriber */
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();
    ros::waitForShutdown();
    return 0;
}

