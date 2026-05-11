//
// Created by yunfan on 2021/8/29.
//


#include <planner_fsm/fsm.h>

using namespace type_utils;
using namespace planner_fsm;
int main(int argc, char **argv) {
    ros::init(argc, argv, "fsm_node");
    ros::NodeHandle nh("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    FSM::Ptr fsm_ptr;

    ros::Publisher cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 1);
    ros::AsyncSpinner spinner(0);
    spinner.start();
    // Reset the simulation env
    ros::Duration(0.2).sleep();
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.frame_id = "world";
    cmd.position.x = 0.0;
    cmd.position.y = -50.0;
    cmd.position.z = 1.5;
    cmd.velocity.x = 0.0;
    cmd.velocity.y = 0.0;
    cmd.velocity.z = 0.0;
    cmd.acceleration.x = 0.0;
    cmd.acceleration.y = 0.0;
    cmd.acceleration.z = 0.0;
    cmd.yaw = 1.5741;
    cmd.yaw_dot = 0.0;
    int cnt = 10;
    while (cnt--) {
        cmd.header.stamp = ros::Time::now();
        cmd_pub.publish(cmd);
        ros::Duration(0.1).sleep();
    }


    fsm_ptr.reset(new FSM);
    fsm_ptr->init(nh);

    ros::Duration(2.5).sleep();

    while (ros::ok()) {
        fsm_ptr->setGoal(Vec3f(0, 50, 1.5));
        ros::Duration(2).sleep();
    }


    /* Publisher and subcriber */

    ros::Duration(1.0).sleep();
    ros::waitForShutdown();
    return 0;
}
