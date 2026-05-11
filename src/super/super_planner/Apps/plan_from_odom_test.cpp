//
// Created by yunfan on 2021/8/29.
//


#include <planner_fsm/fsm.h>

using namespace type_utils;
using namespace planner_fsm;
FSM::Ptr fsm_ptr;
Vec3f pos, vel;
bool ready = false;

bool new_goal{false};

void trigger_callback(const std_msgs::EmptyConstPtr & msg){
    new_goal = true;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
    pos = Vec3f(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
    vel = Vec3f(odom_msg->twist.twist.linear.x, odom_msg->twist.twist.linear.y, odom_msg->twist.twist.linear.z);
    if(!new_goal){
        if(vel.norm() < 0.5){
            return;
        }
        if((pos - Vec3f(12.5, 8.5, 1.0)).norm() < 1){
            return;
        }
    }
    static int cnt = 0;
    if(cnt ++ > 5 || new_goal){
        new_goal = false;
        cnt = 0;
        fsm_ptr->odomPlanTest(pos, vel);
    }
}

void timer_callback(const ros::TimerEvent &event) {
    if (!ready) { return; }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "fsm_node2");
    ros::NodeHandle nh("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);


    fsm_ptr.reset(new FSM);
    fsm_ptr->init(nh);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/lidar_slam/odom", 1, &odom_callback);
    ros::Subscriber trigger_sub = nh.subscribe<std_msgs::Empty>("/trigger", 1, &trigger_callback);
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), &timer_callback);
    ros::AsyncSpinner spinner(0);
    spinner.start();
    // Reset the simulation env
    ros::Duration(2.5).sleep();

    ready = true;
    /* Publisher and subcriber */

    ros::Duration(1.0).sleep();
    ros::waitForShutdown();
    return 0;
}
