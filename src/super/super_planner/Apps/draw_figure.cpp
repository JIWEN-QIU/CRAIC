//
// Created by yunfan on 2021/8/29.
//


#include <super_core/super_planner.h>

/*
 * Test code:
 *      roslaunch simulator test_env.launch
 * */
#define BACKWARD_HAS_DW 1

#include "debug_utils/backward.hpp"

namespace backward {
    backward::SignalHandling sh;
}


using namespace super_planner;
using namespace std;
SuperPlanner::Ptr planner_ptr;

#include <ros/console.h>
#include <ros/ros.h>

void clickCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
    Vec3f goal(15, 0, 2.2);
    double yaw{0};
    RET_CODE ret = planner_ptr->PlanFromRest(goal, yaw, true);
    cout << "ret: " << RET_CODE_str[ret] << endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "fsm_node");
    ros::NodeHandle nh("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    cout << GREEN << " -- [FSM-Test] Begin." << RESET << endl;
    PlannerConfig planner_cfg = PlannerConfig(nh);
    rog_map::ROSParamLoader(nh, planner_cfg.rog_map_cfg);
    planner_cfg.resolution = planner_cfg.rog_map_cfg.resolution;
    planner_cfg.sample_traj_dt = planner_cfg.resolution / planner_cfg.exp_traj_cfg.max_vel;

    int step = ceil(planner_cfg.robot_r / planner_cfg.resolution);
    for (int x = -step; x <= step; x++) {
        for (int y = -step; y <= step; y++) {
            for (int z = -step; z <= step; z++) {
                if (x * x + y * y + z * z <= step * step) {
                    planner_cfg.seed_line_neighbour.push_back({x, y, z});
                }
            }
        }
    }
    std::sort(planner_cfg.seed_line_neighbour.begin(), planner_cfg.seed_line_neighbour.end(),
              [](const auto &a, const auto &b) {
                  return a[0] * a[0] + a[1] * a[1] + a[2] * a[2] < b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
              });
    planner_ptr.reset(new SuperPlanner(nh, planner_cfg));
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/lidar_slam/odom", 1);
    ros::Subscriber click_sub = nh.subscribe("/goal", 1, &clickCallback);

    /* Publisher and subcriber */
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();

    while (ros::ok()) {
        ros::Duration(0.01).sleep();
        nav_msgs::Odometry odom;
        odom.header.frame_id = "world";
        odom.header.stamp = ros::Time::now();
        odom.pose.pose.position.x = 0.0;
        odom.pose.pose.position.y = 0.0;
        odom.pose.pose.position.z = 2.2;
        odom.pose.pose.orientation.w = 1.0;
        RobotState rs;
        planner_ptr->getRobotState(rs);
        planner_ptr->VizDroneFov();
        odom_pub.publish(odom);
    }

    ros::waitForShutdown();
    return 0;
}

