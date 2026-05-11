//
// Created by yunfan on 2022/3/26.
//

#ifndef MISSION_PLANNER_PATH_PLANNER
#define MISSION_PLANNER_PATH_PLANNER

#include "ros/ros.h"
#include "vector"
#include "string"
#include "config.hpp"
#include "nav_msgs/Path.h"


#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/RCIn.h"
#include "Eigen/Core"
#include "queue"

namespace path_mission {
    using namespace std;
    using namespace type_utils;

    class PathMissionPlanner {
    private:
        MissionConfig cfg_;

        ros::NodeHandle nh_;

        Eigen::Vector3d cur_position;
        int waypoint_counter{0};
        bool had_odom{false};
        bool triggered{false};
        bool new_goal{true};
        double odom_rcv_time{0};
        ros::Publisher goal_pub_, path_pub_;
        ros::Subscriber click_sub_, mavros_sub_, odom_sub_;
        ros::Timer goal_pub_timer_;
        queue<Vec3f> goal_path;


        void OdomCallback(const nav_msgs::OdometryConstPtr &msg) {
            had_odom = true;
            odom_rcv_time = ros::Time::now().toSec();
            cur_position = Eigen::Vector3d(msg->pose.pose.position.x,
                                           msg->pose.pose.position.y,
                                           msg->pose.pose.position.z);
        }

        bool CloseToPoint(Vec3f &position) {
            return (position - cur_position).norm() < cfg_.switch_dis;
        }

        void RvizClickCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
            if (!had_odom) {
                return;
            }
            triggered = true;
            new_goal = true;
            waypoint_counter = 0;
            cout << YELLOW << " -- [MISSION] Rviz triggered." << RESET << endl;
        }

        void MavrosRcCallback(const mavros_msgs::RCInConstPtr &msg) {
            static int last_ch_10 = 1000;
            if (!had_odom) {
                return;
            }
            int ch_10 = msg->channels[9];
            bool pitch_up = msg->channels[1] < 1200;
            if (last_ch_10 > 1500 && ch_10 < 1500) {
                triggered = true;
                new_goal = true;
                waypoint_counter = 0;
                cout << YELLOW << " -- [MISSION] Mavros triggered." << RESET << endl;
            }
            last_ch_10 = ch_10;
        }

        void GoalPubTimerCallback(const ros::TimerEvent &e) {

            if (!triggered) {
                return;
            }

//            if((cur_position - goal_path.front()).norm() > cfg_.planning_horizon){
//                return;
//            }
            switch (cfg_.cmd_type) {
                case CmdType::PATH: {
                    cout << YELLOW << " -- [MISSION] Path mode is not defined." << RESET << endl;
                    break;
                }

                case CmdType::WAYPOINT: {
                    double cur_t = ros::Time::now().toSec();
                    if (cur_t - odom_rcv_time > cfg_.odom_timeout) {
                        static double last_print_t = ros::Time::now().toSec();
                        if (cur_t - last_print_t > 1.0) {
                            last_print_t = cur_t;
                            cout << YELLOW << " -- [MISSION] Odom Timeout!" << RESET << endl;
                        }
                        return;
                    }
                    Vec3f temp_p = goal_path.front();
                    while (!goal_path.empty()) {
                        temp_p = goal_path.front();
                        if ((temp_p - cur_position).norm() < cfg_.planning_horizon) {
                            goal_path.pop();
                        } else {
                            break;
                        }
                    }
                    if (goal_path.empty()) {
                        triggered = false;
                        new_goal = false;
                    }

                    geometry_msgs::PoseStamped goal;
                    goal.pose.position.x = temp_p.x();
                    goal.pose.position.y = temp_p.y();
                    goal.pose.position.z = temp_p.z();
                    goal.pose.orientation.w = 1;
                    goal.header.frame_id = "world";
                    goal.header.stamp = ros::Time::now();
                    goal_pub_.publish(goal);
                    cout << YELLOW << " -- [MISSION] Pub goal to " << temp_p.transpose() << RESET << endl;

                    break;
                }
                default: {
                    cout<<RED<< " -- [MISSION] Unknown cmd type."<<RESET<<endl;
                    break;
                }

            }
        }

        void computeGoalPath() {
            for (int i = 0; i < cfg_.waypoints.size() - 1; i++) {
                double dis = (cfg_.waypoints[i] - cfg_.waypoints[i + 1]).norm();
                int num = dis / cfg_.discret_resolution;
                for (int j = 0; j < num; j++) {
                    Vec3f point = cfg_.waypoints[i] + (cfg_.waypoints[i + 1] - cfg_.waypoints[i]) * j / num;
                    goal_path.push(point);
                }
            }
            cout<<GREEN<<" -- [MISSION] Compute goal path with "<<goal_path.size()<<" points."<<RESET<<endl;
        }

    public:
        PathMissionPlanner() {};

        PathMissionPlanner(const ros::NodeHandle &nh) {
            nh_ = nh;
            cfg_ = MissionConfig(nh_);
            odom_sub_ = nh_.subscribe(cfg_.odom_topic, 10, &PathMissionPlanner::OdomCallback, this);
            goal_pub_timer_ = nh_.createTimer(ros::Duration(1.0 / cfg_.goal_pub_hz),
                                              &PathMissionPlanner::GoalPubTimerCallback, this);
            goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(cfg_.goal_pub_topic, 10);
            path_pub_ = nh_.advertise<nav_msgs::Path>(cfg_.path_pub_topic, 10);
            click_sub_ = nh_.subscribe("/goal", 10, &PathMissionPlanner::RvizClickCallback, this);
            mavros_sub_ = nh_.subscribe("/mavros/rc/in", 10, &PathMissionPlanner::MavrosRcCallback, this);

            // Compute goal path
            computeGoalPath();
        }
    };
}
#endif //MISSION_PLANNER_PATH_PLANNER
