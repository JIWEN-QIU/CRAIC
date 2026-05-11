//
// Created by yunfan on 2022/3/26.
//

#ifndef MISSION_PLANNER_WAYPOINT_PLANNER
#define MISSION_PLANNER_WAYPOINT_PLANNER

#include "ros/ros.h"
#include "vector"
#include "string"
#include "config.hpp"
#include "nav_msgs/Path.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_utils/visualization_utils.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/RCIn.h"
#include "Eigen/Core"
#include "type_utils/common_type_name.h"

namespace mission_planner {
    using namespace std;
    using namespace type_utils;

    class WaypointPlanner {

    private:
        MissionConfig cfg_;

        ros::NodeHandle nh_;

        Eigen::Vector3d cur_position;
        int waypoint_counter{0};
        bool had_odom{false};
        bool triggered{false};
        bool new_goal{true};
        double odom_rcv_time{0};
        ros::Publisher goal_pub_, path_pub_, mkr_pub_;
        ros::Subscriber click_sub_, mavros_sub_, odom_sub_;
        ros::Timer goal_pub_timer_;


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

        bool CloseToPoint(Vec3f &position, int id) {
            return (position - cur_position).norm() < cfg_.switch_dis_vec[id];
        }


        void RvizClickCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
            if (!had_odom) {
                return;
            }
            triggered = true;
            new_goal = true;
            waypoint_counter = 0;
            cout << YELLOW
                    " -- [MISSION] Rviz triggered." << RESET << endl;
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
            switch (cfg_.cmd_type) {
                case CmdType::PATH: {
                    if (new_goal) {
                        nav_msgs::Path path;
                        path.header.frame_id = "world";
                        path.header.stamp = ros::Time::now();
                        for (auto &wp: cfg_.waypoints) {
                            geometry_msgs::PoseStamped pose;
                            pose.header.frame_id = "world";
                            pose.header.stamp = ros::Time::now();
                            pose.pose.position.x = wp[0];
                            pose.pose.position.y = wp[1];
                            pose.pose.position.z = wp[2];
                            path.poses.push_back(pose);
                        }
                        path_pub_.publish(path);
                        new_goal = false;
                        triggered = false;
                    }
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


                    if (CloseToPoint(cfg_.waypoints[waypoint_counter], waypoint_counter)) {
                        cout << RED << " -- [MISSION] Close to goal {}, switch to next." << RESET << endl;
                        waypoint_counter++;
                        new_goal = true;
                        if (waypoint_counter >= cfg_.waypoints.size()) {
                            // 结束，停止发布。
                            waypoint_counter = cfg_.waypoints.size() - 1;
                            triggered = false;
                            new_goal = false;
                        }
                    }

                    static double last_pub_time = 0;

                    if (new_goal || cur_t - last_pub_time > cfg_.publish_dt) {
                        new_goal = false;
                        geometry_msgs::PoseStamped goal;
                        goal.pose.position.x = cfg_.waypoints[waypoint_counter].x();
                        goal.pose.position.y = cfg_.waypoints[waypoint_counter].y();
                        goal.pose.position.z = cfg_.waypoints[waypoint_counter].z();
                        goal.pose.orientation.w = 1;
                        goal.header.frame_id = "world";
                        goal.header.stamp = ros::Time::now();
                        last_pub_time = cur_t;
                        goal_pub_.publish(goal);
                        cout << YELLOW << " -- [MISSION] Pub goal to " << cfg_.waypoints[waypoint_counter].transpose()
                             << RESET << endl;
                        cout << YELLOW << "\t cur odom dis = "
                             << (cfg_.waypoints[waypoint_counter] - cur_position).norm() << endl;
                    }
                    break;
                }
                default: {
                    cout << RED << " -- [MISSION] Unknown cmd type." << RESET << endl;
                    break;
                }

            }
        }

    public:
        WaypointPlanner() {};

        WaypointPlanner(const ros::NodeHandle &nh) {
            nh_ = nh;
            cfg_ = MissionConfig(nh_);
            odom_sub_ = nh_.subscribe(cfg_.odom_topic, 10, &WaypointPlanner::OdomCallback, this);
            goal_pub_timer_ = nh_.createTimer(ros::Duration(0.01), &WaypointPlanner::GoalPubTimerCallback, this);
            goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(cfg_.goal_pub_topic, 10);
            path_pub_ = nh_.advertise<nav_msgs::Path>(cfg_.path_pub_topic, 10);
            click_sub_ = nh_.subscribe("/goal", 10, &WaypointPlanner::RvizClickCallback, this);
            mavros_sub_ = nh_.subscribe("/mavros/rc/in", 10, &WaypointPlanner::MavrosRcCallback, this);
            mkr_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("mkr", 1);
            ros::Duration(0.5).sleep();
            visualization_utils::VisualUtils::VisualizePath(mkr_pub_, cfg_.waypoints, Color::SteelBlue(), "waypoints",
                                                            0.5, 0.3);
            for (int i = 0; i < cfg_.switch_dis_vec.size(); i++) {
                Color c = Color::Orange();
                c.a = 0.2;
                visualization_utils::VisualUtils::VisualizePoint(mkr_pub_, cfg_.waypoints[i], c,
                                                                 "switch_dis", cfg_.switch_dis_vec[i] * 2, i);
                ros::Duration(0.01).sleep();
                visualization_utils::VisualUtils::VisualizeText(mkr_pub_, "id", to_string(1 + i), cfg_.waypoints[i],
                                                                Color::Black(), 3, i);
                ros::Duration(0.01).sleep();
            }
        }
    };
}
#endif //MISSION_PLANNER_WAYPOINT_PLANNER
