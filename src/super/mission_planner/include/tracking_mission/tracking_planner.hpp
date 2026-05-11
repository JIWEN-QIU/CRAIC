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


#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/RCIn.h"
#include "Eigen/Core"
#include "config.hpp"

namespace mission_planner {

    using namespace std;
    using namespace type_utils;

    class TrackingPlanner {
    private:
        TrakcingMissionConfig cfg_;

        ros::NodeHandle nh_;

        Eigen::Vector3d cur_position;
        int waypoint_counter{0};
        bool had_odom{false};
        bool had_target_odom{false};
        bool triggered{false};
        bool new_goal{true};
        double odom_rcv_time{0};
        ros::Publisher goal_pub_, path_pub_;
        ros::Subscriber click_sub_, mavros_sub_, odom_sub_, target_odom_sub_;
        ros::Timer goal_pub_timer_;
        Vec3f target_position_;


        void OdomCallback(const nav_msgs::OdometryConstPtr &msg) {
            had_odom = true;
            odom_rcv_time = ros::Time::now().toSec();
            cur_position = Eigen::Vector3d(msg->pose.pose.position.x,
                                           msg->pose.pose.position.y,
                                           msg->pose.pose.position.z);
        }


        void MavrosRcCallback(const mavros_msgs::RCInConstPtr &msg) {

            static int last_ch_10 = 1000;
            int ch_8 = msg->channels[7];
            if (ch_8 > 1300) {
                triggered = true;
            } else {
                triggered = false;
            }
        }

        void TargetOdomCallback(const nav_msgs::OdometryConstPtr &msg) {
            had_target_odom = true;
            Vec3f target_odom_ = Vec3f(msg->pose.pose.position.x,
                                       msg->pose.pose.position.y,
                                       msg->pose.pose.position.z);
            Vec2f dir = target_odom_.head(2) - cur_position.head(2);
            target_position_.head(2) =
                    dir.normalized() * (dir.norm() - cfg_.distance_to_target_xy) + cur_position.head(2);
            target_position_.z() = target_odom_.z() + cfg_.distance_to_target_z;
        }

        void GoalPubTimerCallback(const ros::TimerEvent &e) {

            if (!triggered) {
                return;
            }
            if (!had_odom) {
                cout << RED << "No odom received!" << RESET << endl;
                return;
            }
            if (!had_target_odom) {
                cout << RED << "No target odom." << RESET << endl;
                return;
            }
            static geometry_msgs::PoseStamped goal;
            static double last_pub_t = ros::Time::now().toSec();
            double cur_t = ros::Time::now().toSec();
            if (cur_t - last_pub_t >= cfg_.publish_dt) {
                goal.header.stamp = ros::Time::now();
                goal.header.frame_id = "world";
                goal.pose.position.x = target_position_.x();
                goal.pose.position.y = target_position_.y();
                goal.pose.position.z = target_position_.z();

                Eigen::Quaterniond q = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
                                       * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY())
                                       * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
                goal.pose.orientation.x = q.x();
                goal.pose.orientation.y = q.y();
                goal.pose.orientation.z = q.z();
                goal.pose.orientation.w = q.w();

                goal_pub_.publish(goal);
                last_pub_t = cur_t;
                static int pub_cnt = 0;
                cout << GREEN << " -- [TP] publish goal: " << target_position_.transpose() << RESET << endl;
            }


        }

    public:
        TrackingPlanner() {};

        TrackingPlanner(const ros::NodeHandle &nh) {
            nh_ = nh;
            cfg_ = TrakcingMissionConfig(nh_);
            odom_sub_ = nh_.subscribe(cfg_.odom_topic, 10, &TrackingPlanner::OdomCallback, this);
            target_odom_sub_ = nh_.subscribe(cfg_.target_odom_topic, 10, &TrackingPlanner::TargetOdomCallback, this);
            goal_pub_timer_ = nh_.createTimer(ros::Duration(0.01), &TrackingPlanner::GoalPubTimerCallback, this);
            goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(cfg_.goal_pub_topic, 10);
            path_pub_ = nh_.advertise<nav_msgs::Path>(cfg_.path_pub_topic, 10);
            mavros_sub_ = nh_.subscribe("/mavros/rc/in", 10, &TrackingPlanner::MavrosRcCallback, this);
        }
    };
}
#endif //MISSION_PLANNER_WAYPOINT_PLANNER
