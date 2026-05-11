//
// Created by yunfan on 2022/2/17.
//

#pragma once

#include <iostream>
#include <fstream>
#include "Eigen/Eigen"


#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_utils/visualization_utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <super_core/config.h>
#include <super_core/visualizer.h>

// Utils functions from mars planning utils
#include <geometry_utils/geometry_utils.h>
#include <geometry_utils/trajectory.h>
#include <type_utils/common_type_name.h>
#include <geometry_utils/polynomial_interpolation.h>
#include <geometry_utils/polytope.h>
#include <benchmark_utils/scope_timer.h>


namespace super_planner {
    using namespace geometry_utils;
    using namespace type_utils;
    using namespace benchmark_utils;

    class SwarmBridge {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber poly_traj_sub_;
        ros::Publisher poly_traj_pub_;


        struct TrajInfo{
            Trajectory traj{};
            int drone_id{-1};
        };

        typedef std::vector<TrajInfo> SwarmTrajData;

    public:
        SwarmBridge(const ros::NodeHandle &nh){
            nh_ = nh;
            poly_traj_sub_ = nh_.subscribe("/teammates/poly_traj", 1, &SwarmBridge::TrajCallback, this);

        }

        void PublishCommittedTrajectory(const Trajectory & cmd_traj);

        void TrajCallback(const quadrotor_msgs::PolynomialTrajectory::ConstPtr &traj_msg);

        void ExtractValidTeamTraj();


    };
}
