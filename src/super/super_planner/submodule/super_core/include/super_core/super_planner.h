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
#include "fmt/color.h"

// Utils functions from mars planning utils
#include <geometry_utils/geometry_utils.h>
#include <geometry_utils/trajectory.h>
#include <type_utils/common_type_name.h>
#include <geometry_utils/polynomial_interpolation.h>
#include <geometry_utils/polytope.h>
#include <benchmark_utils/scope_timer.h>


#include "super_core/control_cmd_gene.h"
#include "traj_opt/config.h"
#include "traj_opt/exp_traj_optimizer_s4.h"
#include "traj_opt/backup_traj_optimizer_s4.h"
#include "path_search/astar.h"
#include "rog_map/rog_map.h"
#include "super_core/corridor_generator.h"
#include "super_core/fov_checker.h"
#include "super_core/swarm_bridge.h"

#include "quadrotor_msgs/PolynomialTrajectory.h"
#include "traj_opt/yaw_traj_opt.h"

namespace super_planner {
    using namespace geometry_utils;
    using namespace type_utils;
    using namespace benchmark_utils;
    using namespace path_search;
    using namespace rog_map;

    static vector<string> RET_CODE_str{"FAILED", "NO_NEED", "SUCCESS", "FINISH", "NEW_TRAJ", "EMER", "OPT_FAILED"};

    enum LogTime {
        EPX_TRAJ_FRONTEND = 0,
        EXP_TRAJ_OPT,
        GENERATE_EXP_TRAJ,
        BACK_TRAJ_FRONTEND,
        BACK_TRAJ_OPT,
        GENERATE_BACK_TRAJ,
        TOTAL_REPLAN,
        VISUALIZATION
    };

    static vector<string> log_time_str
            {"EPX_TRAJ_FRONTEND",
             " EXP_TRAJ_OPT",
             " GENERATE_EXP_TRAJ",
             " BACK_TRAJ_FRONTEND",
             " BACK_TRAJ_OPT",
             " GENERATE_BACK_TRAJ",
             " TOTAL_REPLAN",
             " VISUALIZATION"};

    class SuperPlanner {
    public:

        RET_CODE odomPlanTest(const Vec3f &pos, const Vec3f &vel,
                              const Vec3f &goal_p);

        double ft{0}, bt{0};
        int ft_cnt{0}, bt_cnt{0};

        double getFrontendTime() {
            if (ft_cnt == 0) return -1;
            double ave_t = ft / ft_cnt;
            ft = 0;
            ft_cnt = 0;
            return ave_t;
        }

        double getBackendTime() {
            if (bt_cnt == 0) return -1;
            double ave_t = bt / bt_cnt;
            bt = 0;
            bt_cnt = 0;
            return ave_t;
        }

        double getLastMappingAveTime() {
            return map_ptr_->getLastMappingAveTime();
        }

    private:
        ros::NodeHandle nh_;
        PlannerConfig cfg_;
        ROGMap::Ptr map_ptr_;
        CorridorGenerator::Ptr cg_ptr_;
        Astar::Ptr astar_ptr_;
        Visualizer viz_;
        traj_opt::ExpTrajOpt::Ptr exp_traj_opt_;
        traj_opt::BackupTrajOpt::Ptr back_traj_opt_;
        traj_opt::YawTrajOpt::Ptr yaw_traj_opt_;

        ControlCommandGenerator cmd_gene_;
        CIRI::Ptr ciri_;

        RobotState robot_state_;
        std::mutex drone_state_mutex_;
        std::mutex replan_lock_;

        bool emer_stop{false};
        double planner_process_start_WT_;

        struct ExpTrajInfo {
            Trajectory optimized_exp_traj{};
            Trajectory exp_yaw_traj{};
            PolytopeVec sfcs;
            bool connected_to_goal{false};
            bool empty{true};
            bool is_rest2rest{true};
            // Set to -1 means no backuptraj on this exp traj
            double back_traj_start_TT;
            bool all_traj_free{true};
            Vec3f cur_vel;
        };

        struct BackTrajInfo {
            Trajectory optimized_back_traj{};
            Trajectory back_yaw_traj{};
            Polytope free_poly{};
            bool empty{true};
            RobotState robot_state{};
            double back_traj_start_TT{0};
            ExpTrajInfo *ref_exp_traj{};
        };

        struct CmdTrajInfo {
            // The CmdTraj is evaluated by another thread, need mutex.
            std::mutex sample_traj_mutex_;
            // pos_traj = partial exp + backup
            Trajectory pos_traj;
            Trajectory yaw_traj;
            // The Wall time when exp traj has been evaluated.
            double backup_traj_start_TT;
            // The Wall time when backup traj should start
            ros::Time backup_traj_start_WT;
            bool use_back_traj{false};
        };

        struct GoalInfo {
            Vec3f goal_p{0, 0, 0};
            double goal_yaw{0};
            bool new_goal{true};
        } gi_;

        FOVChecker::Ptr fov_checker_;

        CmdTrajInfo cmd_traj_info_;
        ExpTrajInfo last_exp_traj_info_;


        vector<double> time_consuming_;

        vector<Vec3f> sphereical_neighbor_list_;

        void RefreshFineInfNeighbors();

    public:


        SuperPlanner(const ros::NodeHandle &nh, PlannerConfig &cfg);

        ~SuperPlanner() = default;

        typedef shared_ptr<SuperPlanner> Ptr;

        void VizDroneFov() {
            Mat3f R(robot_state_.q);
            vec_E<Mat3f> fov_pts;
            MatrixX4d planes;
            fov_checker_->GetAllFovPlane(R, robot_state_.p, fov_pts);
            viz_.VisualizeDroneFOV(fov_pts);
        }

    public:

        void GetOneHeartbeat(quadrotor_msgs::PolynomialTrajectory &heartbeat, bool &traj_finish);

        void GetCmdTraj(quadrotor_msgs::PolynomialTrajectory &cmd_traj);

        void
        GetOneCommandFromTraj(quadrotor_msgs::PositionCommand &pos_cmd, bool &traj_finish);

        void GetOneMpcCommandFromTraj(quadrotor_msgs::MpcPositionCommand &pos_cmd, bool &traj_finish);

        void GetModuleTimeConsuming(vector<double> &time);

        void SetRobotRadius(const double robot_r);

        double GetRobotRadius() const {
            return cfg_.robot_r;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:

        bool shiftPointToNearestNoneOcc(const Vec3f &start_pt, Vec3f &shifted_point,
                                        bool use_inf_map = true);


    public:

        /* Tow type of replan strategy */
        RET_CODE PlanFromRest(const Vec3f &goal_p,
                              const double &goal_yaw,
                              const bool &new_goal);

        RET_CODE
        ReplanOnce(const Vec3f &goal_p,
                   const double &goal_yaw,
                   const bool &new_goal);


    private:


        /* For EXP traj generation */
        RET_CODE GenerateRestToRestExpTraj(const Vec3f &start_position,
                                           ExpTrajInfo &exp_traj_info);


        RET_CODE GenerateReplanExpTraj(ExpTrajInfo &last_exp_traj_info,
                                       ExpTrajInfo &out_exp_traj_info);

    private:

        /* For Backup traj generation */
        RET_CODE GenerateBackupTrajectory(BackTrajInfo &back_traj_info);

    private:

        bool GetStateFromTraj(const Trajectory &traj, double eval_t, StatePVAJ &out_state);

        int getNearestFurtherGoalPoint(const vec_E<Vec3f> &goals, const Vec3f &start_pt);

        /// The goal waypoints
        /// if success, the output path_segs must have size over 2
        std::mutex path_search_mutex_;

        bool PathSearch(const Vec3f &start_pt, const Vec3f &goal,
                        const double &searching_horizon,
                        vec_Vec3f &path);

        bool CutPolytopeByFov(Polytope &poly, const Vec3f &guide_point);

    public:

        /* For trajectory server */
        bool ReachEnd();

        void getRobotState(RobotState &out);

        int getUnfinishedGoalId();


        // for safe autopilot
    public:

        bool isEasyGoal(const Vec3f &goal_position);


        void findNearestKnownFreeGoal(const Vec3f &goal_position, Vec3f &free_local_goal, const double &safe_dis = 0);

        // for paper experiment
    public:

        RET_CODE pointToPointPlanning(const Vec3f &start_pt, const Vec3f &goal_pt) {
            /// 2) Generate Exp traj
            ExpTrajInfo exp_traj_info;
            BackTrajInfo back_traj_info;
            gi_.goal_p = goal_pt;
            RET_CODE exp_ret_code = GenerateRestToRestExpTraj(start_pt, exp_traj_info);
            if (exp_ret_code == FAILED) {
                ROS_WARN(" -- [SUPER] in [PlanFromRest] GenerateExpTrajectory failed with %s.",
                         RET_CODE_str[exp_ret_code].c_str());
                return FAILED;
            }

            viz_.VisualizeExpTrajInColor(exp_traj_info.optimized_exp_traj);

            cmd_traj_info_.yaw_traj.clear();
            exp_traj_info.is_rest2rest = true;
            back_traj_info.ref_exp_traj = &exp_traj_info;
            back_traj_info.back_traj_start_TT = -1;
            RET_CODE back_ret_code = GenerateBackupTrajectory(back_traj_info);;

            if (back_ret_code == SUCCESS) {
                emer_stop = false;
                const std::lock_guard<std::mutex> lock(cmd_traj_info_.sample_traj_mutex_);
                exp_traj_info.optimized_exp_traj.start_WT = ros::Time::now().toSec();
                if (!exp_traj_info.optimized_exp_traj.
                        getPartialTrajectoryByTime(0, back_traj_info.back_traj_start_TT,
                                                   cmd_traj_info_.pos_traj)) {
                    ROS_ERROR(" -- [SUPER] in [pointToPointPlanning] getPartialTrajectoryByTime failed.");
                    return FAILED;
                }
                cmd_traj_info_.pos_traj = cmd_traj_info_.pos_traj + back_traj_info.optimized_back_traj;
                cmd_traj_info_.backup_traj_start_TT = back_traj_info.back_traj_start_TT;
                cmd_traj_info_.use_back_traj = true;
                last_exp_traj_info_ = exp_traj_info;
                gi_.new_goal = false;
                // For visualization
                TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
                viz_.VisualizeCommitTrajectory(cmd_traj_info_.pos_traj, cmd_traj_info_.backup_traj_start_TT);
                time_consuming_[VISUALIZATION] += t_viz.stop();
                return SUCCESS;
            } else if (back_ret_code == FINISH) {
                // 意味着exp轨迹就以及完全free了
                emer_stop = false;
                const std::lock_guard<std::mutex> lock(cmd_traj_info_.sample_traj_mutex_);
                exp_traj_info.optimized_exp_traj.start_WT = ros::Time::now().toSec();
                cmd_traj_info_.pos_traj = exp_traj_info.optimized_exp_traj;
                cmd_traj_info_.backup_traj_start_TT = back_traj_info.back_traj_start_TT;
                cmd_traj_info_.use_back_traj = false;
                last_exp_traj_info_ = exp_traj_info;
                gi_.new_goal = false;
                // For visualization
                TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
                viz_.VisualizeCommitTrajectory(cmd_traj_info_.pos_traj, -1);
                time_consuming_[VISUALIZATION] += t_viz.stop();
                return SUCCESS;
            }
            ROS_WARN(" -- [SUPER] in [PlanFromRest] GenerateBackupTrajectory return [%s], force return",
                     RET_CODE_str[back_ret_code].c_str());
            return FAILED;
        }
    };
}
