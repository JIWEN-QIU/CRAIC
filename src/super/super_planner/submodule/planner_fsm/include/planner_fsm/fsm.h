#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Empty.h>
#include <memory>
#include "quadrotor_msgs/MincoTrajectory.h"
#include "quadrotor_msgs/PolynomialTrajectory.h"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/Float64.h"
#include "std_srvs/SetBool.h"
#include <visualization_utils/visualization_utils.h>
#include <super_core/super_planner.h>
#include <super_core/config.h>
#include <planner_fsm/config.h>
#include "mavros_msgs/RCIn.h"
#include <queue>
#include <fstream>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>

namespace planner_fsm {
    using namespace Eigen;
    using namespace visualization_utils;
    using namespace std;


    class FSM {

    private:
        /// For UAV swarm
        bool been_selected_{true};

        bool stop{false};

        void SelectedPatchCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
            cout << GREEN << "[Planner] Drone <" << cfg_.drone_id << "> plannning module working well." << RESET
                 << endl;
            pcl::PointCloud<pcl::PointXYZ> cloud_input;
            pcl::fromROSMsg(*msg, (cloud_input));
            static vec_E<Vec3f> selected_pts_;
            selected_pts_.clear();
            been_selected_ = false;
            for (int i = 0; i < cloud_input.size(); i++) {
                selected_pts_.push_back(Vec3f(cloud_input.points[i].x,
                                              cloud_input.points[i].y,
                                              cloud_input.points[i].z));
                if ((robot_state_.p - selected_pts_[i]).norm() < 0.3) {
                    been_selected_ = true;
                    cout << YELLOW << "[Planner] Drone <" << cfg_.drone_id << ">have been selected!" << RESET << endl;
                    break;
                }
            }
        }

        mutex log_mtx;
        ofstream log_writer;
        string log_name{"error.bin"};
        ros::Subscriber collision_sub;

        void PubCmdPolyTraj() {
            quadrotor_msgs::PolynomialTrajectory cmd_traj;
            planner_ptr_->GetCmdTraj(cmd_traj);
            mpc_cmd_pub_.publish(cmd_traj);
        }

    private:
        vector<string> log_time_str{"TIME_STAMPE", "EPX_TRAJ_FRONTEND",
                                    "EXP_TRAJ_OPT", "GENERATE_EXP_TRAJ",
                                    "BACK_TRAJ_FRONTEND", "BACK_TRAJ_OPT",
                                    "GENERATE_BACK_TRAJ", "TOTAL_REPLAN", "VISUALIZATION"};
        FsmConfig cfg_;
        // map, checker, planner
        super_planner::SuperPlanner::Ptr planner_ptr_;
        std::ofstream write_time_;
        vector<double> log_module_time;
        double yaw_{0}, yaw_dot_{0};
        // ros
        ros::NodeHandle nh_;
        ros::Subscriber select_sub_, odom_sub_, goal_sub_, rc_sub_, path_goal_sub_, collision_sub_;
        ros::Publisher cmd_pub, mpc_cmd_pub_, mkr_pub_, current_goal_pub_, current_goalpath_pub_;
        ros::Publisher path_pub_;
        ros::ServiceServer ring_radius_mode_srv_;
        ros::Timer execution_timer_, replan_timer_, cmd_timer_, current_goal_timer_;

        geometry_msgs::PoseStamped current_goal;

        RobotState robot_state_;

        // params
        bool started_{false}, plan_from_rest_{false};
        bool ring_radius_mode_enabled_{false};
        double default_robot_r_{0.30};
        double ring_robot_r_{0.25};

        struct GoalInfo {
            bool new_goal;
            Vec3f goal_p;
            double goal_yaw;
        } gi_;

        Eigen::Vector3d auto_pilot_vel_w_;

        // execution states
        enum MACHINE_STATE {
            INIT = 0,
            WAIT_GOAL,
            YAWING,
            GENERATE_TRAJ,
            FOLLOW_TRAJ,
            EMER_STOP
        };

        vector<string> MACHINE_STATE_STR{"INIT",
                                         "WAIT_GOAL",
                                         "YAWING",
                                         "GENERATE_TRAJ",
                                         "FOLLOW_TRAJ", "EMER_STOP"
        };


        MACHINE_STATE machine_state_{INIT};

    public:
        FSM() = default;

        ~FSM();

        typedef shared_ptr<FSM> Ptr;

        void init(const ros::NodeHandle &nh);

        void setGoal(const Vec3f &goal);

        void odomPlanTest(const Vec3f &pos, const Vec3f &vel) {
            stop = true;
            planner_ptr_->odomPlanTest(pos, vel, Vec3f(12.5, 8.5, 1.0));
        }


    private:
        /* Callback functions */
        bool finish_plan = false;
        ros::Time system_start_time_;
        quadrotor_msgs::PositionCommand pid_cmd_;
        quadrotor_msgs::MpcPositionCommand mpc_cmd_;

        bool traj_finish_{false};

        void WriteTimeToLog();

        void PubCmdCallback(const ros::TimerEvent &event);

        void ReplanTimerCallback(const ros::TimerEvent &event);

        inline void MainFsmCallback(const ros::TimerEvent &event);

        bool closeToGoal(const double &thresh_dis);

        inline void GoalCallback(const geometry_msgs::PoseStampedConstPtr &msg);

        inline void SetGoalPosiAndYaw(const Vec3f &g_p, const double &g_y);

        inline void RcCallback(const mavros_msgs::RCInConstPtr &msg);

        inline void ChangeState(const string &call_func, const MACHINE_STATE &new_state);

        void PathGoalCallback(const nav_msgs::PathConstPtr &msg);

        void PubCurrentGoalCallback(const ros::TimerEvent &event);

        bool SetRingRadiusModeCallback(std_srvs::SetBool::Request &req,
                                       std_srvs::SetBool::Response &res);

    };

}
