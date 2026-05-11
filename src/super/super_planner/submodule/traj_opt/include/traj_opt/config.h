#pragma once

#include <string>
#include <type_utils/common_type_name.h>
#include <geometry_utils/quadrotor_flatness.hpp>
#include <ros/ros.h>
#include <omp.h>
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "log/"+(name)))
namespace traj_opt {
    using std::string;
    using std::vector;

    enum PosConstrainType {
        WAYPOINT = 1,
        CORRIDOR = 2,
    };

    class TrajOptConfig {
    public:
        flatness::FlatnessMap quadrotot_flatness;
        // Swarm collision term enable.
        bool swarm_collision_en{false};
        bool print_optimizer_log{false};
        double swarm_clearance{-1};
        double penna_swarm{-1};

        /// Param for flatness
        double mass, dh, dv, grav, cp, v_eps;

        // if save the optimization problem to log
        bool save_log_en{false};

        int pos_constraint_type{CORRIDOR};
        // Set to true for only min time.
        bool block_energy_cost{false};
        // Limit conditions.
        double max_vel{0}, max_acc{0}, max_jerk{0}, max_omg{0}, max_acc_thr{0}, min_acc_thr{0};
        // Penalty cost.
        double penna_scale{-1}, penna_vel{0}, penna_acc{0}, penna_jerk{0}, penna_omg{0}, penna_thr{0};
        // penna_t; penna_pos only for corridor based method.
        double penna_t{0}, penna_pos{0}, penna_attract{0};
        // penna_ts only for backupTraj;
        double penna_ts{0};
        // for backup traj piece num
        int piece_num{0};

        double penna_margin{0.05};

        double smooth_eps{0};
        int integral_reso{0};
        double opt_accuracy{0};


        template<typename T>
        bool LoadParam(string param_name, T &param_value, T default_value);

        template<typename T>
        bool LoadParam(string param_name, vector<T> &param_value, vector<T> default_value);

        ros::NodeHandle nh_;

        TrajOptConfig() = default;

        explicit TrajOptConfig(const ros::NodeHandle &nh_priv, string ns = "");

    };
}