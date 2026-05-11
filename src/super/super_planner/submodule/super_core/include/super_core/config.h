//
// Created by yunfan on 2022/3/26.
//

#pragma once

#include <rog_map/config.h>
#include <traj_opt/config.h>

namespace super_planner {
    using namespace rog_map;
    using namespace traj_opt;
    using std::cout;
    using std::endl;

    class PlannerConfig {
    public:
        enum YawMode{
            YAW_TO_VEL = 1,
            YAW_TO_GOAL = 2
        };

        ROGMapConfig rog_map_cfg;
        TrajOptConfig exp_traj_cfg, back_traj_cfg;

        // Bool Params
        bool backup_traj_en;
        bool use_fov_cut, print_log;
        bool goal_vel_en,goal_yaw_en;
        bool visual_process;
        bool frontend_in_known_free;

        double resolution;
        double planning_horizon;
        double receding_dis;
        double safe_corridor_line_max_length;

        // Planning Params
        int obs_skip_num;
        double corridor_bound_dis, corridor_line_max_length;
        double replan_forward_dt;
        double sample_traj_dt;
        double robot_r;
        int iris_iter_num;

        double yaw_dot_max;
        // Yaw mode: 1 heading to velocity, 2 heading to goal
        int yaw_mode = YAW_TO_VEL;

        vec_E<Vec3i> seed_line_neighbour;

        template<class T>
        bool LoadParam(string param_name, T &param_value, T default_value) ;

        template<class T>
        bool LoadParam(string param_name, vector<T> &param_value, vector<T> default_value);

        ros::NodeHandle nh_;

        PlannerConfig() = default;

        PlannerConfig(const ros::NodeHandle &nh_priv);
    };
}
