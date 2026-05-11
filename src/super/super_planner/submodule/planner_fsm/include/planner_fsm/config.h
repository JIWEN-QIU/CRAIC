//
// Created by yunfan on 2022/3/26.
//
#pragma once
#include "ros/ros.h"
#include "vector"
#include "string"
#include "rog_map/config.h"
#include "traj_opt/config.h"
#include "super_core/config.h"


namespace planner_fsm {
    using namespace rog_map;
    using namespace traj_opt;
    using namespace super_planner;
    static const int MPC_PVAJ_MODE = 1;
    static const int MPC_POLYTRAJ_MODE = 2;

    class FsmConfig {
    public:
        PlannerConfig planner_cfg;

        // For UAV swarm
        int drone_id;
        bool swarm_en;

        // FSM Params
        bool auto_pilot_en;
        double auto_pilot_safe_dis;
        bool safe_auto_pilot;
        bool click_goal_en;
        bool path_goal_en;
        double replan_rate;
        double yawing_speed;
        double click_height;
        double auto_pilot_speed, auto_pilot_yaw_speed;
        vector<double> pos_gain, vel_gain;
        bool click_yaw_en;
        string cmd_topic, mpc_cmd_topic, path_goal_topic, click_goal_topic;
        int mpc_cmd_type;

        template<class T>
        bool LoadParam(string param_name, T &param_value, T default_value);

        template<class T>
        bool LoadParam(string param_name, vector<T> &param_value, vector<T> default_value);

        ros::NodeHandle nh_;

        FsmConfig() =default;

        FsmConfig(const ros::NodeHandle &nh_priv);
    };
}
