#include <planner_fsm/config.h>

namespace planner_fsm {

    template<class T>
    bool FsmConfig::LoadParam(string param_name, T &param_value, T default_value) {
        if (nh_.getParam(param_name, param_value)) {
            printf("\033[0;32m Load param %s succes: \033[0;0m", param_name.c_str());
            cout << param_value << endl;
            return true;
        } else {
            printf("\033[0;33m Load param %s failed, use default value: \033[0;0m", param_name.c_str());
            param_value = default_value;
            cout << param_value << endl;
            return false;
        }
    }


    template<class T>
    bool FsmConfig::LoadParam(string param_name, vector<T> &param_value, vector<T> default_value) {
        if (nh_.getParam(param_name, param_value)) {
            printf("\033[0;32m Load param %s succes: \033[0;0m", param_name.c_str());
            for (long unsigned int i = 0; i < param_value.size(); i++) {
                cout << param_value[i] << " ";
            }
            cout << endl;
            return true;
        } else {
            printf("\033[0;33m Load param %s failed, use default value: \033[0;0m", param_name.c_str());
            param_value = default_value;
            for (long unsigned int i = 0; i < param_value.size(); i++) {
                cout << param_value[i] << " ";
            }
            cout << endl;
            return false;
        }
    }

    FsmConfig::FsmConfig(const ros::NodeHandle &nh_priv) {
        nh_ = nh_priv;
        vector<double> tem_gain;
        LoadParam("fsm/drone_id", drone_id, 1);
        LoadParam("fsm/auto_pilot_en", auto_pilot_en, false);
        LoadParam("fsm/swarm_en", swarm_en, false);
        LoadParam("fsm/safe_auto_pilot", safe_auto_pilot, false);
        LoadParam("fsm/auto_pilot_safe_dis", auto_pilot_safe_dis, 0.0);
        LoadParam("fsm/path_goal_en", path_goal_en, false);
        LoadParam("fsm/click_goal_en", click_goal_en, false);
        LoadParam("fsm/click_yaw_en", click_yaw_en, false);
        LoadParam("fsm/replan_rate", replan_rate, 10.0);
        LoadParam("fsm/click_height", click_height, 1.5);
        LoadParam("fsm/yawing_speed", yawing_speed, 0.3);
        LoadParam("fsm/auto_pilot_speed", auto_pilot_speed, 3.0);
        LoadParam("fsm/auto_pilot_yaw_speed", auto_pilot_yaw_speed, 1.0);
        LoadParam("fsm/cmd_topic", cmd_topic, string("/planning/pos_cmd"));
        LoadParam("fsm/mpc_cmd_topic", mpc_cmd_topic, string("/planning_cmd/mpc"));
        LoadParam("fsm/mpc_cmd_type", mpc_cmd_type, MPC_PVAJ_MODE);

        LoadParam("fsm/click_goal_topic", click_goal_topic, string("/planning/click_goal_topic"));
        LoadParam("fsm/path_goal_topic", path_goal_topic, string("/planning/path_goal"));
        planner_cfg = PlannerConfig(nh_priv);
        rog_map::ROSParamLoader(nh_priv, planner_cfg.rog_map_cfg);
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
    }

    template bool FsmConfig::LoadParam(string param_name, bool &param_value, bool default_value);

    template bool FsmConfig::LoadParam(string param_name, int &param_value, int default_value);

    template bool FsmConfig::LoadParam(string param_name, double &param_value, double default_value);

    template bool FsmConfig::LoadParam(string param_name, string &param_value, string default_value);

    template bool FsmConfig::LoadParam(string param_name, vector<bool> &param_value, vector<bool> default_value);

    template bool FsmConfig::LoadParam(string param_name, vector<int> &param_value, vector<int> default_value);

    template bool FsmConfig::LoadParam(string param_name, vector<double> &param_value, vector<double> default_value);
}