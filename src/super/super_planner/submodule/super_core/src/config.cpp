#include <super_core/config.h>

namespace super_planner {
    using namespace traj_opt;

    template<class T>
    bool PlannerConfig::LoadParam(string param_name, T &param_value, T default_value) {
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
    bool PlannerConfig::LoadParam(string param_name, vector<T> &param_value, vector<T> default_value) {
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

    PlannerConfig::PlannerConfig(const ros::NodeHandle &nh_priv) {
        nh_ = nh_priv;
        exp_traj_cfg = TrajOptConfig(nh_, "exp_traj");
        back_traj_cfg = TrajOptConfig(nh_, "backup_traj");
        LoadParam("super_planner/print_log", print_log, false);
        LoadParam("super_planner/backup_traj_en", backup_traj_en, false);
        LoadParam("super_planner/goal_vel_en", goal_vel_en, false);
        LoadParam("super_planner/goal_yaw_en", goal_yaw_en, false);
        LoadParam("super_planner/visual_process", visual_process, false);
        LoadParam("super_planner/use_fov_cut", use_fov_cut, false);
        LoadParam("super_planner/frontend_in_known_free", frontend_in_known_free, false);
        LoadParam("super_planner/safe_corridor_line_max_length", safe_corridor_line_max_length, 3.0);
        LoadParam("super_planner/obs_skip_num", obs_skip_num, 1);
        LoadParam("super_planner/replan_forward_dt", replan_forward_dt, 0.3);
        LoadParam("super_planner/corridor_bound_dis", corridor_bound_dis, 3.0);
        LoadParam("super_planner/corridor_line_max_length", corridor_line_max_length, 3.0);
        LoadParam("super_planner/planning_horizon", planning_horizon, 10.0);
        LoadParam("super_planner/receding_dis", receding_dis, 5.0);
        LoadParam("super_planner/robot_r", robot_r, 0.3);
        LoadParam("super_planner/iris_iter_num", iris_iter_num, 1);
        LoadParam("super_planner/yaw_mode", yaw_mode, 1);
        LoadParam("super_planner/yaw_dot_max", yaw_dot_max, 3.14);
    }

    template bool PlannerConfig::LoadParam(string param_name, double &param_value, double default_value);

    template bool PlannerConfig::LoadParam(string param_name, int &param_value, int default_value);

    template bool PlannerConfig::LoadParam(string param_name, bool &param_value, bool default_value);

    template bool PlannerConfig::LoadParam(string param_name, string &param_value, string default_value);

    template bool
    PlannerConfig::LoadParam(string param_name, vector<double> &param_value, vector<double> default_value);

    template bool PlannerConfig::LoadParam(string param_name, vector<int> &param_value, vector<int> default_value);

    template bool PlannerConfig::LoadParam(string param_name, vector<bool> &param_value, vector<bool> default_value);
}