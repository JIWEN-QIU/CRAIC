#include <traj_opt/config.h>

using namespace traj_opt;

template<typename T>
bool TrajOptConfig::LoadParam(string param_name, T &param_value, T default_value) {
    if (nh_.getParam(param_name, param_value)) {
        printf("\033[0;32m Load param %s succes: \033[0;0m", param_name.c_str());
        std::cout << param_value << std::endl;
        return true;
    } else {
        printf("\033[0;33m Load param %s failed, use default value: \033[0;0m", param_name.c_str());
        param_value = default_value;
        std::cout << param_value << std::endl;
        return false;
    }
}

template<typename T>
bool TrajOptConfig::LoadParam(string param_name, vector<T> &param_value, vector<T> default_value) {
    if (nh_.getParam(param_name, param_value)) {
        printf("\033[0;32m Load param %s succes: \033[0;0m", param_name.c_str());
        for (long unsigned int i = 0; i < param_value.size(); i++) {
            std::cout << param_value[i] << " ";
        }
        std::cout << std::endl;
        return true;
    } else {
        printf("\033[0;33m Load param %s failed, use default value: \033[0;0m", param_name.c_str());
        param_value = default_value;
        for (long unsigned int i = 0; i < param_value.size(); i++) {
            std::cout << param_value[i] << " ";
        }
        std::cout << std::endl;
        return false;
    }
}

TrajOptConfig::TrajOptConfig(const ros::NodeHandle
                             &nh_priv,
                             string ns) {
    nh_ = nh_priv;
    if (ns.empty()) {
        ns = "/";
    } else {
        ns = "/" + ns + "/";
    }

    LoadParam("traj_opt/switch/print_optimizer_log", print_optimizer_log, false);
    // For swarm collision avoidance.
    LoadParam("traj_opt/swarm/enable", swarm_collision_en, false);
    LoadParam("traj_opt/swarm/swarm_clearance", swarm_clearance, -1.0);
    LoadParam("traj_opt/swarm/penna_swarm", penna_swarm, -1.0);

    /// Load Param for Flatness
    LoadParam("traj_opt/flatness/mass", mass, 1.0);
    LoadParam("traj_opt/flatness/dh", dh, 0.7);
    LoadParam("traj_opt/flatness/dv", dv, 0.8);
    LoadParam("traj_opt/flatness/grav", grav, 1.0);
    LoadParam("traj_opt/flatness/cp", cp, 0.01);
    LoadParam("traj_opt/flatness/v_eps", v_eps, 0.0001);

    LoadParam("traj_opt/switch/save_log_en", save_log_en, false);
    LoadParam("traj_opt" + ns + "pos_constraint_type", pos_constraint_type, 2);
    LoadParam("traj_opt" + ns + "piece_num", piece_num, 1);
    LoadParam("traj_opt" + ns + "block_energy_cost", block_energy_cost, false);
    LoadParam("traj_opt" + ns + "opt_accuracy", opt_accuracy, 1.0e-5);
    LoadParam("traj_opt" + ns + "integral_reso", integral_reso, 10);
    LoadParam("traj_opt" + ns + "smooth_eps", smooth_eps, 0.01);
    LoadParam("traj_opt/boundary/max_vel", max_vel, -1.0);
    LoadParam("traj_opt/boundary/max_acc", max_acc, -1.0);
    LoadParam("traj_opt/boundary/max_jerk", max_jerk, -1.0);
    LoadParam("traj_opt/boundary/max_omg", max_omg, -1.0);
    LoadParam("traj_opt/boundary/max_acc_thr", max_acc_thr, -1.0);
    LoadParam("traj_opt/boundary/min_acc_thr", min_acc_thr, -1.0);
    LoadParam("traj_opt/boundary/penna_margin", penna_margin, 0.05);

    LoadParam("traj_opt" + ns + "penna_scale", penna_scale, -1.0);
    LoadParam("traj_opt" + ns + "penna_t", penna_t, -1.0);
    LoadParam("traj_opt" + ns + "penna_ts", penna_ts, -1.0);
    LoadParam("traj_opt" + ns + "penna_pos", penna_pos, -1.0);
    LoadParam("traj_opt" + ns + "penna_vel", penna_vel, -1.0);
    LoadParam("traj_opt" + ns + "penna_acc", penna_acc, -1.0);
    LoadParam("traj_opt" + ns + "penna_jerk", penna_jerk, -1.0);
    LoadParam("traj_opt" + ns + "penna_attract", penna_attract, -1.0);
    LoadParam("traj_opt" + ns + "penna_omg", penna_omg, -1.0);
    LoadParam("traj_opt" + ns + "penna_thr", penna_thr, -1.0);

    if (penna_scale > 0) {
        penna_t = penna_t * penna_scale;
        penna_ts = penna_ts * penna_scale;
        penna_pos = penna_pos * penna_scale;
        penna_vel = penna_vel * penna_scale;
        penna_acc = penna_acc * penna_scale;
        penna_jerk = penna_jerk * penna_scale;
        penna_attract = penna_attract * penna_scale;
        penna_omg = penna_omg * penna_scale;
        penna_thr = penna_thr * penna_scale;
    }

    quadrotot_flatness.reset(mass, grav, dh, dv, cp, v_eps);
}

template bool TrajOptConfig::LoadParam(string param_name, string &param_value, string default_value);

template bool TrajOptConfig::LoadParam(string param_name, double &param_value, double default_value);

template bool TrajOptConfig::LoadParam(string param_name, vector<double> &param_value, vector<double> default_value);

template bool TrajOptConfig::LoadParam(string param_name, int &param_value, int default_value);

template bool TrajOptConfig::LoadParam(string param_name, vector<int> &param_value, vector<int> default_value);

template bool TrajOptConfig::LoadParam(string param_name, bool &param_value, bool default_value);

template bool TrajOptConfig::LoadParam(string param_name, vector<bool> &param_value, vector<bool> default_value);