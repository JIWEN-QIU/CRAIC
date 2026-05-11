//
// Created by yunfan on 2022/3/26.
//

#ifndef MISSION_PLANNER_CONFIG
#define MISSION_PLANNER_CONFIG

#include "ros/ros.h"
#include "vector"
#include "string"
#include "ros/package.h"
#include <type_utils/common_type_name.h>

namespace mission_planner {

    using namespace std;
    using namespace type_utils;
    enum TriggerType {
        RVIZ_CLICK = 0,
        MAVROS_RC = 1,
        TARGET_ODOM = 2
    };

    enum CmdType {
        PATH = 0,
        WAYPOINT = 1
    };

    class MissionConfig {
    public:
        // Bool Params

        int start_trigger_type;
        int cmd_type;
        string path_pub_topic, goal_pub_topic, odom_topic, waypoints_file_name;
        vec_E<Vec3f> waypoints;
        vector<double> switch_dis_vec;
        double switch_dis;
        double odom_timeout;
        double publish_dt;

        double str2double(string s) {
            double d;
            stringstream ss;
            ss << s;
            ss >> setprecision(16) >> d;
            ss.clear();
            return d;
        }

        void LoadWaypoint(string file_name) {
            string package_path_ = ros::package::getPath("mission_planner");

            file_name = package_path_ + "/data/" + file_name;
            ifstream theFile(file_name);
            std::string line;
            Vec3f log;
            while (std::getline(theFile, line)) {
                std::vector<std::string> result;
                std::istringstream iss(line);
                for (std::string s; iss >> s;) {
                    result.push_back(s);
                }
                for (size_t i = 0; i < result.size() - 1; i++) {
                    log(i) = str2double(result[i]);
                }
                switch_dis_vec.push_back(str2double(result[result.size() - 1]));
                waypoints.push_back(log);
            }
        }

        template<class T>
        bool LoadParam(string param_name, T &param_value, T default_value) {
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
        bool LoadParam(string param_name, vector<T> &param_value, vector<T> default_value) {
            if (nh_.getParam(param_name, param_value)) {
                printf("\033[0;32m Load param %s succes: \033[0;0m", param_name.c_str());
                for (int i = 0; i < param_value.size(); i++) {
                    cout << param_value[i] << " ";
                }
                cout << endl;
                return true;
            } else {
                printf("\033[0;33m Load param %s failed, use default value: \033[0;0m", param_name.c_str());
                param_value = default_value;
                for (int i = 0; i < param_value.size(); i++) {
                    cout << param_value[i] << " ";
                }
                cout << endl;
                return false;
            }
        }

        ros::NodeHandle nh_;

        MissionConfig() {};

        MissionConfig(const ros::NodeHandle &nh_priv) {
            nh_ = nh_priv;

            LoadParam("start_trigger_type", start_trigger_type, 1);
            LoadParam("cmd_type", cmd_type, 1);
            LoadParam("switch_dis", switch_dis, 1.0);
            LoadParam("odom_timeout", odom_timeout, 0.1);
            LoadParam("publish_dt", publish_dt, 1.0);
            LoadParam("goal_pub_topic", goal_pub_topic, string("/planner/goal"));
            LoadParam("odom_topic", odom_topic, string("/lidar_slam/odom"));
            LoadParam("path_pub_topic", path_pub_topic, string("/planner/path_cmd"));
            LoadParam("waypoints_file_name", waypoints_file_name, string("a_working_waypoints.txt"));
            LoadWaypoint(waypoints_file_name);

            cout << GREEN << " -- [MISSION] Load " << waypoints.size() << " waypoints." << RESET << endl;
            for (int i = 0; i < waypoints.size(); i++) {
                cout << BLUE << " -- [MISSION] Waypoint " << i << " at (" << waypoints[i].x() << ", "
                     << waypoints[i].y() << ", " << waypoints[i].z() << ")" << " Switch dis = " << switch_dis_vec[i]
                     << RESET << endl;
            }
            string types[]{"RVIZ_CLICK", "MAVROS_RC"};
            string way_pt_types[]{"PATH", "WAYPOINTS"};
            cout << GREEN << " -- [MISSION] Trigger type " << types[start_trigger_type] << RESET << endl;
            cout << GREEN << " -- [MISSION] Command type " << way_pt_types[cmd_type] << RESET << endl;

        }

    };
}
#endif //PLANNER_CONFIG_HPP
